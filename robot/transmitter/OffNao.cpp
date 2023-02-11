#include "OffNao.hpp"
#include <map>
#include <vector>

#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"

using namespace std;
using boost::asio::ip::tcp;
using namespace boost;
namespace po = boost::program_options;

OffNaoTransmitter::OffNaoTransmitter(Blackboard *bb)
    : Adapter(bb), port_(10125) {

   if (bb->config.count("simulation"))
      // If we're running in simulation, modify the port with the team number
      // and player number, so we don't have port conflicts when we run multiple
      // instances of runswift
      port_ += ((bb->config)["player.team"].as<int>() * MAX_NUM_PLAYERS) + (bb->config)["player.number"].as<int>();

   start_accept();
}
extern __u32 controlIds[NUM_CONTROLS];
static const char* ControlNames[NUM_CONTROLS] = {
   "hflip",
   "vflip",
   "autoexp",
   "brightness",
   "contrast",
   "saturation",
   "hue",
   "sharpness",
   "autowb",
   "backlightcompensation",
   "autoexp",
   "exposure",
   "gain",
   "whitebalance"
};
static const int cNUM_CAM_CONTROLS = sizeof(ControlNames)/sizeof(char *);

void OffNaoTransmitter::start_accept(){
   try {
      tcp::endpoint endpoint(tcp::v4(), port_);
      acceptor_ = new tcp::acceptor(io_service_, endpoint);
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));

      acceptor_->async_accept(new_session->socket(),
                              boost::bind(&OffNaoTransmitter::handle_accept,
                                          this, new_session,
                                          boost::asio::placeholders::error));
      std::cout << "Listening for Offnao on port " << port_ << ".\n";
      // in case say.py hangs around, don't let it keep the port open
      fcntl(acceptor_->native_handle(), F_SETFD, FD_CLOEXEC);
   } catch (const std::exception& e) {
      cerr << "Offnao exception: " << e.what() << "\n";
   }

}

void OffNaoTransmitter::tick() {
   llog(VERBOSE) << "ticking away" << endl;
   io_service_.poll();
   room_.deliver(blackboard);
   io_service_.poll();
}

OffNaoTransmitter::~OffNaoTransmitter() {
   delete acceptor_;
}

void OffNaoTransmitter::handle_accept(offnao_session_ptr session,
                                      const boost::system::error_code& error) {
   if (!error) {
      offnao_session_ptr new_session(new offnao_session(&io_service_, &room_));
         acceptor_->async_accept(new_session->socket(),
                              boost::bind(&OffNaoTransmitter::handle_accept,
                                          this, new_session,
                                          boost::asio::placeholders::error));
      session->start(blackboard);
      session->socket().set_option(tcp::no_delay(true));
      session->socket().non_blocking(false);
      llog(DEBUG1) << "Created wireless link." << endl;
   } else {
      llog(ERROR) << error.message() << endl;
      start_accept();
   }
}

OffNaoTransmitter::offnao_session::
offnao_session(boost::asio::io_service* io_service, offnao_room *room)
   : connection_(io_service), room_(*room), sendingMask(INITIAL_MASK) {}

tcp::socket& OffNaoTransmitter::offnao_session::socket() {
   return connection_.socket();
}

void OffNaoTransmitter::offnao_session::start(Blackboard *blackboard) {
   room_.join(shared_from_this());
   connection_.async_read((ProtobufSerialisable&)commands,
                          boost::bind(&offnao_session::handle_read,
                                      shared_from_this(),
                                      boost::asio::placeholders::error, blackboard));
}

void OffNaoTransmitter::offnao_session::deliver(Blackboard *blackboard) {
   // mask set before delivery to attempt to allow multiple clients to stream different things
   // writeTo(, mask, sendingMask);
   blackboard->write(&(blackboard->mask), sendingMask);
   /*
   connection_.async_write(*blackboard,
                           boost::bind(&offnao_session::handle_write,
                                       shared_from_this(),
                                       boost::asio::placeholders::error));
   */
   boost::system::error_code e;
   if (sendingMask & USE_BATCHED_MASK) {
        e = connection_.batch_write((ProtobufSerialisable&)*blackboard);
   } else {
        e = connection_.sync_write((ProtobufSerialisable&)*blackboard);
   }
   if (e)
      llog(ERROR) << "Failed to write: " << e << endl;
}

void OffNaoTransmitter::offnao_room::join(offnao_participant_ptr participant) {
   participants_.insert(participant);
}

void OffNaoTransmitter::offnao_room::
leave(offnao_participant_ptr participant) {
   participants_.erase(participant);
}

void OffNaoTransmitter::offnao_room::deliver(Blackboard *blackboard) {
   for_each(participants_.begin(), participants_.end(),
            boost::bind(&offnao_participant::deliver, _1, boost::ref(blackboard)));
}

void OffNaoTransmitter::offnao_session::
handle_write(boost::system::error_code const& error) {
   if (error)
      room_.leave(shared_from_this());
}

void OffNaoTransmitter::offnao_session::
handle_read(boost::system::error_code const& error, Blackboard *blackboard) {
   if (!error) {
      llog(INFO) << "Received commands " << commands << endl;
      if (commands.hasArgV()) {
            po::variables_map vm;
            try {
               po::options_description cmdline_options = store_and_notify(commands.getArgV(), vm);
               blackboard->config = vm;
               options_print(vm);

               for (map<string, boost::function<void(const po::variables_map &)> >::const_iterator ci = readFrom(thread, configCallbacks).begin();
                    ci != readFrom(thread, configCallbacks).end(); ++ci)
                  if (!ci->second.empty()) {
                     ci->second(vm);
                  }
            } catch (program_options::error& e) {
               llog(WARNING) << "Error when parsing command line arguments: " <<
               e.what() << endl;
            }
      }
      if (commands.hasSendingMask()) {
         sendingMask = commands.getSendingMask();
      }
      connection_.async_read((ProtobufSerialisable&)commands,
                             boost::bind(&offnao_session::handle_read,
                                         shared_from_this(),
                                         boost::asio::placeholders::error, blackboard));
   } else {
      llog(DEBUG1) << "Received data with error!!!" << std::endl;
      room_.leave(shared_from_this());
   }
}
