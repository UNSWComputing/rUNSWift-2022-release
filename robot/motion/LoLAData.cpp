#include "LoLAData.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/local/stream_protocol.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <msgpack.hpp>

#include "utils/speech.hpp"

using namespace std;
using namespace boost::asio;

namespace LoLAData {
   bool isConnected = false;

   local::stream_protocol::socket &getSocket() {
      static local::stream_protocol::socket *socket = nullptr;
      if (!isConnected) {
         // in case daemon.py is running, tell it to release lola
         system("/usr/bin/pkill --signal USR1 daemon.py");
         // disconnect
         if (socket != nullptr) {
            delete socket;
            socket = nullptr;
         }

         // connect
         if (socket == nullptr) {
            static io_service ioservice;
            socket = new local::stream_protocol::socket(ioservice);
         }
         boost::system::error_code ec;
         socket->connect("/tmp/robocup", ec);
         isConnected = !ec;
         if (ec) {
            SAY("Could not connect to LoLA: " + ec.message());
         } else {
            // in case say.py hangs around, don't let it keep the port open
            fcntl(socket->native_handle(), F_SETFD, FD_CLOEXEC);
         }
      }

      return *socket;
   }

   const map<string, vector<msgpack::v2::object> > read() {
      static const int          MSGPACK_READ_LENGTH = 896;
      static char               data[MSGPACK_READ_LENGTH];
      boost::system::error_code ec;
      LoLAData::getSocket().receive(buffer(data), 0, ec);
      isConnected = !ec;
      if (!ec) {
         msgpack::v2::object_handle objectHandle = msgpack::unpack(data, MSGPACK_READ_LENGTH);
         msgpack::v2::object        obj          = objectHandle.get();
         return obj.as<map<string, vector<msgpack::v2::object> > >();
      } else {
         SAY("Could not read from LoLA: " + ec.message());
         return map<string, vector<msgpack::v2::object> >();
      }
   }

   template<typename T>
   void write(const map<string, vector<T>> &map) {
      // Send off map
      boost::asio::streambuf sbMap;
      ostream                osMap(&sbMap);
      msgpack::pack(osMap, map);
      boost::system::error_code ec;
      boost::asio::write(getSocket(), sbMap, ec);
      isConnected = !ec;
   }

   template
   void write(const map <string, vector<float>> &);

   template
   void write(const map <string, vector<bool>> &);

   // shared between touch and effector
   bool          head_limp    = false, limp = true, standing = false;
   SensorValues  sensors      = SensorValues(true);
   vector<float> targetAngles = vector<float>();
}
