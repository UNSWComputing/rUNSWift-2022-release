#include "SimulationConnection.hpp"

#include <boost/thread/thread.hpp>

#include <iomanip>
#include <sstream>
#include <vector>

// TODO debug
#include <iostream>

namespace Simulation
{
// Helper template for implementing anything options.
   template<int Level, int Name, typename T>
   class anything {
   public:
      // Construct with a specific option value.
      explicit anything(T *v)
         : value_(v) {
      }

      // Set the value of the int option.
      anything &operator=(T *v) {
         value_ = v;
         return *this;
      }

      // Get the current value of the T* option.
      T *value() const {
         return value_;
      }

      // Get the level of the socket option.
      template<typename Protocol>
      int level(const Protocol &) const {
         return Level;
      }

      // Get the name of the socket option.
      template<typename Protocol>
      int name(const Protocol &) const {
         return Name;
      }

      // Get the address of the T data.
      template<typename Protocol>
      T *data(const Protocol &) {
         return value_;
      }

      // Get the address of the T data.
      template<typename Protocol>
      const T *data(const Protocol &) const {
         return value_;
      }

      // Get the size of the T data.
      template<typename Protocol>
      std::size_t size(const Protocol &) const {
         return sizeof(*value_);
      }

      // Set the size of the T* data.
      template<typename Protocol>
      void resize(const Protocol &, std::size_t s) {
         if (s != sizeof(*value_)) {
            std::length_error ex("anything socket option resize");
            boost::asio::detail::throw_exception(ex);
         }
      }

   private:
      T *value_;
   };

    SimulationConnection::SimulationConnection(
        std::string hostIP,
        unsigned int port,
        std::string team_name,
        int player_num,
        double starting_x,
        double starting_y,
        double starting_theta)
      //   double starting_ball_x,
      //   double starting_ball_y)
        : io_service_()
        , socket_(io_service_)
        , host_ip_(hostIP)
        , port_(port)
        , player_num_(player_num)
        , starting_x_(starting_x)
        , starting_y_(starting_y)
        , starting_theta_(starting_theta)
      //   , starting_ball_x_(starting_ball_x)
      //   , starting_ball_y_(starting_ball_y)
    {
        connectSocketToServer();

        PerceptorInfo info;
        std::stringstream ss;

        // Create player (Create Effector)
        ss << "(scene rsg/agent/nao/nao.rsg)";
        ss << "(syn)";
        sendString(ss.str());
        receivePerceptorInfo(&info);

        // Init player (Init Effector)
        ss.str("");
        ss << "(init (unum " << player_num << ")(teamname " << team_name << "))";
        ss << "(syn)";
        sendString(ss.str());
        receivePerceptorInfo(&info);

        // Beam player (Beam Effector)
        ss.str("");
        ss << "(beam " << starting_x_ << " " << starting_y_ << " " << starting_theta_ << ")";
        ss << "(syn)";
        sendString(ss.str());
    }

    SimulationConnection::~SimulationConnection()
    {
        closeSocketToServer();
    }

    bool SimulationConnection::sendEffectorCommand(const EffectorCommand &cmd)
    {
        std::ostringstream ss;
        ss << cmd.joints;
        ss << "(syn)";
        return sendString(ss.str());
    }

    bool SimulationConnection::sendString(const std::string& msg)
    {
        //std::cout << "\nSending: " << msg << "\n";
        unsigned int len = htonl(msg.size());
        std::string prefix((const char*)&len, sizeof(unsigned int));
        std::string to_send = prefix + msg;
        socket_.send(boost::asio::buffer(to_send));
        return true;
    }

    bool SimulationConnection::receivePerceptorInfo(PerceptorInfo* info_out)
    {
        boost::system::error_code error;
        uint32_t len = 0;

        size_t recv = socket_.read_some(boost::asio::buffer((char*)&len, 4), error);
        if (error)
        {
            return false;
        }
        len = ntohl(len);

        std::vector<char> buf(len);
        recv = socket_.read_some(boost::asio::buffer(buf), error);
        if (error)
        {
            return false;
        }

        std::string msg (buf.data(), static_cast<unsigned int>(recv));
        if (len != recv)
        {
            return false;
        }

        return info_out->fromString(msg);
     }

     void SimulationConnection::connectSocketToServer()
     {
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(boost::asio::ip::tcp::resolver::query(host_ip_, boost::lexical_cast<std::string>(port_)));
        boost::asio::connect(socket_, endpoint);

        struct timeval tv = {1, 0};
        const anything<SOL_SOCKET, SO_RCVTIMEO, struct timeval> option =
           anything<SOL_SOCKET, SO_RCVTIMEO, struct timeval>(&tv);
        socket_.set_option(option);
     }

     void SimulationConnection::closeSocketToServer()
     {
        boost::system::error_code ec;
        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        socket_.close();
     }

} // End namespace Simulation
