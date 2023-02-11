#ifndef REMOTE_CONTROL_RECEIVER_HPP
#define REMOTE_CONTROL_RECEIVER_HPP

#include <iostream>
#ifndef Q_MOC_RUN
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#endif
#include <string>
#include <map>
#include "blackboard/Adapter.hpp"

class Blackboard;

#define MAX_MESSAGE_LEN 2000
#define REMOTE_CONTROL_PORT 2000
#define DEADZONE_LINEAR 1
#define DEADZONE_TURN 0.001

class RemoteControlReceiver : Adapter
{
  public:
    RemoteControlReceiver(Blackboard *bb);
    ~RemoteControlReceiver();

    void startReceive();
    void remoteControlHandler(const boost::system::error_code &ec, std::size_t size);
    void tick();

  private:
    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;

    boost::thread *t;

    char recvBuffer[MAX_MESSAGE_LEN]; // We probably don't need more than 10 bytes for now
    std::map<std::string, std::string> ProcessInput(const char *str);
    void writeActionToBlackboard(bool kick, double forwards, double left, double turn);
};

#endif //REMOTE_CONTROL_RECEIVER_HPP
