#include "SimExTransmitter.hpp"

using namespace boost::asio::ip;

/*
 * SimExTransmitter is a Synchronous Transmitter 
 * Sends strings to sim-scripts (for simulation)
 * 
 * https://www.boost.org/doc/libs/1_63_0/doc/html/boost_asio/example/cpp03/echo/blocking_tcp_echo_client.cpp
 * 
 */

SimExTransmitter::SimExTransmitter(Blackboard *bb)
    : Adapter(bb)
    , io_service_()
    , socket_(io_service_)
{
    std::string host = "localhost";
    std::string port = "2000";

    tcp::resolver resolver(io_service_);
    tcp::resolver::query query(host, port);
    tcp::resolver::iterator iterator = resolver.resolve(query);

    boost::asio::connect(socket_, iterator);
}

SimExTransmitter::~SimExTransmitter()
{
    
}

void SimExTransmitter::tick()
{
    std::string msg = "(test 1)"; // example message
    sendString(msg);
}

void SimExTransmitter::sendString(std::string& msg)
{
    unsigned int len = htonl(msg.size());
    std::string prefix((const char*)&len, sizeof(unsigned int));
    std::string to_send = prefix + msg;
    socket_.send(boost::asio::buffer(to_send));
}