#ifndef SIM_EX_TRANSMITTER_HPP
#define SIM_EX_TRANSMITTER_HPP

#include <blackboard/Blackboard.hpp>
#include <blackboard/Adapter.hpp>
#ifndef Q_MOC_RUN
#include <boost/asio.hpp>
#endif

class SimExTransmitter : Adapter
{
  public:
    SimExTransmitter(Blackboard *bb);

    ~SimExTransmitter();

    void tick();

  private:
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;

    void sendString(std::string& msg);

};

#endif // SIM_EX_TRANSMITTER_HPP
