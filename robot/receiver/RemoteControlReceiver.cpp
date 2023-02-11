#include "RemoteControlReceiver.hpp"
#include <boost/bind.hpp>
#include <string>
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include "blackboard/Blackboard.hpp"

std::string tokensToString(std::vector<std::string> tokens);

/*
Syntax for the receiver:
Expects messages of the form:
forwardsVelocity=<value> leftVelocity=<value> turnVelocity=<value>

Note key-pairs are seperated by spaces
values are chars '0'->'9' and '.'

e.g.
forwardsVelocity=1.02 leftVelocity=0 turnVelocity=0.123
*/

RemoteControlReceiver::RemoteControlReceiver(Blackboard *bb)
    : Adapter(bb), io_service_(), socket_(io_service_, boost::asio::ip::udp::v4())
{
    socket_.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), REMOTE_CONTROL_PORT));
    startReceive();
    t = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

RemoteControlReceiver::~RemoteControlReceiver()
{
    io_service_.stop();
    t->join();
    delete t;
    if (socket_.is_open())
    {
        socket_.close();
    }
}

void RemoteControlReceiver::startReceive()
{
    socket_.async_receive_from(
        boost::asio::buffer((boost::asio::mutable_buffer((void *)&recvBuffer,
                                                         sizeof(recvBuffer)))),
        remote_endpoint_,
        boost::bind(&RemoteControlReceiver::remoteControlHandler, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void RemoteControlReceiver::remoteControlHandler(const boost::system::error_code &ec, std::size_t size)
{
    startReceive();

    std::map<std::string, std::string> commands = this->ProcessInput(recvBuffer);
    double forwards = atof(commands["forwardsVelocity"].c_str());
    double left = atof(commands["leftVelocity"].c_str());
    double turn = atof(commands["turnVelocity"].c_str());
    bool kick = commands["kick"] == "True";

    writeActionToBlackboard(kick, forwards, left, turn);
}

void RemoteControlReceiver::tick()
{
}

std::map<std::string, std::string> RemoteControlReceiver::ProcessInput(const char *str)
{
    const std::string tokenDelim = " ";
    const std::string keyValueDelim = "=";

    std::map<std::string, std::string> command;
    std::string rawData(str);

    //Tokenise the input into key=value pairs
    std::vector<std::string> tokens;
    boost::split(tokens, rawData, boost::is_any_of(tokenDelim));

    for (std::vector<std::string>::iterator token = tokens.begin(); token != tokens.end(); token++)
    {
        //split up the key and value
        std::vector<std::string> keyValuePair;
        keyValuePair.clear();
        boost::split(keyValuePair, *token, boost::is_any_of(keyValueDelim));
        if (keyValuePair.size() == 2)
        {
            //if (keyValuePair.length() == 2 && ALLOWED_KEYS.contains(keyValuePair[0])) {
            std::string value;
            std::string key = keyValuePair[0];
            std::stringstream valStr(keyValuePair[1]);
            valStr >> value;
            command.insert(std::pair<std::string, std::string>(key, value));
        }
        else
        {
            llog(VERBOSE) << "RemoteControlReceiver: Warning: malformed key-value pair: " << *token
                          << " size was" << keyValuePair.size()
                          << " tokenised as" << tokensToString(keyValuePair);
        }
    }
    return command;
}

void RemoteControlReceiver::writeActionToBlackboard(bool kick, double forwards, double left, double turn) {

    ActionCommand::Body bodyCommand;
    if (kick) {
        bodyCommand = ActionCommand::Body(ActionCommand::Body::KICK);
    } else {
        if ((fabs(forwards) < DEADZONE_LINEAR) && (fabs(left) < DEADZONE_LINEAR) && (fabs(turn) < DEADZONE_TURN)) {
            //set bend to 0 to have straight legs
            bodyCommand = ActionCommand::Body(ActionCommand::Body::WALK, 0, 0, 0, 0.1, 0);
        } else {
            bodyCommand = ActionCommand::Body(ActionCommand::Body::WALK,
                                                forwards,
                                                left,
                                                turn);
        }
    }

    BehaviourRequest request;
    request.actions.body = bodyCommand;

    time_t curTime;
    time(&curTime);
    writeTo(remoteControl, time_received, curTime);
    writeTo(remoteControl, request, request);
}

std::string tokensToString(std::vector<std::string> tokens)
{
    std::ostringstream out;
    for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
    {
        out << *it << " ";
    }
    return out.str();
}
