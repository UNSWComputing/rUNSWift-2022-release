#ifndef SIMULATION_SIMULATIONCONNECTION_H_
#define SIMULATION_SIMULATIONCONNECTION_H_

#include "PerceptorInfo.hpp"
#include "EffectorCommand.hpp"

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <string>

namespace Simulation
{
    /*
     * The SimulationConnection class handles network IO with the simulator server.
     */
    class SimulationConnection
    {
    public:

        /*
         *  Constructor, initialise connection to the server and places robot
         */
        SimulationConnection(const std::string host,
                             unsigned int port,
                             std::string team_name,
                             int player_num,
                             double starting_x,
                             double starting_y,
                             double starting_theta);
                            //  double starting_ball_x,
                            //  double starting_ball_y);

        ~SimulationConnection();

        /*
         *  Sends effector command to the server
         */
        bool sendEffectorCommand(const EffectorCommand& cmd);

        /*
         *  Receives perceptor information from the serevr
         */
        bool receivePerceptorInfo(PerceptorInfo* info_out);
        bool sendString(const std::string&);

    private:

        void connectSocketToServer();
        void closeSocketToServer();

        boost::asio::io_service io_service_;
        boost::asio::ip::tcp::socket socket_;

        const std::string host_ip_;
        unsigned int port_;

        std::string team_name_;
        int player_num_;
        float starting_x_;
        float starting_y_;
        float starting_theta_;
        // float starting_ball_x_;
        // float starting_ball_y_;
    };
} // End namespace Simulation

#endif //SIMULATION_SIMULATIONCONNECTION_H_

