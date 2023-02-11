#ifndef SIMULATION_SIMULATIONTHREAD_H_
#define SIMULATION_SIMULATIONTHREAD_H_

#include "AngleSensor.hpp"
#include "SimulationConnection.hpp"
#include "SimVisionAdapter.hpp"
#include "utils/Timer.hpp"
#include "blackboard/Adapter.hpp"
#include "libagent/AgentData.hpp"

#include <semaphore.h>
#include <fstream>

using namespace Simulation;

/* Wrapper class for simulator thread */
class SimulationThread : Adapter {
public:
    /* Constructor */
    explicit SimulationThread(Blackboard *bb);

    /* Destructor */
    ~SimulationThread();

    /* One cycle of this thread */
    void tick();

private:

    /*
     * Relaunch robot, under circumstances.
     * This can be used for repeated testing.
     */
    bool shouldRestart(SensorValues& s);

    int shared_fd_;
    sem_t* semaphore_;
    AgentData* shared_data_;
    SimulationConnection* connection_;               /**< Handles TCP connection to simulation server */
    SimVisionAdapter vision_;

    Simulation::Joints current_error_;
    Simulation::Joints previous_error_;
    Simulation::Joints cumulative_error_;
    JointValues perception_joint_angles_;
    AngleSensor angle_;

    int team_number_;                          // Team Number
    std::string team_name_;                    // Team Name
    int player_number_;                        // Player Number

    float starting_x_;
    float starting_y_;
    float starting_theta_;

    float starting_ball_x;
    float starting_ball_y;

    Timer sim_timer_;

    int ticks_since_fallen_;
    int fallen_ticks_delay_;
    bool has_fallen_;


    std::ofstream sim_position_log; // Stream to write ground truth x,y,theta, received from simulator
                                    // true ball position
                                    // sensed robot position
                                    // and sensed ball position

    
    std::ofstream multiple_simulation_log; // Stream to log results from multiple simulations



};

#endif   // SIMULATION_SIMULATIONTHREAD_H_


