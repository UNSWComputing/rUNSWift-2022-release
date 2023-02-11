#include "SimulationThread.hpp"
#include "types/AbsCoord.hpp"
#include "types/SensorValues.hpp"
#include "types/JointValues.hpp"
#include "utils/TransitionPoses.hpp"
#include "blackboard/Blackboard.hpp"

#include <sys/mman.h>  /* For shared memory */
#include <fcntl.h>     /* For O_* constants */
#include <arpa/inet.h> /* To convert size to network order */
#include <iostream>
#include <cmath>
#include <sstream>

// Boost
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/thread/thread.hpp>

using namespace std;

SimulationThread::SimulationThread(Blackboard *bb)
    : Adapter(bb)
    , connection_(NULL)
    , vision_(bb)
    , current_error_()
    , previous_error_()
    , cumulative_error_()
    , perception_joint_angles_()
    , angle_()
    , team_number_((bb->config)["player.team"].as<int>())
    , team_name_()
    , player_number_((bb->config)["player.number"].as<int>())
    , ticks_since_fallen_(0)
    , has_fallen_(false)
{
    // open shared memory as RW
    string mem_path(AGENT_MEMORY);

    // Append the team / player number to the shared memory path so we don't
    // have multiple instances of sim build using the same shared memory (this
    // means multiple instances MUST use different player numbers).
    int mod = (team_number_ * MAX_NUM_PLAYERS) + player_number_;
    stringstream ss;
    ss << mod;
    mem_path += ss.str();

    shared_fd_ = shm_open(mem_path.c_str(), O_CREAT | O_RDWR, 0600);
    if (shared_fd_ < 0)
    {
        throw runtime_error("AgentEffector: shm_open() failed");
    }

    // map shared memory to process memory
    shared_data_ = (AgentData *)mmap(NULL, sizeof(AgentData),
                                     PROT_READ | PROT_WRITE,
                                     MAP_SHARED, shared_fd_, 0);

    if (shared_data_ == MAP_FAILED)
    {
        throw runtime_error("AgentEffector: mmap() failed");
    }

    if (ftruncate(shared_fd_, sizeof(AgentData)) == -1)
    {
        throw runtime_error("ftruncate() failed");
    }

    // create semaphore with permissions 600, value 0
    string sem_path(AGENT_SEMAPHORE);
    sem_path += ss.str();

    semaphore_ = sem_open(sem_path.c_str(), O_RDWR | O_CREAT, 0600, 0);
    if (semaphore_ == SEM_FAILED)
    {
        throw runtime_error("sem_open() failed");
    }

    shared_data_->init();

    SensorValues null_sensors;
    JointValues null_joints;
    ActionCommand::LED null_leds;

    int i;
    for (i = 0; i < ::Joints::NUMBER_OF_JOINTS; ++i)
    {
        null_joints.angles[i] = 0.0f;
        null_joints.stiffnesses[i] = 0.0f;
        null_joints.temperatures[i] = 0.0f;
    }
    null_sensors.joints = null_joints;
    for (i = 0; i < ::Sensors::NUMBER_OF_SENSORS; ++i)
        null_sensors.sensors[i] = 0.0f;

    for (i = 0; i < 3; ++i)
    {
        shared_data_->sensors[i] = null_sensors;
        shared_data_->joints[i] = null_joints;
        shared_data_->leds[i] = null_leds;
    }
    shared_data_->standing = true;

    srand(time(NULL));

    int initial_x, initial_y;
    float initial_theta;
    if ((bb->config)["stateestimation.initial_pose_type"].as<string>() == "SPECIFIED" || (bb->config)["stateestimation.initial_pose_type"].as<string>() == "ONEVSONE"){
        initial_x = (bb->config)["stateestimation.specified_initial_x"].as<int>();
        initial_y = (bb->config)["stateestimation.specified_initial_y"].as<int>();
        initial_theta = (bb->config)["stateestimation.specified_initial_theta"].as<int>() * boost::math::float_constants::pi / 180.f;
    } else {
        switch (player_number_)
        {
        case 1:
            initial_x = INITIAL_POSE_P1_X;
            initial_y = INITIAL_POSE_P1_Y;
            initial_theta = INITIAL_POSE_P1_THETA;
            break;

        case 2:
            initial_x = INITIAL_POSE_P2_X;
            initial_y = INITIAL_POSE_P2_Y;
            initial_theta = INITIAL_POSE_P2_THETA;
            break;

        case 3:
            initial_x = INITIAL_POSE_P3_X;
            initial_y = INITIAL_POSE_P3_Y;
            initial_theta = INITIAL_POSE_P3_THETA;
            break;

        case 4:
            initial_x = INITIAL_POSE_P4_X;
            initial_y = INITIAL_POSE_P4_Y;
            initial_theta = INITIAL_POSE_P4_THETA;
            break;

        case 5:
            initial_x = INITIAL_POSE_P5_X;
            initial_y = INITIAL_POSE_P5_Y;
            initial_theta = INITIAL_POSE_P5_THETA;
            break;

        case 6:
            initial_x = INITIAL_POSE_P6_X;
            initial_y = INITIAL_POSE_P6_Y;
            initial_theta = INITIAL_POSE_P6_THETA;
            break;

        default:
            initial_x = INITIAL_POSE_DEFAULT_X;
            initial_y = INITIAL_POSE_DEFAULT_Y;
            initial_theta = INITIAL_POSE_DEFAULT_THETA;
            break;
        }
    }
    AbsCoord initialPos = AbsCoord(initial_x, initial_y, initial_theta);

    // int initial_ball_x = 0;
    // int initial_ball_y = 0;
    // if ((bb->config)["stateestimation.initial_ball_x"].as<int>() != 0) {
    //     initial_ball_x = (bb->config)["stateestimation.initial_ball_x"].as<int>();
    // }
    // if ((bb->config)["stateestimation.initial_ball_x"].as<int>() != 0) {
    //     initial_ball_y = (bb->config)["stateestimation.initial_ball_y"].as<int>();
    // }

    // AbsCoord initialBallPos = AbsCoord(initial_ball_x, initial_ball_y, 0);
    // starting_ball_x = initialBallPos.x() / 1000;
    // starting_ball_y = initialBallPos.y() / 1000;

    stringstream ss_teamname;
    ss_teamname << (bb->config)["player.team"].as<int>();
    team_name_ = ss_teamname.str();

    starting_x_ = initialPos.x() / 1000;
    starting_y_ = initialPos.y() / 1000;
    starting_theta_ = RAD2DEG(initialPos.theta());

    connection_ = new SimulationConnection("localhost", 3100, team_name_, player_number_, starting_x_, starting_y_, starting_theta_);

    stringstream ss_player_num;
    ss_player_num << player_number_;
    string player_num = ss_player_num.str();

    string log_suffix = team_name_ + "-" + player_num + ".tsv";

    // Save GroundTruthData to file
    if ((bb->config)["simulation.log_sensed_and_true_positions"].as<bool>())
    {   
        string filename = "sim-position-log-" + log_suffix;
        sim_position_log.open(filename.c_str());
    }

    if ((bb->config)["simulation.multiple_sims"].as<bool>()) {
        string filename = "mul-sim-" + log_suffix;
        multiple_simulation_log.open(filename.c_str());
    }


    if (player_number_ == 1) fallen_ticks_delay_ = 500; // Let the goalie lie on the floor for ~5 seconds (possibly after a dive)
    else fallen_ticks_delay_ = 30; // Field players stand up quickly otherwise they continuously fall over
}

SimulationThread::~SimulationThread()
{
    if ((blackboard->config)["simulation.log_sensed_and_true_positions"].as<bool>()) {
        sim_position_log.close();
    }

    if ((blackboard->config)["simulation.multiple_sims"].as<bool>()) {
        multiple_simulation_log.close();
    }

}

void SimulationThread::tick()
{
    sim_timer_.restart();
    // TODO revisit gyrX/Y and angleX/Y and 'GetupGenerator'

    // Receive perceptor info from server
    PerceptorInfo recv;
    // NOTE: Angles are converted from degrees to radians here
    if (!connection_->receivePerceptorInfo(&recv))
    {
        cerr << "Error receiving perceptor info!\n";
    }

    // Tick simulator vision
    // Only update vision if we have received vision information, since
    // we only receive vision information once every three frames!
    if (recv.observations.size() > 0 || recv.fieldFeatures.size() > 0)
        vision_.tick(recv);

    // Here we need to write the info we received from the simulator to
    // the shared memory. This is effectively replacing the workflow of
    // libagent.
    SensorValues s;
    recv.toSensorValues(&s);

    // Find our AngleX and AngleY values (sim doesn't provide AngleX/Y sensors)
    angle_.addMeasurement(
        s.sensors[::Sensors::InertialSensor_GyroscopeX],
        s.sensors[::Sensors::InertialSensor_GyroscopeY],
        s.sensors[::Sensors::InertialSensor_GyroscopeZ],
        s.sensors[::Sensors::InertialSensor_AccelerometerX],
        s.sensors[::Sensors::InertialSensor_AccelerometerY],
        s.sensors[::Sensors::InertialSensor_AccelerometerZ]);
    angle_.getAngles(
        &s.sensors[::Sensors::InertialSensor_AngleX],
        &s.sensors[::Sensors::InertialSensor_AngleY]);

    // Add sonar measurements (for closest object only)
    // (ijnek) Commented out as sonar sensor format is outdated as of the V6 (FUTURE WORK TO REIMPLMENT THIS)

    // Find the right index in shared memory to write sim data to so rUNSWift
    // can access it
    uint8_t i;
    for (i = 0; i != shared_data_->sensors_latest &&
                i != shared_data_->sensors_read;
         ++i)
    {
    }
    shared_data_->sensors[i] = s;
    shared_data_->sensors_latest = i;

    sem_post(semaphore_);

    // Read values from motion
    // Angles are in radians here
    shared_data_->actuators_read = shared_data_->actuators_latest;

    // This is our motion request
    JointValues motion_request = shared_data_->joints[shared_data_->actuators_read];

    // Translate to Simulator joints
    Simulation::Joints motion_joint_request;
    motion_joint_request.fromJointValues(motion_request);

    // Copy joint request for hipyawpitch
    motion_joint_request.rlj1 = motion_joint_request.llj1;

    // actual thing that is going to bein command
    Simulation::Joints to_simulator_joint_request;

    // Use a PID Controller to controller joints (taken from UT-Austin 3d sim league)
    float P = 0.50;
    float I = 0.0;
    float D = 0.01;
    previous_error_.hj1 = current_error_.hj1;
    current_error_.hj1 = motion_joint_request.hj1 - recv.joints.hj1;
    cumulative_error_.hj1 += current_error_.hj1;
    to_simulator_joint_request.hj1 = P * current_error_.hj1 + I * cumulative_error_.hj1 + D * (current_error_.hj1 - previous_error_.hj1);
    previous_error_.hj2 = current_error_.hj2;
    current_error_.hj2 = motion_joint_request.hj2 - recv.joints.hj2;
    cumulative_error_.hj2 += current_error_.hj2;
    to_simulator_joint_request.hj2 = P * current_error_.hj2 + I * cumulative_error_.hj2 + D * (current_error_.hj2 - previous_error_.hj2);
    previous_error_.raj1 = current_error_.raj1;
    current_error_.raj1 = motion_joint_request.raj1 - recv.joints.raj1;
    cumulative_error_.raj1 += current_error_.raj1;
    to_simulator_joint_request.raj1 = P * current_error_.raj1 + I * cumulative_error_.raj1 + D * (current_error_.raj1 - previous_error_.raj1);
    previous_error_.raj2 = current_error_.raj2;
    current_error_.raj2 = motion_joint_request.raj2 - recv.joints.raj2;
    cumulative_error_.raj2 += current_error_.raj2;
    to_simulator_joint_request.raj2 = P * current_error_.raj2 + I * cumulative_error_.raj2 + D * (current_error_.raj2 - previous_error_.raj2);
    previous_error_.raj3 = current_error_.raj3;
    current_error_.raj3 = motion_joint_request.raj3 - recv.joints.raj3;
    cumulative_error_.raj3 += current_error_.raj3;
    to_simulator_joint_request.raj3 = P * current_error_.raj3 + I * cumulative_error_.raj3 + D * (current_error_.raj3 - previous_error_.raj3);
    previous_error_.raj4 = current_error_.raj4;
    current_error_.raj4 = motion_joint_request.raj4 - recv.joints.raj4;
    cumulative_error_.raj4 += current_error_.raj4;
    to_simulator_joint_request.raj4 = P * current_error_.raj4 + I * cumulative_error_.raj4 + D * (current_error_.raj4 - previous_error_.raj4);
    previous_error_.laj1 = current_error_.laj1;
    current_error_.laj1 = motion_joint_request.laj1 - recv.joints.laj1;
    cumulative_error_.laj1 += current_error_.laj1;
    to_simulator_joint_request.laj1 = P * current_error_.laj1 + I * cumulative_error_.laj1 + D * (current_error_.laj1 - previous_error_.laj1);
    previous_error_.laj2 = current_error_.laj2;
    current_error_.laj2 = motion_joint_request.laj2 - recv.joints.laj2;
    cumulative_error_.laj2 += current_error_.laj2;
    to_simulator_joint_request.laj2 = P * current_error_.laj2 + I * cumulative_error_.laj2 + D * (current_error_.laj2 - previous_error_.laj2);
    previous_error_.laj3 = current_error_.laj3;
    current_error_.laj3 = motion_joint_request.laj3 - recv.joints.laj3;
    cumulative_error_.laj3 += current_error_.laj3;
    to_simulator_joint_request.laj3 = P * current_error_.laj3 + I * cumulative_error_.laj3 + D * (current_error_.laj3 - previous_error_.laj3);
    previous_error_.laj4 = current_error_.laj4;
    current_error_.laj4 = motion_joint_request.laj4 - recv.joints.laj4;
    cumulative_error_.laj4 += current_error_.laj4;
    to_simulator_joint_request.laj4 = P * current_error_.laj4 + I * cumulative_error_.laj4 + D * (current_error_.laj4 - previous_error_.laj4);
    previous_error_.rlj1 = current_error_.rlj1;
    current_error_.rlj1 = motion_joint_request.rlj1 - recv.joints.rlj1;
    cumulative_error_.rlj1 += current_error_.rlj1;
    to_simulator_joint_request.rlj1 = P * current_error_.rlj1 + I * cumulative_error_.rlj1 + D * (current_error_.rlj1 - previous_error_.rlj1);
    previous_error_.rlj2 = current_error_.rlj2;
    current_error_.rlj2 = motion_joint_request.rlj2 - recv.joints.rlj2;
    cumulative_error_.rlj2 += current_error_.rlj2;
    to_simulator_joint_request.rlj2 = P * current_error_.rlj2 + I * cumulative_error_.rlj2 + D * (current_error_.rlj2 - previous_error_.rlj2);
    previous_error_.rlj3 = current_error_.rlj3;
    current_error_.rlj3 = motion_joint_request.rlj3 - recv.joints.rlj3;
    cumulative_error_.rlj3 += current_error_.rlj3;
    to_simulator_joint_request.rlj3 = P * current_error_.rlj3 + I * cumulative_error_.rlj3 + D * (current_error_.rlj3 - previous_error_.rlj3);
    previous_error_.rlj4 = current_error_.rlj4;
    current_error_.rlj4 = motion_joint_request.rlj4 - recv.joints.rlj4;
    cumulative_error_.rlj4 += current_error_.rlj4;
    to_simulator_joint_request.rlj4 = P * current_error_.rlj4 + I * cumulative_error_.rlj4 + D * (current_error_.rlj4 - previous_error_.rlj4);
    previous_error_.rlj5 = current_error_.rlj5;
    current_error_.rlj5 = motion_joint_request.rlj5 - recv.joints.rlj5;
    cumulative_error_.rlj5 += current_error_.rlj5;
    to_simulator_joint_request.rlj5 = P * current_error_.rlj5 + I * cumulative_error_.rlj5 + D * (current_error_.rlj5 - previous_error_.rlj5);
    previous_error_.rlj6 = current_error_.rlj6;
    current_error_.rlj6 = motion_joint_request.rlj6 - recv.joints.rlj6;
    cumulative_error_.rlj6 += current_error_.rlj6;
    to_simulator_joint_request.rlj6 = P * current_error_.rlj6 + I * cumulative_error_.rlj6 + D * (current_error_.rlj6 - previous_error_.rlj6);
    previous_error_.llj1 = current_error_.llj1;
    current_error_.llj1 = motion_joint_request.llj1 - recv.joints.llj1;
    cumulative_error_.llj1 += current_error_.llj1;
    to_simulator_joint_request.llj1 = P * current_error_.llj1 + I * cumulative_error_.llj1 + D * (current_error_.llj1 - previous_error_.llj1);
    previous_error_.llj2 = current_error_.llj2;
    current_error_.llj2 = motion_joint_request.llj2 - recv.joints.llj2;
    cumulative_error_.llj2 += current_error_.llj2;
    to_simulator_joint_request.llj2 = P * current_error_.llj2 + I * cumulative_error_.llj2 + D * (current_error_.llj2 - previous_error_.llj2);
    previous_error_.llj3 = current_error_.llj3;
    current_error_.llj3 = motion_joint_request.llj3 - recv.joints.llj3;
    cumulative_error_.llj3 += current_error_.llj3;
    to_simulator_joint_request.llj3 = P * current_error_.llj3 + I * cumulative_error_.llj3 + D * (current_error_.llj3 - previous_error_.llj3);
    previous_error_.llj4 = current_error_.llj4;
    current_error_.llj4 = motion_joint_request.llj4 - recv.joints.llj4;
    cumulative_error_.llj4 += current_error_.llj4;
    to_simulator_joint_request.llj4 = P * current_error_.llj4 + I * cumulative_error_.llj4 + D * (current_error_.llj4 - previous_error_.llj4);
    previous_error_.llj5 = current_error_.llj5;
    current_error_.llj5 = motion_joint_request.llj5 - recv.joints.llj5;
    cumulative_error_.llj5 += current_error_.llj5;
    to_simulator_joint_request.llj5 = P * current_error_.llj5 + I * cumulative_error_.llj5 + D * (current_error_.llj5 - previous_error_.llj5);
    previous_error_.llj6 = current_error_.llj6;
    current_error_.llj6 = motion_joint_request.llj6 - recv.joints.llj6;
    cumulative_error_.llj6 += current_error_.llj6;
    to_simulator_joint_request.llj6 = P * current_error_.llj6 + I * cumulative_error_.llj6 + D * (current_error_.llj6 - previous_error_.llj6);

    EffectorCommand cmd;
    cmd.joints = to_simulator_joint_request;
    connection_->sendEffectorCommand(cmd);

    if (shouldRestart(s)){
        cumulative_error_ = Simulation::Joints(); // reset Integral component of PID controller
        if (connection_){
            delete connection_;
            double x = recv.myPos[0]/1000.0;
            double y = recv.myPos[1]/1000.0;
            // double ball_x = recv.ballPos[0]/1000.0;
            // double ball_y = recv.ballPos[1]/1000.0;
            double theta = RAD2DEG(recv.myOrien);
            connection_ = new SimulationConnection("localhost", 3100, team_name_, player_number_, x, y, theta);
        }
    }

    // Write to file, true x,y,theta of robot
    if ((blackboard->config)["simulation.log_sensed_and_true_positions"].as<bool>()
        && recv.ballPos
        && recv.parsedMyPosAndOrien)
    {
        sim_position_log << recv.time << "\t";

        sim_position_log << recv.myPos[0] << "\t" << recv.myPos[1] << "\t" << recv.myOrien << "\t";

        int s_myPos_x = (blackboard->stateEstimation.robotPos).x();
        int s_myPos_y = (blackboard->stateEstimation.robotPos).y(); 
        double s_myPos_th = (blackboard->stateEstimation.robotPos).theta();
        sim_position_log << s_myPos_x << "\t" << s_myPos_y << "\t" << s_myPos_th << "\t";


        sim_position_log << recv.ballPos[0] << "\t" << recv.ballPos[1] << "\t";

        int s_ball_x = (blackboard->stateEstimation.teamBallPos).x();
        int s_ball_y = (blackboard->stateEstimation.teamBallPos).y(); 
        sim_position_log << s_ball_x << "\t" << s_ball_y << endl;
    }
}

bool SimulationThread::shouldRestart(SensorValues& s)
{
    if ((blackboard->config)["simulation.restart_when_fallen"].as<bool>())
    {
        
        if (has_fallen_ || abs(RAD2DEG(s.sensors[::Sensors::InertialSensor_AngleX])) > 70 || abs(RAD2DEG(s.sensors[::Sensors::InertialSensor_AngleY])) > 70) {
            
            
            has_fallen_ = true;
            if (ticks_since_fallen_++ > fallen_ticks_delay_) {
                ticks_since_fallen_ = 0;
                has_fallen_ = false;
                return true;
            }
        }
    }
     
    return false;
}
