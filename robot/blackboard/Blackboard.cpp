#include "blackboard/Blackboard.hpp"
#include "thread/Thread.hpp"
#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <limits>
#include "utils/angles.hpp"
#include "utils/Logger.hpp"
#include "utils/options.hpp"
#include "utils/ConfigUtils.hpp"
#include "perception/stateestimation/StateEstimationDefinitions.hpp"

using namespace std;
using namespace boost;

Blackboard::Blackboard(){
      boost::program_options::variables_map vm;
      std::vector<std::string> argv;
      store_and_notify(argv, vm, NULL);
      config = boost::program_options::variables_map(vm);

      mask = INITIAL_MASK;
      readOptions(vm);
      thread.configCallbacks["Blackboard"] = bind(&Blackboard::readOptions, this, _1);
}

Blackboard::Blackboard(const program_options::variables_map &vm)
   : config(vm), mask(INITIAL_MASK) {
      readOptions(vm);
      thread.configCallbacks["Blackboard"] = bind(&Blackboard::readOptions, this, _1);
      thread.configCallbacks["Logger"] = &Logger::readOptions;
   }

Blackboard::~Blackboard() {
   thread.configCallbacks["Blackboard"] =
      boost::function<void(const program_options::variables_map &)>();
}

void Blackboard::readOptions(const program_options::variables_map& config) {
   behaviour.readOptions(config);
   gameController.readOptions(config);
   receiver.readOptions(config);
   kinematics.readOptions(config);
   stateEstimation.readOptions(config);
   vision.readOptions(config);
}

BehaviourBlackboard::BehaviourBlackboard() {
   readBuf = 0;
}

void BehaviourBlackboard::readOptions(const program_options::variables_map& config) {
    skill = config["behaviour.skill"].as<string>();
    headskill = config["behaviour.headskill"].as<string>();
    positioning = config["behaviour.positioning"].as<string>();
    remoteStiffen = config["behaviour.remote_stiffen"].as<bool>();
    useGetups = config["behaviour.use_getups"].as<bool>();
    cleverNaoInfo.cleverNaoBehaviour = config["behaviour.clever_nao"].as<bool>();
    cleverNaoInfo.cleverNaoRecording = config["behaviour.clever_nao.record"].as<bool>();
    if(config["kick.foot"].as<string>() == "LEFT")
        cleverNaoInfo.kickLeft = true;
    else
        cleverNaoInfo.kickLeft = false;
}

void StateEstimationBlackboard::readOptions(const program_options::variables_map& config) {
}

StateEstimationBlackboard::StateEstimationBlackboard() {
   robotObstacles.reserve(MAX_ROBOT_OBSTACLES);
   robotPos = AbsCoord();
   robotPosUncertainty = 0.0f;
   robotHeadingUncertainty = 0.0f;
   allRobotPos.reserve(MAX_POSE_HYPOTHESES);
   ballPosRR = RRCoord();
   ballPosRRC = AbsCoord();
   ballVelRRC = AbsCoord();
   ballVel = AbsCoord();
   ballPos = AbsCoord();
   teamBallPos = AbsCoord();
   teamBallVel = AbsCoord();
   teamBallPosUncertainty = 0.0f;
   sharedStateEstimationBundle = SharedStateEstimationBundle();

   // This flag is here so that state estimation can know when a shared state estimation bundle has
   // been sent so it can reset it.
   havePendingOutgoingSharedBundle = false;

   // one flag per teammate, include ourselves for simplicity.
   havePendingIncomingSharedBundle = std::vector<bool>(5, false);

   hadTeamBallUpdate = false;
}

VisionBlackboard::VisionBlackboard()
{
   balls.reserve(MAX_BALLS);
   robots.reserve(MAX_ROBOTS);
   fieldBoundaries.reserve(MAX_FIELD_BOUNDARIES);
   fieldFeatures.reserve(MAX_FIELD_FEATURES);
   regions.reserve(ESTIMATED_REGIONS);

   topSaliency = NULL;
   botSaliency = NULL;
   topFrame = NULL;
   botFrame = NULL;

   horizontalFieldOfView = IMAGE_HFOV;
   verticalFieldOfView = IMAGE_VFOV;
}

void VisionBlackboard::readOptions(const program_options::variables_map& config) {
   topFrameJPEGQuality = config["debug.top.jpeg"].as<int>();
   botFrameJPEGQuality = config["debug.bot.jpeg"].as<int>();
}

PerceptionBlackboard::PerceptionBlackboard() {
   kinematics = 0;
   stateEstimation = 0;
   vision = 0;
   behaviour = 0;
   total = 33;
}

MotionBlackboard::MotionBlackboard() {
   buttons = ButtonPresses();
   uptime = 0;
   isStiff = false;
}

RemoteControlBlackboard::RemoteControlBlackboard() {
   time_received = 0;
}

KinematicsBlackboard::KinematicsBlackboard() {
}

void KinematicsBlackboard::readOptions(const program_options::variables_map& config) {
    parameters.cameraYawBottom =
         config["kinematics.cameraYawBottom"].as<float>();
    parameters.cameraPitchBottom =
         config["kinematics.cameraPitchBottom"].as<float>();
    parameters.cameraRollBottom =
         config["kinematics.cameraRollBottom"].as<float>();
    parameters.cameraRollTop =
         config["kinematics.cameraRollTop"].as<float>();
    parameters.cameraYawTop = config["kinematics.cameraYawTop"].as<float>();
    parameters.cameraPitchTop =
         config["kinematics.cameraPitchTop"].as<float>();
    parameters.bodyPitch = config["kinematics.bodyPitch"].as<float>();
    bodyName = config["player.bodyName"].as<std::string>();
    if(bodyName.length() == 0)
        bodyName = getNaoHeadName();
}

GameControllerBlackboard::GameControllerBlackboard() {
   connected = false;
   memset(&our_team, 0, sizeof our_team);
   memset(&data, 0, sizeof data);
   lastGameControllerIPAddress = NULL;
   gameState = 0;
   whistleDetected = false;
}

static const string TEAM_COLOURS[] = {
   // same order as #define TEAM_colour in robot/gamecontroller/RoboCupGameControlData.hpp
   "blue",
   "red",
   "yellow",
   "black",
   "white",
   "green",
   "orange",
   "purple",
   "brown",
   "gray",
};
static const int NUM_TEAM_COLOURS = sizeof(TEAM_COLOURS) / sizeof(string);

void GameControllerBlackboard::readOptions(const program_options::variables_map& config) {
   connect = config["gamecontroller.connect"].as<bool>();
   player_number = config["player.number"].as<int>();
   our_team.teamNumber = static_cast<uint8_t>(config["player.team"].as<int>());
   our_team.teamColour = static_cast<uint8_t>(find(TEAM_COLOURS,
                                 TEAM_COLOURS + NUM_TEAM_COLOURS,
                                                   config["gamecontroller.ourcolour"].as<string>()) -
                                              TEAM_COLOURS);
   if (our_team.teamColour >= NUM_TEAM_COLOURS)
      llog(ERROR) << "Invalid gamecontroller.ourcolour\n";
   our_team.score = static_cast<uint8_t>(config["gamecontroller.ourscore"].as<int>());
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      our_team.players[i].penalty = PENALTY_NONE;
      our_team.players[i].secsTillUnpenalised = 0;
   }
   TeamInfo their_team = {};
   their_team.teamNumber = static_cast<uint8_t>(config["gamecontroller.opponentteam"].as<int>());
   their_team.teamColour = static_cast<uint8_t>(find(TEAM_COLOURS,
                                   TEAM_COLOURS + NUM_TEAM_COLOURS,
                                                     config["gamecontroller.opponentcolour"].as<string>()) -
                                                TEAM_COLOURS);
   if (their_team.teamColour >= NUM_TEAM_COLOURS)
      llog(ERROR) << "Invalid gamecontroller.opponentteam\n";
   their_team.score = static_cast<uint8_t>(config["gamecontroller.opponentscore"].as<int>());
   for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
      their_team.players[i].penalty = PENALTY_NONE;
      their_team.players[i].secsTillUnpenalised = 0;
   }

   // Config allows testing states without running the Java GameController
   map<string, int> gcStateMap;
   gcStateMap["INITIAL"] = STATE_INITIAL;
   gcStateMap["READY"] = STATE_READY;
   gcStateMap["SET"] = STATE_SET;
   gcStateMap["PLAYING"] = STATE_PLAYING;
   gcStateMap["FINISHED"] = STATE_FINISHED;
   data.state = gcStateMap[config["gamecontroller.state"].as<string>()];
   data.firstHalf = config["gamecontroller.firsthalf"].as<bool>();
   data.kickingTeam = static_cast<uint8_t>(config["gamecontroller.kickingteam"].as<int>());
   map<string, int> gcGamePhaseMap;
   gcGamePhaseMap["NORMAL"] = GAME_PHASE_NORMAL;
   gcGamePhaseMap["PENALTYSHOOT"] = GAME_PHASE_PENALTYSHOOT;
   gcGamePhaseMap["OVERTIME"] = GAME_PHASE_OVERTIME;
   gcGamePhaseMap["TIMEOUT"] = GAME_PHASE_TIMEOUT;
   data.gamePhase = gcGamePhaseMap[
      config["gamecontroller.gamephase"].as<string>()];
   data.secsRemaining = config["gamecontroller.secsremaining"].as<int>();
   // I hate introducing hard-to-reproduce elements, but we should be testing both scenarios
   srand(time(NULL));
   if (rand() % 2) {
      data.teams[0] = our_team;
      data.teams[1] = their_team;
   } else {
      data.teams[0] = their_team;
      data.teams[1] = our_team;
   }

   map<string, int> gcSetPlayMap;
   gcSetPlayMap["NONE"] = SET_PLAY_NONE;
   gcSetPlayMap["GOAL_KICK"] = SET_PLAY_GOAL_KICK;
   gcSetPlayMap["PUSHING_FREE_KICK"] = SET_PLAY_PUSHING_FREE_KICK;
   gcSetPlayMap["CORNER_KICK"] = SET_PLAY_CORNER_KICK;
   gcSetPlayMap["KICK_IN"] = SET_PLAY_KICK_IN;
   data.setPlay = gcSetPlayMap[
      config["gamecontroller.setplay"].as<string>()];

}

ReceiverBlackboard::ReceiverBlackboard() {
   BOOST_FOREACH(time_t & lr, lastReceived) {
      lr = 0;
   }

   for (unsigned i = 0; i < ROBOTS_PER_TEAM; ++i)
      incapacitated.push_back(i);
}

void ReceiverBlackboard::readOptions(const program_options::variables_map& config) {
   team = config["player.team"].as<int>();
}

ThreadBlackboard::ThreadBlackboard() {
}

SynchronisationBlackboard::SynchronisationBlackboard() {
   buttons = new boost::mutex();
   serialization = new boost::mutex();
}
