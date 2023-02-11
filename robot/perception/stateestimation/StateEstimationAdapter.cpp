#include "StateEstimationAdapter.hpp"
#include "blackboard/Blackboard.hpp"

#include "localiser/Localiser.hpp"
#include "egoballtracker/EgoBallTracker.hpp"
#include "teamballtracker/TeamBallTracker.hpp"
#include "multiballtracker/MultiBallTracker.hpp"
#include "robotfilter/RobotFilter.hpp"
#include "utils/incapacitated.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"

#include <boost/archive/text_oarchive.hpp>
#include <boost/math/constants/constants.hpp>

// Chooses between multiballtracker and egoballtracker (single ball)
// When using Multiball you'll want to comment out `#define EARLY_EXIT 1` in BallDetector.cpp
// #define USE_MULTIBALL 1

StateEstimationAdapter::StateEstimationAdapter(Blackboard *bb)
    : Adapter(bb),
      estimatorInfoInit(NULL),
      estimatorInfoIn(NULL),
      estimatorInfoMiddle(NULL),
      estimatorInfoOut(NULL),
      prevOdometry(NULL),
      prevTimeStampInMicroSeconds(-1),
      saveEstimatorObjectsToFile((bb->config)["stateestimation.save_estimator_objects_to_file"].as<bool>())
{
    estimatorInfoInit = new EstimatorInfoInit(
        readFrom(gameController, player_number),
        readFrom(gameController, our_team).teamNumber,
        (bb->config)["stateestimation.initial_pose_type"].as<std::string>(),
        AbsCoord((bb->config)["stateestimation.specified_initial_x"].as<int>(),
            (bb->config)["stateestimation.specified_initial_y"].as<int>(),
            (bb->config)["stateestimation.specified_initial_theta"].as<int>() * boost::math::float_constants::pi / 180.0f),
        readFrom(behaviour, skill),
        readFrom(gameController, data).competitionType,
        readFrom(gameController, data).state,
        readFrom(gameController, data).gamePhase,
        readFrom(gameController, data).setPlay,
        (bb->config)["stateestimation.handle_referee_mistakes"].as<bool>());

    initEstimators();

    // Save estimatorInfoInit to file
    if (saveEstimatorObjectsToFile)
    {
        std::string dirName  = Logger::getLogDir();
        std::string fileName = (bb->config)["stateestimation.estimator_objects_file_name"].as<std::string>();
        estimatorObjectsOut.open((dirName + "/" + fileName).c_str());
        boost::archive::text_oarchive oa(estimatorObjectsOut);
        oa << estimatorInfoInit;
    }
}

/* Functions used for state-estimation-simulation) */
StateEstimationAdapter::StateEstimationAdapter(EstimatorInfoInit *estimatorInfoInit)
    : Adapter(NULL),
      estimatorInfoInit(estimatorInfoInit),
      estimatorInfoIn(NULL),
      estimatorInfoMiddle(NULL),
      estimatorInfoOut(NULL),
      prevOdometry(NULL),
      prevTimeStampInMicroSeconds(-1),
      saveEstimatorObjectsToFile(false)
{
    initEstimators();
}

StateEstimationAdapter::~StateEstimationAdapter()
{
    delete estimatorInfoInit;
    delete prevOdometry;
    for (unsigned i = 0; i < ESTIMATOR_TOTAL; ++i)
    {
        delete estimators[i];
    }

    if (saveEstimatorObjectsToFile)
    {
        estimatorObjectsOut.close();
    }
}

void StateEstimationAdapter::initEstimators()
{
    addEstimator(LOCALISER, new Localiser(*estimatorInfoInit));
#ifndef USE_MULTIBALL
    addEstimator(EGOBALLTRACKER, new EgoBallTracker(*estimatorInfoInit));
#else
    addEstimator(MULTIBALLTRACKER, new MultiBallTracker(*estimatorInfoInit));
#endif
    addEstimator(TEAMBALLTRACKER, new TeamBallTracker(*estimatorInfoInit));
    addEstimator(ROBOTFILTER, new RobotFilter(*estimatorInfoInit));
}

void StateEstimationAdapter::addEstimator(
    unsigned index,
    Estimator *estimator)
{
    estimators[index] = estimator;
}

void StateEstimationAdapter::runEstimators()
{
    runEstimator(LOCALISER);
#ifndef USE_MULTIBALL
    runEstimator(EGOBALLTRACKER);
#else
    runEstimator(MULTIBALLTRACKER);
#endif
    runEstimator(TEAMBALLTRACKER);
    runEstimator(ROBOTFILTER);
}

void StateEstimationAdapter::runEstimator(unsigned index)
{
    getEstimator(index)->tick(
        *estimatorInfoIn,
        *estimatorInfoMiddle,
        *estimatorInfoOut);
}

Estimator *StateEstimationAdapter::getEstimator(unsigned index)
{
    return estimators[index];
}

void StateEstimationAdapter::tick()
{
    createEstimatorInfoIn();
    estimatorInfoMiddle = new EstimatorInfoMiddle();
    estimatorInfoOut = new EstimatorInfoOut();

    runEstimators();

    writeToBlackboard();

    // Save estimatorInfoIn to file
    if (saveEstimatorObjectsToFile)
    {
        boost::archive::text_oarchive oa(estimatorObjectsOut);
        oa << estimatorInfoIn;
        oa << readFrom(vision, timestamp);
    }

    // Delete objects
    delete estimatorInfoIn;
    delete estimatorInfoMiddle;
    delete estimatorInfoOut;
}

void StateEstimationAdapter::tick(
    EstimatorInfoIn *estimatorInfoIn,
    EstimatorInfoMiddle *estimatorInfoMiddle,
    EstimatorInfoOut *estimatorInfoOut)
{
    this->estimatorInfoIn = estimatorInfoIn;
    this->estimatorInfoMiddle = estimatorInfoMiddle;
    this->estimatorInfoOut = estimatorInfoOut;

    runEstimators();
}

void StateEstimationAdapter::createEstimatorInfoIn()
{
    // Update odometryDiff and dtInSeconds
    Odometry odometryDiff = calculateOdometryDiff(readFrom(motion, odometry));
    float dtInSeconds = calculateDtInSeconds(readFrom(vision, timestamp));

    // Read Incoming Broadcast Data
    const std::vector<bool> &havePendingIncomingSharedBundle =
        readFrom(stateEstimation, havePendingIncomingSharedBundle);
    std::vector<BroadcastData> incomingBroadcastData;
    for (unsigned i = 0; i < havePendingIncomingSharedBundle.size(); ++i)
    {
        if (havePendingIncomingSharedBundle[i])
        {
            incomingBroadcastData.push_back(readFrom(receiver, data)[i]);
        }
    }

    estimatorInfoIn = new EstimatorInfoIn(
        readFrom(vision, fieldFeatures),
        readFrom(vision, balls),
        readFrom(gameController, data).competitionType,
        readFrom(gameController, data).state,
        readFrom(gameController, data).gamePhase,
        readFrom(gameController, data).setPlay,
        readFrom(gameController, data).kickingTeam,
        readFrom(behaviour, behaviourSharedData),
        readFrom(gameController, our_team).players[estimatorInfoInit->playerNumber - 1].penalty,
        readFrom(motion, active),
        readFrom(receiver, incapacitated),
        readFrom(stateEstimation, havePendingOutgoingSharedBundle),
        havePendingIncomingSharedBundle,
        incomingBroadcastData,
        readFrom(vision, robots),
        readFrom(motion, sensors).joints.angles[Joints::HeadYaw],
        isIncapacitated(readFrom(motion, active).body.actionType),
        odometryDiff,
        dtInSeconds,
        readFrom(motion, active).body,
        readFrom(motion, sensors));
}

void StateEstimationAdapter::writeToBlackboard()
{
    acquireLock(serialization);
    writeTo(stateEstimation, robotPos, estimatorInfoOut->robotPos);
    writeTo(stateEstimation, robotPosUncertainty, estimatorInfoOut->robotPosUncertainty);
    writeTo(stateEstimation, robotHeadingUncertainty, estimatorInfoOut->robotHeadingUncertainty);
    writeTo(stateEstimation, allRobotPos, estimatorInfoOut->allRobotPos);
    writeTo(stateEstimation, ballPosRR, estimatorInfoOut->ballPosRR);
    writeTo(stateEstimation, ballPosRRC, estimatorInfoOut->ballPosRRC);
    writeTo(stateEstimation, ballVelRRC, estimatorInfoOut->ballVelRRC);
    writeTo(stateEstimation, ballPos, estimatorInfoOut->ballPos);
    writeTo(stateEstimation, ballVel, estimatorInfoOut->ballVel);
    writeTo(stateEstimation, teamBallPos, estimatorInfoOut->teamBallPos);
    writeTo(stateEstimation, teamBallVel, estimatorInfoOut->teamBallVel);
    writeTo(stateEstimation, teamBallPosUncertainty, estimatorInfoOut->teamBallPosUncertainty);
    writeTo(stateEstimation, sharedStateEstimationBundle, estimatorInfoOut->sharedStateEstimationBundle);
    writeTo(stateEstimation, havePendingOutgoingSharedBundle, true);
    writeTo(stateEstimation, havePendingIncomingSharedBundle, std::vector<bool>(5, false));
    writeTo(stateEstimation, robotObstacles, estimatorInfoOut->robotObstacles);
    writeTo(stateEstimation, hadTeamBallUpdate, estimatorInfoOut->hadTeamBallUpdate);
    releaseLock(serialization);
}

Odometry StateEstimationAdapter::calculateOdometryDiff(Odometry newOdometry)
{
    Odometry odometryDiff;

    // Check if not first time
    if (prevOdometry)
    {
        odometryDiff = newOdometry - *prevOdometry;
        delete prevOdometry;
    }

    // Copy odometry
    prevOdometry = new Odometry(newOdometry);

    return odometryDiff;
}

float StateEstimationAdapter::calculateDtInSeconds(int64_t newTimeStampInMicroSeconds)
{
    float dtInSeconds = 0;

    // Check if not first time
    if (prevTimeStampInMicroSeconds > 0)
    {
        // timestamp is in micro-seconds, hence the 1e6.
        dtInSeconds = (newTimeStampInMicroSeconds - prevTimeStampInMicroSeconds) / 1000000.f;
    }

    // Copy timestamp
    prevTimeStampInMicroSeconds = newTimeStampInMicroSeconds;

    return dtInSeconds;
}
