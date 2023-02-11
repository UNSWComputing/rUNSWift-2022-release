#include "MultiBallCMKF.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"
#include "types/ActionCommand.hpp"
#include "MultiBallCMKFParams.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/eigen_helpers.hpp"
#include "perception/stateestimation/StateEstimationDefinitions.hpp"

// #define DEBUG

static const Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_TOTAL>
    ballObservationModel((Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_TOTAL>() << 1, 0, 0, 0, 0, 1, 0, 0).finished());

static const Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_POS>
    ballObservationModelTranspose(ballObservationModel.transpose());

MultiBallCMKF::MultiBallCMKF(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
{
    params = new MultiBallCMKFParams();
}

MultiBallCMKF::~MultiBallCMKF()
{
    if (params)
        delete params;
}

void MultiBallCMKF::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    // Once we have sent out the data, reset haveOutgoingBallUpdate
    if (!estimatorInfoIn.havePendingOutgoingSharedBundle)
        haveOutgoingBallUpdate = false;

    const std::vector<BallInfo> &balls = estimatorInfoIn.balls;
    std::vector<bool> used = std::vector<bool>(kfs.size(), false);

#ifdef DEBUG
    std::cout << "\n\nMultiBallCMKF " << kfs.size() << "kfs and " << balls.size() << "balls" << std::endl;
#endif

    if (estimatorInfoIn.actionCommandBody.actionType == ActionCommand::Body::KICK)
        timeSinceLastKick = 0;
    else
        timeSinceLastKick = std::min(timeSinceLastKick + estimatorInfoIn.dtInSeconds, 2 * 10 * 60.0f); // Avoid an overflow. Longer than a game

    for (unsigned i = 0; i < kfs.size(); ++i)
    {
        BCMKF &kf = kfs[i];

        predictCollision(kf);
        predict(kf, estimatorInfoIn.dtInSeconds);
        predictWithOdometry(kf, estimatorInfoIn.odometryDiff);

        if (estimatorInfoMiddle.canDoObservations)
        {
            float closestDistance = -1;
            unsigned closestIndex = -1;

            // Find the closest unused ball observation to the ith kf
            for (unsigned j = 0; j < balls.size(); ++j)
            {
                const BallInfo &ball = balls[j];
                // Sometimes we get nan in the observations, ignore these
                if (!check_finite(ball.rr.vec, __FILE__ "\tball.rr.vec") ||
                    !check_finite(ball.rr.var, __FILE__ "\tball.rr.var"))
                {
                    std::cerr << "ball:\t" << ball << "\n";
                    continue;
                }

                // Ensure each ball observation are used exactly once
                if (used[j])
                    continue;

                AbsCoord b = AbsCoord(kf.getBallCoordRRC().x() + kf.getBallVelCoordRRC().x() * estimatorInfoIn.dtInSeconds * 4,
                                      kf.getBallCoordRRC().y() + kf.getBallVelCoordRRC().y() * estimatorInfoIn.dtInSeconds * 4, 0);
                float distance = b.convertToRobotRelative().distance(ball.rr);
#ifdef DEBUG
                std::cout << i << "th kf is " << distance << " or " << kf.getBallCoordRRC().convertToRobotRelative().distance() << " away from the " << j << "th ball" << std::endl;
#endif
                if (closestDistance == -1 || closestDistance > distance)
                {
                    closestIndex = j;
                    closestDistance = distance;
                }
            }

            if (closestDistance == -1) continue;

            bool close = closestDistance < params->similarThreshConstant + kf.getBallCoordRRC().distance() * params->similarThreshLinear;

            // Hack to try and join observations to the ball we just kicked
            // TODO: change this so we delete the KF at our feet when we kick and don't add KFs right infront after we kick
            bool kicked = (estimatorInfoIn.actionCommandBody.actionType == ActionCommand::Body::KICK || timeSinceLastKick < 2) // We are/were kicking
                          && kf.getBallCoordRRC().distance() < params->similarThreshConstant * 4                                      // The kf is close to the robots feet
                          && balls[closestIndex].rr.heading() < M_PI_2                                                         // The ball observation is infront of the robot
                          && closestDistance < params->similarThreshConstant * 4;                                                     // The ball observation is kindof close

#ifdef DEBUG
            std::cout << closestDistance << " " << params->similarThreshConstant + kf.getBallCoordRRC().distance() * params->similarThreshLinear << std::endl;
            std::cout << (estimatorInfoIn.actionCommandBody.actionType == ActionCommand::Body::KICK || timeSinceLastKick < 2) // We are kicking
                      << (kf.getBallCoordRRC().distance() < params->similarThreshConstant * 4)                                       // The kf is close to the robots feet
                      << (balls[closestIndex].rr.heading() < M_PI_2)                                                          // The ball observation is infront of the robot
                      << (closestDistance < params->similarThreshConstant * 4)                                                       // The ball observation is kindof close
                      << std::endl;
#endif

            if (close || kicked)
            {
#ifdef DEBUG
                std::cout << "Updating the " << i << "th kf with the " << closestIndex << "th ball" << close << kicked << std::endl;
#endif
                update(kf, balls[closestIndex], estimatorInfoIn);
                used[closestIndex] = true;
                haveOutgoingBallUpdate = true;
            }
        }
    }

    // Add a new KF for each unused observation
    std::vector<BCMKF> newCMKFs;
    for (unsigned i = 0; i < balls.size(); ++i)
    {
        if (!used[i])
        {
            BallStateVector state = BallStateVector::Zero();
            state(BALL_X_DIM, 0) = balls[i].rr.distance() * cosf(balls[i].rr.heading());
            state(BALL_Y_DIM, 0) = balls[i].rr.distance() * sinf(balls[i].rr.heading());

#ifdef DEBUG
            std::cout << "Adding new kf at (" << state(BALL_X_DIM, 0) << ", " << state(BALL_Y_DIM, 0) << ")" << std::endl;
#endif
            BallCovarianceMatrix covariance = BallCovarianceMatrix::Zero();
            covariance(BALL_X_DIM, BALL_X_DIM) = 1000000;
            covariance(BALL_Y_DIM, BALL_Y_DIM) = 1000000;
            covariance(BALL_U_DIM, BALL_U_DIM) = 1000;
            covariance(BALL_V_DIM, BALL_V_DIM) = 1000;

            BCMKF newCMKF(state, covariance, params->ballWeightInitial);
            newCMKFs.push_back(newCMKF);
            haveOutgoingBallUpdate = true;
        }
    }

    kfs.insert(kfs.end(), newCMKFs.begin(), newCMKFs.end());

    if (estimatorInfoMiddle.canDoObservations)
    {
        deleteOffFieldCMKFs(estimatorInfoOut);
        mergeCMKFs();
        decayCMKFWeights();
        deleteLowWeightCMKFs();
    }

    const float minWeight = 1; // TODO: this weight requirement does practically nothing

    float closestDistance = -1;
    int closestIndex = -1;
    for (unsigned j = 0; j < kfs.size(); ++j)
    {
        // Enforce some level of confidence
        if (kfs[j].weight < minWeight)
            continue;
            
        RRCoord ballPosRR = kfs[j].getBallCoordRRC().convertToRobotRelative();
        const float distance = ballPosRR.distance();
        bool closer = closestDistance == -1 || closestDistance > distance;

        AbsCoord robotPos = estimatorInfoOut.robotPos;
        float ballPosWorldX = robotPos.x() + ballPosRR.distance() * cosf(robotPos.theta() + ballPosRR.heading());
        bool onOurHalf = ballPosWorldX < 0;

        if (onOurHalf && closer)
        {
            closestDistance = distance;
            closestIndex = j;
        }
    }
    if (closestIndex == -1)
        return;

    BCMKF &kf = kfs[closestIndex];

    AbsCoord ballPosRRC = kf.getBallCoordRRC();
    estimatorInfoOut.ballPosRRC = ballPosRRC;

    AbsCoord ballVelRRC = kf.getBallVelCoordRRC();
    estimatorInfoOut.ballVelRRC = ballVelRRC;
#ifdef DEBUG
    std::cout << "Weight = " << kf.weight << " Distance = " << closestDistance << std::endl;
    std::cout << ballPosRRC.x() << " " << ballPosRRC.y() << " " << ballVelRRC.x() << " " << ballVel.y() << std::endl;
#endif

    estimatorInfoOut.sharedStateEstimationBundle.ballPosRRC = ballPosRRC;
    estimatorInfoOut.sharedStateEstimationBundle.ballVelRRC = ballVelRRC;
    estimatorInfoOut.sharedStateEstimationBundle.haveBallUpdate = haveOutgoingBallUpdate;

    // (TODO-kenji) I dont think the following should belong here: but I'm adding to make behaviours work for now./
    // It should be outside this file, as EgoBall shouldn't care about the where the robot is
    AbsCoord robotPos = estimatorInfoOut.robotPos;
    RRCoord ballPosRR = ballPosRRC.convertToRobotRelative();
    float ballPosWorldX = robotPos.x() + ballPosRR.distance() * cosf(robotPos.theta() + ballPosRR.heading());
    float ballPosWorldY = robotPos.y() + ballPosRR.distance() * sinf(robotPos.theta() + ballPosRR.heading());
    // clip ball back on to field
    ballPosWorldX = std::min((FIELD_LENGTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f,
                                std::max(-(FIELD_LENGTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f, ballPosWorldX));
    ballPosWorldY = std::min((FIELD_WIDTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f,
                                std::max(-(FIELD_WIDTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f, ballPosWorldY));
    AbsCoord ballPosWorld = AbsCoord(ballPosWorldX, ballPosWorldY, 0);

    Eigen::Matrix2f rotation;
    float heading = robotPos.theta();
    rotation << cosf(heading), -sinf(heading), sinf(heading), cosf(heading);
    Eigen::Matrix2f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse
    check_finite(rotationInv, "ball rotation inverse");

    Eigen::Matrix2f absCovariance = rotation * kf.covariance.block<NUM_DIM_BALL_POS, NUM_DIM_BALL_POS>(BALL_X_DIM, BALL_X_DIM) * rotationInv;
    check_finite(absCovariance, "ball abbsCovariance");
    ballPosWorld.var.block<2, 2>(0, 0) = absCovariance;
    estimatorInfoOut.ballPos = ballPosWorld;

    estimatorInfoOut.ballPosRR = ballPosRR;
    RRCoord ballVelRR = ballVelRRC.convertToRobotRelative();
    estimatorInfoOut.ballVel = AbsCoord(
        ballVelRR.distance() * cosf(robotPos.theta() + ballVelRR.heading()),
        ballVelRR.distance() * sinf(robotPos.theta() + ballVelRR.heading()),
        0);
    // (TODO-kenji) up to here
}

void MultiBallCMKF::predict(BCMKF &kf, const float dtInSeconds)
{
    if (dtInSeconds <= 1)
    {
        /* Followed "Example application, technical" from "https://en.wikipedia.org/wiki/Kalman_filter" */

        // Error in acceleration (TODO: tune these values)
        float stdAccelerationX = params->ballStdAccelerationX;
        float stdAccelerationY = params->ballStdAccelerationY;

        // Take part of the kalman filter
        Eigen::Block<BallStateVector, NUM_DIM_BALL_TOTAL, 1> blockedState = kf.state.segment<NUM_DIM_BALL_TOTAL>(BALL_X_DIM);
        Eigen::Block<BallCovarianceMatrix, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> blockedCovariance =
            kf.covariance.block<NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL>(BALL_X_DIM, BALL_X_DIM);

        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> stateTransitionMatrix;
        stateTransitionMatrix.setIdentity();
        stateTransitionMatrix(0, 2) = dtInSeconds;
        stateTransitionMatrix(1, 3) = dtInSeconds;
        check_finite(stateTransitionMatrix, "ball state transition matrix");

        float dt2InSeconds2 = dtInSeconds * dtInSeconds;
        float halfDt3InSeconds3 = dt2InSeconds2 * dtInSeconds / 2;
        float quarterDt4InSeconds4 = dt2InSeconds2 * dt2InSeconds2 / 4;

        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> processNoiseCovarianceA;
        processNoiseCovarianceA.setZero();
        processNoiseCovarianceA(0, 0) = quarterDt4InSeconds4;
        processNoiseCovarianceA(0, 2) = halfDt3InSeconds3;
        processNoiseCovarianceA(2, 0) = halfDt3InSeconds3;
        processNoiseCovarianceA(2, 2) = dt2InSeconds2;

        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> processNoiseCovarianceB;
        processNoiseCovarianceB.setZero();
        processNoiseCovarianceB(1, 1) = quarterDt4InSeconds4;
        processNoiseCovarianceB(1, 3) = halfDt3InSeconds3;
        processNoiseCovarianceB(3, 1) = halfDt3InSeconds3;
        processNoiseCovarianceB(3, 3) = dt2InSeconds2;

        // Use knowledge about friction for control input (DO we need a covariance for this?)
        float controlVector;
        float u = kf.state(BALL_U_DIM, 0);
        float v = kf.state(BALL_V_DIM, 0);
        float ballSpeed = hypotf(u, v);
        float ballRollDirection = atan2f(v, u);
        controlVector = std::max(BALL_ACCELERATION, (0 - ballSpeed) / dtInSeconds);

        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, 1> controlInputModel;
        controlInputModel(0, 0) = dt2InSeconds2 / 2 * cosf(ballRollDirection);
        controlInputModel(1, 0) = dt2InSeconds2 / 2 * sinf(ballRollDirection);
        controlInputModel(2, 0) = dtInSeconds * cosf(ballRollDirection);
        controlInputModel(3, 0) = dtInSeconds * sinf(ballRollDirection);
        check_finite(controlInputModel, "ball control input model");

        blockedState = stateTransitionMatrix * blockedState + controlInputModel * controlVector;
        blockedCovariance =
            stateTransitionMatrix * blockedCovariance * stateTransitionMatrix.transpose() +
            processNoiseCovarianceA * (stdAccelerationX * stdAccelerationX) +
            processNoiseCovarianceB * (stdAccelerationY * stdAccelerationY);
        check_finite(blockedState, "ball blocked state");
        check_finite(blockedCovariance, "ball blocked covariance");
    }
}

// Predict any possible ball collisions with myself
void MultiBallCMKF::predictCollision(BCMKF &kf)
{
    float x = kf.state(BALL_X_DIM, 0);
    float y = kf.state(BALL_Y_DIM, 0);
    float u = kf.state(BALL_U_DIM, 0);
    float v = kf.state(BALL_V_DIM, 0);
    float ballDistance = hypotf(x, y);
    float ballHeading = atan2f(y, x);
    float ballRollDirection = atan2f(v, u);

    if (ballDistance < COLLISION_ROBOT_RADIUS)
    {
        bool ballIsRollingTowardsRobot =
            fabs(normaliseTheta(ballHeading + M_PI - ballRollDirection)) < COLLISION_ACCEPTABLE_HEADING_DIFF;

        if (ballIsRollingTowardsRobot)
        {
            // If the ball is rolling into the robot, it bounces off!
            kf.state(BALL_U_DIM, 0) *= -COLLISION_COEFF_OF_RESTITUTION;
            kf.state(BALL_V_DIM, 0) *= -COLLISION_COEFF_OF_RESTITUTION;
        }
        else
        {
            // If the ball is inside the robot collision radius, we push it out.
            float distanceIncrease = COLLISION_ROBOT_RADIUS - ballDistance;
            kf.state(BALL_X_DIM, 0) += distanceIncrease * cosf(ballHeading);
            kf.state(BALL_Y_DIM, 0) += distanceIncrease * sinf(ballHeading);
        }
    }
}

void MultiBallCMKF::predictWithOdometry(BCMKF &kf, const Odometry &odometry)
{
    kf.state(BALL_X_DIM, 0) -= odometry.forward;
    kf.state(BALL_Y_DIM, 0) -= odometry.left;

    AbsCoord ballPosRRC = AbsCoord(kf.state(BALL_X_DIM, 0), kf.state(BALL_Y_DIM, 0), 0);
    RRCoord ballPosRR = ballPosRRC.convertToRobotRelative();

    kf.state(BALL_X_DIM, 0) = ballPosRR.distance() * cosf(ballPosRR.heading() - odometry.turn);
    kf.state(BALL_Y_DIM, 0) = ballPosRR.distance() * sinf(ballPosRR.heading() - odometry.turn);

    AbsCoord ballVelRRC = AbsCoord(kf.state(BALL_U_DIM, 0), kf.state(BALL_V_DIM, 0), 0);
    RRCoord ballVelRR = ballVelRRC.convertToRobotRelative();

    kf.state(BALL_U_DIM, 0) = ballVelRR.distance() * cosf(ballVelRR.heading() - odometry.turn);
    kf.state(BALL_V_DIM, 0) = ballVelRR.distance() * sinf(ballVelRR.heading() - odometry.turn);

    // (TODO-kenji) I need an appropriate covariance transform here
    // this matrix rotation is derived by myself and seems to be the same
    // as B-Human's. I think though, that it's not right, but "good enough" for our use.
    // I think the 0's in the covariance parts shouldn't be 0...
    // Some genius in the future rUNSWift can derivce the actual covariance.
    Eigen::Matrix4f rotation;
    float rotationCos = cosf(-odometry.turn);
    float rotationSin = sinf(-odometry.turn);
    rotation << rotationCos, -rotationSin, 0, 0,
        rotationSin, rotationCos, 0, 0,
        0, 0, rotationCos, -rotationSin,
        0, 0, rotationSin, rotationCos;
    Eigen::Matrix4f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse
    check_finite(rotationInv, "ball predict with odom rotation inverse");
    kf.covariance = rotation * kf.covariance * rotationInv;
    check_finite(kf.covariance, "ball predict with odom covariance");
    // std::cout << "Covariance\n"
    //           << covariance << std::endl;

    // (TODO-Carlin) Do we need some equivalent of the following?
    // // Calculate uncertainty propagation thing (http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf)
    // Eigen::Matrix<float, 3, 3> propagationJacobian = Eigen::Matrix<float, 3, 3>::Identity();
    // float distanceTravelled = sqrt(pow(odometry.forward, 2) + pow(odometry.left, 2));
    // propagationJacobian(0, 2) = -distanceTravelled * sinf(kf.state(ME_H_DIM, 0));
    // propagationJacobian(1, 2) = distanceTravelled * cosf(kf.state(ME_H_DIM, 0));
    // kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM) = propagationJacobian * kf.covariance.block<3, 3>(ME_X_DIM, ME_X_DIM) * propagationJacobian.transpose();

    // kf.covariance.block<2, 2>(ME_X_DIM, ME_X_DIM) += odometryCovariance;
    // kf.covariance(ME_H_DIM, ME_H_DIM) += params->odometryHeadingMultiplyFactor * (odometry.turn * odometry.turn);
    // check_finite(kf.covariance, "MMCMKF covariance");
}

void MultiBallCMKF::update(
    BCMKF &kf,
    const BallInfo &ballInfo,
    const EstimatorInfoIn &EstimatorInfoIn)
{
    // Error in observation in polar coordinates
    float stdObservationDistBase;
    float stdObservationDistIncreaseRate;
    float stdObservationHead;

    const ActionCommand::Body &actionCommand = EstimatorInfoIn.actionCommandBody;
    const SensorValues &sensorValues = EstimatorInfoIn.sensorValues;

    // If we're standing or crouching, we have less errors (important for predicting changing velocities)
    bool actionTypeCanGetAccurateMeasurement =
        (actionCommand.actionType == ActionCommand::Body::WALK && actionCommand.forward == 0 && actionCommand.left == 0 && actionCommand.turn == 0) ||
        actionCommand.actionType == ActionCommand::Body::STAND ||
        actionCommand.actionType == ActionCommand::Body::GOALIE_STAND;

    bool gyroscopeStable =
        sensorValues.sensors[Sensors::InertialSensor_GyroscopeX] < DEG2RAD(4) &&
        sensorValues.sensors[Sensors::InertialSensor_GyroscopeY] < DEG2RAD(4);

    if (actionTypeCanGetAccurateMeasurement && gyroscopeStable)
    {
        stdObservationDistBase = params->BallStdObservationDistBaseStanding;
        stdObservationDistIncreaseRate = params->BallStdObservationDistIncreaseRateStanding;
        stdObservationHead = DEG2RAD(params->BallStdObservationHeadStanding);
    }
    else
    {
        stdObservationDistBase = params->BallStdObservationDistBaseWalking;
        stdObservationDistIncreaseRate = params->BallStdObservationDistIncreaseRateWalking;
        stdObservationHead = DEG2RAD(params->BallStdObservationHeadWalking);
    }

    // Get blockedState and blockedCovariance
    Eigen::Block<BallStateVector, NUM_DIM_BALL_TOTAL, 1> blockedState = kf.state.segment<NUM_DIM_BALL_TOTAL>(BALL_X_DIM);
    Eigen::Block<BallCovarianceMatrix, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> blockedCovariance =
        kf.covariance.block<NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL>(BALL_X_DIM, BALL_X_DIM);

    // Calculate observation vector
    float x = ballInfo.rr.distance() * cosf(ballInfo.rr.heading());
    float y = ballInfo.rr.distance() * sinf(ballInfo.rr.heading());
    Eigen::Matrix<float, NUM_DIM_BALL_POS, 1> observationVector;
    observationVector << x, y;
    check_finite(observationVector, "ball update observation vector");
    // std::cout << "Observation Vector\n"
    //           << observationVector << std::endl;

    // Calculate innovation vector
    Eigen::Matrix<float, NUM_DIM_BALL_POS, 1> innovationVector =
        observationVector - ballObservationModel * blockedState;
    check_finite(innovationVector, "ball update innovation vector");
    // std::cout << "Innovation Vector\n"
    //           << innovationVector << std::endl;

    // Calculate observation covariance
    Eigen::Matrix<float, 2, 2> observationCovarianceRRC = Eigen::Matrix<float, 2, 2>::Zero();
    float stdObservationRRForward = stdObservationDistBase + stdObservationDistIncreaseRate * ballInfo.rr.distance();
    float stdObservationRRLeft = ballInfo.rr.distance() * stdObservationHead;
    observationCovarianceRRC(0, 0) = (stdObservationRRForward * stdObservationRRForward);
    observationCovarianceRRC(1, 1) = (stdObservationRRLeft * stdObservationRRLeft);
    check_finite(observationCovarianceRRC, "ball update observation covariance RRC");
    // std::cout << "Observation Covariance RRC\n"
    //           << observationCovarianceRRC << std::endl;

    Eigen::Matrix2f rotation;
    float heading = ballInfo.rr.heading();
    rotation << cosf(heading), -sinf(heading), sinf(heading), cosf(heading);
    Eigen::Matrix2f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse

    Eigen::Matrix2f observationCovariance = rotation * observationCovarianceRRC * rotationInv;
    check_finite(observationCovariance, "ball update observation covariance");
    // std::cout << "Observation Covariance\n"
    //           << observationCovariance << std::endl;

    // Calculate innovation covariance
    Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_POS> innovationCovariance =
        observationCovariance + ballObservationModel * blockedCovariance * ballObservationModelTranspose;
    check_finite(innovationCovariance, "ball update innovation covariance");
    // std::cout << "Innovation Covariance\n"
    //           << innovationCovariance << std::endl;

    // Calculate Kalman Gain
    Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_POS> kalmanGain =
        blockedCovariance * ballObservationModelTranspose * innovationCovariance.inverse();
    check_finite(kalmanGain, "ball update kalman gain");
    // std::cout << "Kalman Gain\n"
    //           << kalmanGain << std::endl;

    // Update State Vector
    blockedState += kalmanGain * innovationVector;
    check_finite(blockedState, "ball update blocked state");
    // std::cout << "Blocked State\n"
    //           << blockedState << std::endl;

    // Update Covariance Matrix
    Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> identity =
        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL>::Identity();
    blockedCovariance = (identity - kalmanGain * ballObservationModel) * blockedCovariance * (identity - kalmanGain * ballObservationModel).transpose() +
                        kalmanGain * observationCovariance * kalmanGain.transpose();
    check_finite(blockedCovariance, "ball update blocked covariance");
    // std::cout << "Blocked Covariance\n"
    //           << blockedCovariance << std::endl;

    // (TODO-Carlin) do we need to handle the above being non finite for the weight adjustment
    Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_POS> innovationCovarianceInv =
        innovationCovariance.inverse();
    check_finite(innovationCovarianceInv, "MMCMKF update innovation covariance inverse");

    // Adjust weight
    float weightAdjustment = 1.0;
    float weightUpdate = innovationVector.transpose() * innovationCovarianceInv * innovationVector;
    weightAdjustment = exp(-0.25 * weightUpdate);
    weightAdjustment = std::max(std::min(weightAdjustment, 1.0f), 0.01f);
    kf.weight += params->ballWeightGrowth;
    kf.weight *= weightAdjustment;
    kf.weight = std::min(200.0f, kf.weight);
}

bool noWeight(BCMKF &kf)
{
    return kf.weight < 0;
}

void MultiBallCMKF::mergeCMKFs()
{
    for (std::vector<BCMKF>::iterator belief = kfs.begin(); belief != kfs.end(); ++belief)
    {
        if (belief->weight > 0)
        {
            for (std::vector<BCMKF>::iterator belief2 = belief + 1; belief2 != kfs.end(); ++belief2)
            {
                if (belief2->weight > 0)
                {
                    if (similar((*belief), (*belief2)))
                    {
                        merge((*belief), (*belief2));
                    }
                }
            }
        }
    }

    kfs.erase(std::remove_if(kfs.begin(), kfs.end(), noWeight), kfs.end());
}

bool MultiBallCMKF::similar(BCMKF &kf1, BCMKF &kf2)
{
    // For now, just compare states.
    // TODO: This should probably be extended to check covariances too
    float xDiff = kf1.state(BALL_X_DIM, 0) - kf2.state(BALL_X_DIM, 0);
    float yDiff = kf1.state(BALL_Y_DIM, 0) - kf2.state(BALL_Y_DIM, 0);

    // TODO: similarThreshConstant is used for both determining if an observation is for a new ball
    //       or if two kfs should be combined. This one should probably be smaller
    if (fabs(xDiff) < params->similarThreshConstant && fabs(yDiff) < params->similarThreshConstant)
    {
        return true;
    }

    return false;
}

void MultiBallCMKF::merge(BCMKF &kf1, BCMKF &kf2)
{
    /*
     * We take a weight average if the weights are relatively similar to each other.
     * We don't take the weighted average if one weight is significanlty greater than
     * the other, since it causes drift (the drift is explained in the following paper)
     * https://www.cs.utexas.edu/~pstone/Courses/393Rfall11/resources/RC09-Quinlan.pdf
     */
    float sumWeights = kf1.weight + kf2.weight;

    if (kf1.weight > 10.0 * kf2.weight)
    {
        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
    else if (kf2.weight > 10.0 * kf1.weight)
    {
        kf1.state = kf2.state;
        kf1.covariance = kf2.covariance;
        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
    else
    {
        float kf1Ratio = kf1.weight / sumWeights;
        float kf2Ratio = kf2.weight / sumWeights;

        kf1.covariance = kf1Ratio * kf1.covariance + kf2Ratio * kf2.covariance;

        kf1.weight = sumWeights;
        kf2.weight = -1;
    }
}

void MultiBallCMKF::decayCMKFWeights()
{
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        const float distance = kfs[i].getBallCoordRRC().convertToRobotRelative().distance();
        // Decay balls further from the robot slower. Give the nao a chance to walk up to a ball before giving up on it
        if(distance < 1500)
            kfs[i].weight -= params->ballCloseDecayRate;
        else
            kfs[i].weight -= params->ballFarDecayRate;
    }
}

void MultiBallCMKF::deleteLowWeightCMKFs()
{
    std::vector<BCMKF> newCMKFs;
    for (unsigned i = 0; i < kfs.size(); ++i)
    {
        if (kfs[i].weight > 0)
            newCMKFs.push_back(kfs[i]);
    }

    kfs = newCMKFs;
}

void MultiBallCMKF::deleteOffFieldCMKFs(const EstimatorInfoOut &estimatorInfoOut)
{
    float fieldXClip = FIELD_LENGTH / 2 + 700;
    float fieldYClip = FIELD_WIDTH / 2 + 700;

    std::vector<BCMKF> newCMKFs;
    for (unsigned i = 0; i < kfs.size(); i++)
    {
        BCMKF &kf = kfs[i];
        AbsCoord robotPos = estimatorInfoOut.robotPos;
        RRCoord ballPosRR = kf.getBallCoordRRC().convertToRobotRelative();
        float ballPosWorldX = robotPos.x() + ballPosRR.distance() * cosf(robotPos.theta() + ballPosRR.heading());
        float ballPosWorldY = robotPos.y() + ballPosRR.distance() * sinf(robotPos.theta() + ballPosRR.heading());

        if (fabs(ballPosWorldX) > fieldXClip || fabs(ballPosWorldY) > fieldYClip)
        {
            continue;
        }
        newCMKFs.push_back(kfs[i]);
    }
    kfs = newCMKFs;
}
