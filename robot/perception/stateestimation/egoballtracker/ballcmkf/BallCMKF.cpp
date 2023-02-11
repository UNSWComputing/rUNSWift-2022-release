#include "BallCMKF.hpp"

#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoOut.hpp"
#include "BallCMKFParams.hpp"
#include "utils/basic_maths.hpp"
#include "utils/eigen_helpers.hpp"

static const Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_TOTAL>
    ballObservationModel((Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_TOTAL>() << 1, 0, 0, 0, 0, 1, 0, 0).finished());

static const Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_POS>
    ballObservationModelTranspose(ballObservationModel.transpose());

BallCMKF::BallCMKF(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
{
    state.setZero();
    covariance.setZero();
    covariance(BALL_X_DIM, BALL_X_DIM) = 1000000;
    covariance(BALL_Y_DIM, BALL_Y_DIM) = 1000000;
    covariance(BALL_U_DIM, BALL_U_DIM) = 1000;
    covariance(BALL_V_DIM, BALL_V_DIM) = 1000;

    params = new BallCMKFParams();
}

BallCMKF::~BallCMKF()
{
   delete params;
}

void BallCMKF::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    // Once we have sent out the data, reset haveOutgoingBallUpdate
    if (!estimatorInfoIn.havePendingOutgoingSharedBundle)
        haveOutgoingBallUpdate = false;

    predictCollision();
    predict(estimatorInfoIn.dtInSeconds);
    predictWithOdometry(estimatorInfoIn.odometryDiff);
    if (!estimatorInfoIn.balls.empty())
    {

        const BallInfo &ballObservation = estimatorInfoIn.balls[0];

        // Sometimes we get nan in the observations, ignore these
        if (!check_finite(ballObservation.rr.vec, __FILE__ "\ballObservation.rr.vec") ||
            !check_finite(ballObservation.rr.var, __FILE__ "\ballObservation.rr.var"))
        {
            std::cerr << "ballObservation:\t" << ballObservation << "\n";
        }
        else
        {
            update(ballObservation, estimatorInfoIn.actionCommandBody, estimatorInfoIn.sensorValues);
            haveOutgoingBallUpdate = true;
        }
    }

    AbsCoord ballPosRRC = getBallCoordRRC();
    estimatorInfoOut.ballPosRRC = ballPosRRC;

    AbsCoord ballVelRRC = getBallVelCoordRRC();
    estimatorInfoOut.ballVelRRC = ballVelRRC;

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
                             std::max(- (FIELD_LENGTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f, ballPosWorldX));
    ballPosWorldY = std::min((FIELD_WIDTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f,
                             std::max(- (FIELD_WIDTH + POSSIBLE_OFF_FIELD_BALL_MARGIN) / 2.0f, ballPosWorldY));
    AbsCoord ballPosWorld = AbsCoord(ballPosWorldX, ballPosWorldY, 0);
    estimatorInfoOut.ballPos = ballPosWorld;

    Eigen::Matrix2f rotation;
    float heading = robotPos.theta();
    rotation << cosf(heading), -sinf(heading), sinf(heading), cosf(heading);
    Eigen::Matrix2f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse
    check_finite(rotationInv, "ball rotation inverse");

    Eigen::Matrix2f absCovariance = rotation * covariance.block<NUM_DIM_BALL_POS, NUM_DIM_BALL_POS>(BALL_X_DIM, BALL_X_DIM) * rotationInv;
    check_finite(absCovariance, "ball abbsCovariance");
    estimatorInfoOut.ballPos.var.block<2,2>(0,0) = absCovariance;

    estimatorInfoOut.ballPosRR = ballPosRR;
    RRCoord ballVelRR = ballVelRRC.convertToRobotRelative();
    estimatorInfoOut.ballVel = AbsCoord(
        ballVelRR.distance() * cosf(robotPos.theta() + ballVelRR.heading()),
        ballVelRR.distance() * sinf(robotPos.theta() + ballVelRR.heading()),
        0);
    // (TODO-kenji) up to here
}

void BallCMKF::predict(const float dtInSeconds)
{
    if (dtInSeconds <= 1) {
        /* Followed "Example application, technical" from "https://en.wikipedia.org/wiki/Kalman_filter" */

        // Error in acceleration (TODO: tune these values)
        float stdAccelerationX = params->ballStdAccelerationX;
        float stdAccelerationY = params->ballStdAccelerationY;

        // Take part of the kalman filter
        Eigen::Block<BallStateVector, NUM_DIM_BALL_TOTAL, 1> blockedState = state.segment<NUM_DIM_BALL_TOTAL>(BALL_X_DIM);
        Eigen::Block<BallCovarianceMatrix, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> blockedCovariance =
            covariance.block<NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL>(BALL_X_DIM, BALL_X_DIM);

        Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> stateTransitionMatrix;
        stateTransitionMatrix.setIdentity();
        stateTransitionMatrix(0, 2) = dtInSeconds;
        stateTransitionMatrix(1, 3) = dtInSeconds;
        check_finite(stateTransitionMatrix,  "ball state transition matrix");

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
        float u = state(BALL_U_DIM, 0);
        float v = state(BALL_V_DIM, 0);
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
void BallCMKF::predictCollision()
{
    float x = state(BALL_X_DIM, 0);
    float y = state(BALL_Y_DIM, 0);
    float u = state(BALL_U_DIM, 0);
    float v = state(BALL_V_DIM, 0);
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
            state(BALL_U_DIM, 0) *= -COLLISION_COEFF_OF_RESTITUTION;
            state(BALL_V_DIM, 0) *= -COLLISION_COEFF_OF_RESTITUTION;
        }
        else
        {
            // If the ball is inside the robot collision radius, we push it out.
            float distanceIncrease = COLLISION_ROBOT_RADIUS - ballDistance;
            state(BALL_X_DIM, 0) += distanceIncrease * cosf(ballHeading);
            state(BALL_Y_DIM, 0) += distanceIncrease * sinf(ballHeading);
        }
    }
}

void BallCMKF::predictWithOdometry(const Odometry &odometry)
{
    state(BALL_X_DIM, 0) -= odometry.forward;
    state(BALL_Y_DIM, 0) -= odometry.left;

    AbsCoord ballPosRRC = AbsCoord(state(BALL_X_DIM, 0), state(BALL_Y_DIM, 0), 0);
    RRCoord ballPosRR = ballPosRRC.convertToRobotRelative();

    state(BALL_X_DIM, 0) = ballPosRR.distance() * cosf(ballPosRR.heading() - odometry.turn);
    state(BALL_Y_DIM, 0) = ballPosRR.distance() * sinf(ballPosRR.heading() - odometry.turn);

    AbsCoord ballVelRRC = AbsCoord(state(BALL_U_DIM, 0), state(BALL_V_DIM, 0), 0);
    RRCoord ballVelRR = ballVelRRC.convertToRobotRelative();

    state(BALL_U_DIM, 0) = ballVelRR.distance() * cosf(ballVelRR.heading() - odometry.turn);
    state(BALL_V_DIM, 0) = ballVelRR.distance() * sinf(ballVelRR.heading() - odometry.turn);

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
    covariance = rotation * covariance * rotationInv;
    check_finite(covariance, "ball predict with odom covariance");
    // std::cout << "Covariance\n"
    //           << covariance << std::endl;
}

void BallCMKF::update(const BallInfo &ballInfo, const ActionCommand::Body &actionCommand, const SensorValues &sensorValues)
{
    // Error in observation in polar coordinates
    float stdObservationDistBase;
    float stdObservationDistIncreaseRate;
    float stdObservationHead;

    // If we're standing or crouching, we have less errors (important for predicting changing velocities)
    bool actionTypeCanGetAccurateMeasurement =
        (actionCommand.actionType == ActionCommand::Body::WALK  && actionCommand.forward == 0 && actionCommand.left == 0 && actionCommand.turn == 0) ||
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
    Eigen::Block<BallStateVector, NUM_DIM_BALL_TOTAL, 1> blockedState = state.segment<NUM_DIM_BALL_TOTAL>(BALL_X_DIM);
    Eigen::Block<BallCovarianceMatrix, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> blockedCovariance =
        covariance.block<NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL>(BALL_X_DIM, BALL_X_DIM);

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
}

AbsCoord BallCMKF::getBallCoordRRC()
{
    AbsCoord ballAbsCoord = AbsCoord(state(BALL_X_DIM, 0), state(BALL_Y_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_BALL_POS; i++)
    {
        for (int j = 0; j < NUM_DIM_BALL_POS; j++)
        {
            ballAbsCoord.var(i, j) = covariance(BALL_X_DIM + i, BALL_X_DIM + j);
        }
    }
    ballAbsCoord.weight = 1;
    return ballAbsCoord;
}

AbsCoord BallCMKF::getBallVelCoordRRC()
{
    AbsCoord ballVelAbsCoord = AbsCoord(state(BALL_U_DIM, 0), state(BALL_V_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_BALL_VEL; i++)
    {
        for (int j = 0; j < NUM_DIM_BALL_VEL; j++)
        {
            ballVelAbsCoord.var(i, j) = covariance(BALL_U_DIM + i, BALL_U_DIM + j);
        }
    }
    ballVelAbsCoord.weight = 1;
    return ballVelAbsCoord;
}
