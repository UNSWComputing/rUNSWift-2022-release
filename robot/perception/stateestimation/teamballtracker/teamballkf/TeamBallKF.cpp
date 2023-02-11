#include "TeamBallKF.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoOut.hpp"
#include "perception/stateestimation/teamballtracker/teamballkf/TeamBallKFTransitioner.hpp"
#include "types/BroadcastData.hpp"
#include "utils/Logger.hpp"
#include "TeamBallKFParams.hpp"
#include <cstdio>
#include "utils/eigen_helpers.hpp"

static const TeamBallCovarianceMatrix getDefaultCovariance() {
   TeamBallCovarianceMatrix defaultCovariance = TeamBallCovarianceMatrix::Zero();
   defaultCovariance(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) = 1000000;
   defaultCovariance(TEAM_BALL_Y_DIM, TEAM_BALL_Y_DIM) = 1000000;
   defaultCovariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
   defaultCovariance(TEAM_BALL_V_DIM, TEAM_BALL_V_DIM) = 1000;
   return defaultCovariance;
}

static const TeamBallStateVector getMinState() {
   TeamBallStateVector minState;
   minState << -FIELD_LENGTH/2, -FIELD_WIDTH/2, -10000, -10000;
   return minState;
}

static const TeamBallStateVector getMaxState() {
   TeamBallStateVector maxState;
   maxState << FIELD_LENGTH/2, FIELD_WIDTH/2, 10000, 10000;
   return maxState;
}

static const TeamBallStateVector MIN_STATE = getMinState();
static const TeamBallStateVector MAX_STATE = getMaxState();
static const TeamBallCovarianceMatrix DEFAULT_COVARIANCE = getDefaultCovariance();

static void setWithinField(TeamBallStateVector &state) {
   state << MAX_STATE.cwiseMin(MIN_STATE.cwiseMax(state));
}

TeamBallKF::TeamBallKF(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
{
    transitioner = new TeamBallKFTransitioner(estimatorInfoInit, this);
    state.setZero();
    covariance = DEFAULT_COVARIANCE;

    params = new TeamBallKFParams();
}

TeamBallKF::~TeamBallKF()
{
   delete params;
}

/* Function that gets called for every frame */
void TeamBallKF::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    transitioner->handleTransition(estimatorInfoIn);

    predict(estimatorInfoIn.dtInSeconds);
    bool hadTeamBallUpdate = update(estimatorInfoIn.incomingBroadcastData);

    estimatorInfoOut.teamBallPos = getBallAbsCoord();
    estimatorInfoOut.teamBallVel = getBallVelAbsCoord();
    estimatorInfoOut.teamBallPosUncertainty = getBallPosUncertainty();
    estimatorInfoOut.hadTeamBallUpdate = hadTeamBallUpdate;
}

void TeamBallKF::predict(const float dtInSeconds)
{
    if (dtInSeconds <= 1) {
        /* Followed "Example application, technical" from "https://en.wikipedia.org/wiki/Kalman_filter" */

        // Error in acceleration (TODO: tune these values)
        float stdAccelerationX = params->ballStdAccelerationX;
        float stdAccelerationY = params->ballStdAccelerationY;

        // Take part of the kalman filter
        Eigen::Block<TeamBallStateVector, NUM_DIM_TEAM_BALL_TOTAL, 1> blockedState = state.segment<
            NUM_DIM_TEAM_BALL_TOTAL>(TEAM_BALL_X_DIM);
        Eigen::Block<TeamBallCovarianceMatrix, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> blockedCovariance =
            covariance.block<NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL>(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM);

        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> stateTransitionMatrix;
        stateTransitionMatrix.setIdentity();
        stateTransitionMatrix(0, 2) = dtInSeconds;
        stateTransitionMatrix(1, 3) = dtInSeconds;

        float dt2InSeconds2 = dtInSeconds * dtInSeconds;
        float halfDt3InSeconds3 = dt2InSeconds2 * dtInSeconds / 2;
        float quarterDt4InSeconds4 = dt2InSeconds2 * dt2InSeconds2 / 4;

        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> processNoiseCovarianceA;
        processNoiseCovarianceA.setZero();
        processNoiseCovarianceA(0, 0) = quarterDt4InSeconds4;
        processNoiseCovarianceA(0, 2) = halfDt3InSeconds3;
        processNoiseCovarianceA(2, 0) = halfDt3InSeconds3;
        processNoiseCovarianceA(2, 2) = dt2InSeconds2;

        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> processNoiseCovarianceB;
        processNoiseCovarianceB.setZero();
        processNoiseCovarianceA(1, 1) = quarterDt4InSeconds4;
        processNoiseCovarianceA(1, 3) = halfDt3InSeconds3;
        processNoiseCovarianceA(3, 1) = halfDt3InSeconds3;
        processNoiseCovarianceA(3, 3) = dt2InSeconds2;

        // Use knowledge about friction for control input (DO we need a covariance for this?)
        float controlVector;
        float u = state(TEAM_BALL_U_DIM, 0);
        float v = state(TEAM_BALL_V_DIM, 0);
        float ballSpeed = hypotf(u, v);
        float ballRollDirection = atan2f(v, u);
        controlVector = std::max(TEAM_BALL_ACCELERATION, (0 - ballSpeed) / dtInSeconds);

        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1> controlInputModel;
        controlInputModel(0, 0) = dt2InSeconds2 / 2 * cosf(ballRollDirection);
        controlInputModel(1, 0) = dt2InSeconds2 / 2 * sinf(ballRollDirection);
        controlInputModel(2, 0) = dtInSeconds * cosf(ballRollDirection);
        controlInputModel(3, 0) = dtInSeconds * sinf(ballRollDirection);

        blockedState = stateTransitionMatrix * blockedState + controlInputModel * controlVector;
        setWithinField(state);
        check_finite(state, "teamballKF predict state");
        blockedCovariance =
            stateTransitionMatrix * blockedCovariance * stateTransitionMatrix.transpose() +
            processNoiseCovarianceA * (stdAccelerationX * stdAccelerationX) +
            processNoiseCovarianceB * (stdAccelerationY * stdAccelerationY);
        check_finite(covariance, "teamballKF predict covariance");
    }
}

bool TeamBallKF::update(const std::vector<BroadcastData> &incomingBroadcastData)
{
    bool haveUpdate = false; // bool whether there were any updates from teammembers

    for (unsigned i = 0; i < incomingBroadcastData.size(); ++i)
    {
        const BroadcastData &broadcastData = incomingBroadcastData[i];

        if (!broadcastData.sharedStateEstimationBundle.haveBallUpdate)
            // Don't update teamball if sender has no ball updates, because updates ALWAYS decrease uncertainty.
            // Uncertainty should not decrease if no one has seen the ball
            continue;

        haveUpdate = true;

        const AbsCoord &robotPos = broadcastData.sharedStateEstimationBundle.robotPos;
        const AbsCoord &ballPosRRC = broadcastData.sharedStateEstimationBundle.ballPosRRC;
        const AbsCoord &ballVelRRC = broadcastData.sharedStateEstimationBundle.ballVelRRC;

        // Calculate observation vector
        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1> observationVector = Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1>::Zero();
        RRCoord ballPosRR = ballPosRRC.convertToRobotRelative();
        observationVector(TEAM_BALL_X_DIM, 0) = robotPos.x() + ballPosRR.distance() * cosf(robotPos.theta() + ballPosRR.heading());
        observationVector(TEAM_BALL_Y_DIM, 0) = robotPos.y() + ballPosRR.distance() * sinf(robotPos.theta() + ballPosRR.heading());

        RRCoord ballVelRR = ballVelRRC.convertToRobotRelative();
        observationVector(TEAM_BALL_U_DIM, 0) = ballVelRR.distance() * cosf(robotPos.theta() + ballVelRR.heading());
        observationVector(TEAM_BALL_V_DIM, 0) = ballVelRR.distance() * sinf(robotPos.theta() + ballVelRR.heading());
        check_finite(observationVector, "team ball ovservation vector");
        // std::cout << "Observation Vector\n"
        //           << observationVector << std::endl;

        // Calculate innovation vector
        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1> innovationVector = Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, 1>::Zero();
        innovationVector = observationVector - state;
        // std::cout << "Innovation Vector\n"
        //           << innovationVector << std::endl;

        // Calculate observation covariance
        // For position covariance, we consider
        // 1. Covariance of ball
        // 2. Covariance of robot pose
        // 3. Covariance of robot heading
        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> observationCovariance = Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL>::Zero();
        Eigen::Matrix2f rotation;
        float heading = robotPos.theta();
        rotation << cosf(heading), -sinf(heading), sinf(heading), cosf(heading);
        Eigen::Matrix<float, 2, 2> headingErrorCovariance = Eigen::Matrix<float, 2, 2>::Zero();
        headingErrorCovariance(1, 1) = powf(ballPosRR.distance() * sinf(sqrtf(robotPos.var(2, 2))),2);
        Eigen::Matrix2f rotationInv = rotation.transpose(); // Rotation matrix is orthogonal, and hence transpose=inverse
        observationCovariance.block<NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS>(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) =
            rotation * ballPosRRC.var.block<NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS>(0, 0) * rotationInv +
            robotPos.var.block<NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS>(0, 0) +
            rotation * headingErrorCovariance * rotationInv;
        observationCovariance.block<NUM_DIM_TEAM_BALL_VEL, NUM_DIM_TEAM_BALL_VEL>(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) =
            rotation * ballVelRRC.var.block<NUM_DIM_TEAM_BALL_VEL, NUM_DIM_TEAM_BALL_VEL>(0, 0) * rotationInv;
        check_finite(observationCovariance, "team ball update observation covariance");
        // std::cout << "Observation Covariancer\n"
        //           << observationCovariance << std::endl;

        // Calculate innovation covariance
        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> innovationCovariance =
            observationCovariance + covariance;

        // Calculate innovation covariance inverse
        Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> innovationCovarianceInv =
            innovationCovariance.inverse();
        bool is_finite = check_finite(innovationCovarianceInv, "team ball innovation covariance inverse");

        // For now, we just ignore the update if this has NaN or +/- inf
        if (is_finite) {
            // Calculate optimal Kalman gain
            Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> kalmanGain =
                covariance * innovationCovarianceInv;
            // std::cout << "Kalman gain\n"
            //           << kalmanGain << std::endl;

            // Update state vector
            state += kalmanGain * innovationVector;
            setWithinField(state);
            check_finite(state, "team ball state update");
            // std::cout << "State\n"
            //           << state << std::endl;

            // Update covariance matrix
            Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL> identity = Eigen::Matrix<float, NUM_DIM_TEAM_BALL_TOTAL, NUM_DIM_TEAM_BALL_TOTAL>::Identity();
            covariance = (identity - kalmanGain) * covariance * (identity - kalmanGain).transpose() + kalmanGain * observationCovariance * kalmanGain.transpose();
            check_finite(covariance, "team ball update covariance");
            // std::cout << "Covariance\n"
            //           << covariance << std::endl;
        } else {
           llog(WARNING) << "!innovationCovariance.inverse().allFinite()\n"
                            "innovationCovariance:\n"
                         << innovationCovariance << "\n"
                            "innovationCovariance.inverse():\n"
                         << innovationCovarianceInv << std::endl;
           covariance = DEFAULT_COVARIANCE;
        }
    }
    return haveUpdate;
}

AbsCoord TeamBallKF::getBallAbsCoord()
{
    AbsCoord ballAbsCoord = AbsCoord(state(TEAM_BALL_X_DIM, 0), state(TEAM_BALL_Y_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_TEAM_BALL_POS; i++)
    {
        for (int j = 0; j < NUM_DIM_TEAM_BALL_POS; j++)
        {
            ballAbsCoord.var(i, j) = covariance(TEAM_BALL_X_DIM + i, TEAM_BALL_X_DIM + j);
        }
    }
    ballAbsCoord.weight = 1;
    return ballAbsCoord;
}

AbsCoord TeamBallKF::getBallVelAbsCoord()
{
    AbsCoord ballVelAbsCoord = AbsCoord(state(TEAM_BALL_U_DIM, 0), state(TEAM_BALL_V_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_TEAM_BALL_VEL; i++)
    {
        for (int j = 0; j < NUM_DIM_TEAM_BALL_VEL; j++)
        {
            ballVelAbsCoord.var(i, j) = covariance(TEAM_BALL_U_DIM + i, TEAM_BALL_U_DIM + j);
        }
    }
    ballVelAbsCoord.weight = 1;
    return ballVelAbsCoord;
}

// We define uncertainty as the area of the smallest possible rectangle that can be fit around the
// position covariance ellipse.
float TeamBallKF::getBallPosUncertainty()
{
    Eigen::Matrix<float, NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS> positionCovariance =
        covariance.block<NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS>(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM);
    Eigen::EigenSolver<Eigen::Matrix<float, NUM_DIM_TEAM_BALL_POS, NUM_DIM_TEAM_BALL_POS> > es(positionCovariance);

    float std_a = sqrt(abs(es.eigenvalues()[0])); // std of one of the ellipse axes
    float std_b = sqrt(abs(es.eigenvalues()[1])); // std of the other ellipse axis

    return std_a * std_b;
}
