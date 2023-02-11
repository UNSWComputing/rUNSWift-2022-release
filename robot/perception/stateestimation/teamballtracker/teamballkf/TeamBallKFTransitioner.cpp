#include "TeamBallKFTransitioner.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "TeamBallKF.hpp"
#include <Eigen/Eigen>

TeamBallKFTransitioner::TeamBallKFTransitioner(
    const EstimatorInfoInit &estimatorInfoInit,
    TeamBallKF *tbkf)
    : TeamBallTrackerTransitioner(estimatorInfoInit)
    , tbkf(tbkf)
{
}

void TeamBallKFTransitioner::resetToOffenseGoalKickPosition()
{
    // Put the ball in the goal free kick position, which it was closer to, since
    // Ball should be placed to the closer free kick position to where it
    // went out.
    float ballY = tbkf->state(TEAM_BALL_Y_DIM, 0);
    TeamBallStateVector state = TeamBallStateVector::Zero();
    state(TEAM_BALL_X_DIM, 0) = -GOAL_KICK_ABS_X;
    state(TEAM_BALL_Y_DIM, 0) = GOAL_KICK_ABS_Y * (ballY > 0 ? 1 : -1);
    state(TEAM_BALL_U_DIM, 0) = 0;
    state(TEAM_BALL_V_DIM, 0) = 0;
    tbkf->state = state;

    TeamBallCovarianceMatrix covariance = TeamBallCovarianceMatrix::Zero();
    covariance(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) = 10000;
    covariance(TEAM_BALL_Y_DIM, TEAM_BALL_Y_DIM) = 10000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    tbkf->covariance = covariance;
}

void TeamBallKFTransitioner::resetToDefenseGoalKickPosition()
{
    // Put the ball in the goal free kick position, which it was closer to, since
    // Ball should be placed to the closer free kick position to where it
    // went out.
    float ballY = tbkf->state(TEAM_BALL_Y_DIM, 0);
    TeamBallStateVector state = TeamBallStateVector::Zero();
    state(TEAM_BALL_X_DIM, 0) = GOAL_KICK_ABS_X;
    state(TEAM_BALL_Y_DIM, 0) = GOAL_KICK_ABS_Y * (ballY > 0 ? 1 : -1);
    state(TEAM_BALL_U_DIM, 0) = 0;
    state(TEAM_BALL_V_DIM, 0) = 0;
    tbkf->state = state;

    TeamBallCovarianceMatrix covariance = TeamBallCovarianceMatrix::Zero();
    covariance(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) = 10000;
    covariance(TEAM_BALL_Y_DIM, TEAM_BALL_Y_DIM) = 10000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    tbkf->covariance = covariance;
}

void TeamBallKFTransitioner::resetToOffenseCornerKickPosition()
{
    // Put the ball in the corner, which it was closer to, since
    // Ball should be placed to the closer corner to where it
    // went out.
    float ballY = tbkf->state(TEAM_BALL_Y_DIM, 0);
    TeamBallStateVector state = TeamBallStateVector::Zero();
    state(TEAM_BALL_X_DIM, 0) = CORNER_KICK_ABS_X;
    state(TEAM_BALL_Y_DIM, 0) = CORNER_KICK_ABS_Y * (ballY > 0 ? 1 : -1);
    state(TEAM_BALL_U_DIM, 0) = 0;
    state(TEAM_BALL_V_DIM, 0) = 0;
    tbkf->state = state;

    TeamBallCovarianceMatrix covariance = TeamBallCovarianceMatrix::Zero();
    covariance(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) = 10000;
    covariance(TEAM_BALL_Y_DIM, TEAM_BALL_Y_DIM) = 10000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    tbkf->covariance = covariance;
}

void TeamBallKFTransitioner::resetToDefenseCornerKickPosition()
{
    // Put the ball in the corner, which it was closer to, since
    // Ball should be placed to the closer corner to where it
    // went out.
    float ballY = tbkf->state(TEAM_BALL_Y_DIM, 0);
    TeamBallStateVector state = TeamBallStateVector::Zero();
    state(TEAM_BALL_X_DIM, 0) = -CORNER_KICK_ABS_X;
    state(TEAM_BALL_Y_DIM, 0) = CORNER_KICK_ABS_Y * (ballY > 0 ? 1 : -1);
    state(TEAM_BALL_U_DIM, 0) = 0;
    state(TEAM_BALL_V_DIM, 0) = 0;
    tbkf->state = state;

    TeamBallCovarianceMatrix covariance = TeamBallCovarianceMatrix::Zero();
    covariance(TEAM_BALL_X_DIM, TEAM_BALL_X_DIM) = 10000;
    covariance(TEAM_BALL_Y_DIM, TEAM_BALL_Y_DIM) = 10000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    covariance(TEAM_BALL_U_DIM, TEAM_BALL_U_DIM) = 1000;
    tbkf->covariance = covariance;
}

