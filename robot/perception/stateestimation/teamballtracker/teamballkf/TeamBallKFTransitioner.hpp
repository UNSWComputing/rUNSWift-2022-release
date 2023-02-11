#ifndef TEAM_BALL_KF_TRANSITIONER_HPP
#define TEAM_BALL_KF_TRANSITIONER_HPP

#include "perception/stateestimation/teamballtracker/TeamBallTrackerTransitioner.hpp"

class TeamBallKF;
class EstimatorInfoInit;

class TeamBallKFTransitioner : public TeamBallTrackerTransitioner
{
  public:
    TeamBallKFTransitioner(
        const EstimatorInfoInit &estimatorInfoInit,
        TeamBallKF *tbkf);

  private:

    TeamBallKF *tbkf;

    void resetToOffenseGoalKickPosition();
    void resetToDefenseGoalKickPosition();
    void resetToOffenseCornerKickPosition();
    void resetToDefenseCornerKickPosition();
};

#endif // TEAM_BALL_KF_TRANSITIONER_HPP
