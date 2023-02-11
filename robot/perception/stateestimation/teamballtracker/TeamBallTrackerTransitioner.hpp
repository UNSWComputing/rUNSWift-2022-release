#ifndef TEAM_BALL_TRACKER_TRANSITIONER_HPP
#define TEAM_BALL_TRACKER_TRANSITIONER_HPP

#include <stdint.h>
#include "utils/Timer.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;

class TeamBallTrackerTransitioner
{
  public:
    TeamBallTrackerTransitioner(const EstimatorInfoInit &estimatorInfoInit);
    void handleTransition(const EstimatorInfoIn &estimatorInfoIn);
    virtual ~TeamBallTrackerTransitioner(){};

  protected:
    const EstimatorInfoInit &estimatorInfoInit;

  private:

    uint8_t prevSetPlay;

    virtual void resetToOffenseGoalKickPosition() = 0;
    virtual void resetToDefenseGoalKickPosition() = 0;
    virtual void resetToOffenseCornerKickPosition() = 0;
    virtual void resetToDefenseCornerKickPosition() = 0;
};

#endif // TEAM_BALL_TRACKER_TRANSITIONER_HPP
