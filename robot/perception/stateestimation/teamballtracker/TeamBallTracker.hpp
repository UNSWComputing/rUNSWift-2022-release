#ifndef TEAM_BALL_TRACKER_HPP
#define TEAM_BALL_TRACKER_HPP

#include "perception/stateestimation/Estimator.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class TeamBallKF;

class TeamBallTracker : public Estimator
{
  public:
    TeamBallTracker(const EstimatorInfoInit &estimatorInfoInit);
    ~TeamBallTracker();
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:
    TeamBallKF * teamBallKF;
};


#endif // TEAM_BALL_TRACKER_HPP