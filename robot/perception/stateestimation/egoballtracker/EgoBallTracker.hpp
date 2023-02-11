#ifndef EGO_BALL_TRACKER_HPP
#define EGO_BALL_TRACKER_HPP

#include "perception/stateestimation/Estimator.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class BallCMKF;

class EgoBallTracker : public Estimator
{
  public:
    EgoBallTracker(const EstimatorInfoInit &estimatorInfoInit);
    ~EgoBallTracker();
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:
    BallCMKF *ballcmkf;
};


#endif // EGO_BALL_TRACKER_HPP