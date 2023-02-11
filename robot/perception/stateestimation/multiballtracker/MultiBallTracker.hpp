#ifndef MULTI_BALL_TRACKER_HPP
#define MULTI_BALL_TRACKER_HPP

#include "perception/stateestimation/Estimator.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class MultiBallCMKF;

class MultiBallTracker : public Estimator
{
  public:
    MultiBallTracker(const EstimatorInfoInit &estimatorInfoInit);
    ~MultiBallTracker();
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:
    MultiBallCMKF *multiballcmkf;
};


#endif // EGO_BALL_TRACKER_HPP