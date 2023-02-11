#ifndef ESTIMATOR_INFO_MIDDLE_HPP
#define ESTIMATOR_INFO_MIDDLE_HPP

#include "types/Odometry.hpp"

class EstimatorInfoMiddle
{
  public:
    EstimatorInfoMiddle(){};

    bool canLocaliseInState;
    bool canDoObservations;
    
};

#endif // ESTIMATOR_INFO_MIDDLE_HPP