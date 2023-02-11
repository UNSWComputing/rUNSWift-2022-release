#include "MultiBallTracker.hpp"
#include "multiballcmkf/MultiBallCMKF.hpp"

MultiBallTracker::MultiBallTracker(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)
{
    multiballcmkf = new MultiBallCMKF(estimatorInfoInit);
}

MultiBallTracker::~MultiBallTracker()
{
    if (multiballcmkf) delete multiballcmkf;
}

void MultiBallTracker::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    multiballcmkf ->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
}