#ifndef LOCALISER_HPP
#define LOCALISER_HPP

#include "perception/stateestimation/Estimator.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class MultiModalCMKF;

class Localiser : public Estimator
{
  public:
    Localiser(const EstimatorInfoInit &estimatorInfoInit);
    ~Localiser();
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

  private:

    MultiModalCMKF *mmcmkf;

    /* Fills the "canLocaliseInState" field in EstimatorInfoMiddle */
    void fillCanLocaliseInState(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle);

    /* Fills the "canDoObservations" field in EstimatorInfoMiddle */
    void fillCanDoObservations(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle);

};

#endif // LOCALISER_HPP