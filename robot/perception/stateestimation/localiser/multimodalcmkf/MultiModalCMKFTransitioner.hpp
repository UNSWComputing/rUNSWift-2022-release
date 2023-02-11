#ifndef MULTI_MODAL_CMKF_LOCALISER_TRANSITIONER_HPP
#define MULTI_MODAL_CMKF_LOCALISER_TRANSITIONER_HPP

#include "perception/stateestimation/localiser/LocaliserTransitioner.hpp"

class MultiModalCMKF;
class EstimatorInfoInit;

class MultiModalCMKFTransitioner : public LocaliserTransitioner
{
  public:
    MultiModalCMKFTransitioner(
        const EstimatorInfoInit &estimatorInfoInit,
        MultiModalCMKF *mmcmkf);

  private:

    MultiModalCMKF *mmcmkf;

    void resetToInitialPose();
    void resetToGameInitialPose();
    void resetToOneVsOneInitialPose();
    void resetToSpecifiedInitialPose();
    void resetToUnpenalisedPose();
    void resetToManualPlacementPoseOffense();
    void resetToManualPlacementPoseDefense();
    void resetToPenaltyshootPhasePoseOffense();
    void resetToPenaltyshootPhasePoseDefense();
    void resetToPenaltyshootPhasePlayerSelectedPose();
};

#endif // MULTI_MODAL_CMKF_LOCALISER_TRANSITIONER_HPP
