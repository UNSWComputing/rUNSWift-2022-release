#ifndef LOCALISER_TRANSITIONER_HPP
#define LOCALISER_TRANSITIONER_HPP

#include <stdint.h>
#include "utils/Timer.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;

class LocaliserTransitioner
{
  public:
    LocaliserTransitioner(const EstimatorInfoInit &estimatorInfoInit);
    void handleTransition(const EstimatorInfoIn &estimatorInfoIn);
    virtual ~LocaliserTransitioner(){};

  protected:
    const EstimatorInfoInit &estimatorInfoInit;
    void resetToInitialPose();

  private:

    uint8_t prevCompetitionType;
    uint8_t prevGameState;
    uint8_t prevGamePhase;
    uint8_t prevPenalty;
    bool prevPickedup;

    Timer penalisedTimer;
    Timer refPickupTimer;
    Timer unpenalisedTimer;
    bool pickedUpDuringPenalised;

    virtual void resetToGameInitialPose() = 0;
    virtual void resetToOneVsOneInitialPose() = 0;
    virtual void resetToSpecifiedInitialPose() = 0;
    virtual void resetToUnpenalisedPose() = 0;
    virtual void resetToManualPlacementPoseOffense() = 0;
    virtual void resetToManualPlacementPoseDefense() = 0;
    virtual void resetToPenaltyshootPhasePoseOffense() = 0;
    virtual void resetToPenaltyshootPhasePoseDefense() = 0;
    virtual void resetToPenaltyshootPhasePlayerSelectedPose() = 0;
};

#endif // LOCALISER_TRANSITIONER_HPP
