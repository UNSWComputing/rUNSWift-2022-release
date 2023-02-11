#include "Localiser.hpp"

#include "multimodalcmkf/MultiModalCMKF.hpp"
#include "multimodalcmkf/MultiModalCMKFTransitioner.hpp"

#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"
#include "types/EstimatorInfoMiddle.hpp"
#include "types/EstimatorInfoOut.hpp"

Localiser::Localiser(
    const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)
{
    mmcmkf = new MultiModalCMKF(estimatorInfoInit);
}

Localiser::~Localiser()
{
    if (mmcmkf) delete mmcmkf;
}

void Localiser::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    // Fill in canLocaliseInState in estimatorInfoMiddle
    fillCanLocaliseInState(estimatorInfoIn, estimatorInfoMiddle);

    // Fill in canDoObservations in estimatorInfoMiddle
    fillCanDoObservations(estimatorInfoIn, estimatorInfoMiddle);

    // Tick Localisation Algorithm
    mmcmkf->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);

}

void Localiser::fillCanLocaliseInState(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle)
{
    // Default to true
    estimatorInfoMiddle.canLocaliseInState = true;

    // If we're running a GameController skill
    if (estimatorInfoInit.skill == "Game" || estimatorInfoInit.skill == "OneVsOne")
    {
        const uint8_t &state = estimatorInfoIn.state;
        // If out state is in ready, set or playing
        if (!(state == STATE_READY || state == STATE_SET || state == STATE_PLAYING))
        {
            estimatorInfoMiddle.canLocaliseInState = false;
        }
    }
}

void Localiser::fillCanDoObservations(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle)
{
    // Default to true
    estimatorInfoMiddle.canDoObservations = true;

    ActionCommand::Body::ActionType currentAction = estimatorInfoIn.active.body.actionType;
    bool isPickedUp = (currentAction == ActionCommand::Body::REF_PICKUP);
    bool isDiving = (currentAction == ActionCommand::Body::GOALIE_DIVE_LEFT || currentAction == ActionCommand::Body::GOALIE_DIVE_RIGHT);
    bool isDead = (currentAction == ActionCommand::Body::LIMP);
    bool isGettingUp =
        (currentAction == ActionCommand::Body::GETUP_FRONT) || (currentAction == ActionCommand::Body::GETUP_BACK) || (currentAction == ActionCommand::Body::TIP_OVER);

    if (isPickedUp || isDiving || isDead || isGettingUp)
    {
        estimatorInfoMiddle.canDoObservations = false;
    }
}
