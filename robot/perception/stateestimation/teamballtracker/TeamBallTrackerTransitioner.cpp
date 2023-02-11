#include "TeamBallTrackerTransitioner.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"

TeamBallTrackerTransitioner::TeamBallTrackerTransitioner(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
    , prevSetPlay(estimatorInfoInit.setPlay)
{
}

void TeamBallTrackerTransitioner::handleTransition(const EstimatorInfoIn &estimatorInfoIn)
{
    uint8_t newSetPlay = estimatorInfoIn.setPlay;
    bool weAreKickingTeam = estimatorInfoIn.kickingTeam == estimatorInfoInit.teamNumber;
    if (prevSetPlay != newSetPlay)
    {
        if (newSetPlay == SET_PLAY_GOAL_KICK)
        {
            if (weAreKickingTeam)
            {
                resetToOffenseGoalKickPosition();
            }
            else
            {
                resetToDefenseGoalKickPosition();
            }
        }
        else if (newSetPlay == SET_PLAY_CORNER_KICK)
        {
            if (weAreKickingTeam)
            {
                resetToOffenseCornerKickPosition();
            }
            else
            {
                resetToDefenseCornerKickPosition();
            }
        }
    }

    // Update prev variables
    prevSetPlay = newSetPlay;
}
