#include "LocaliserTransitioner.hpp"
#include "types/EstimatorInfoInit.hpp"
#include "types/EstimatorInfoIn.hpp"

#define REF_PICKUP_TIMER_MIN_MSEC 500
#define PENALISED_TIMER_MIN_MSEC 20000
#define REF_FORGOT_TO_TURN_ROBOT_AROUND_MSEC 15000

LocaliserTransitioner::LocaliserTransitioner(const EstimatorInfoInit &estimatorInfoInit)
    : estimatorInfoInit(estimatorInfoInit)
    , prevCompetitionType(estimatorInfoInit.competitionType)
    , prevGameState(estimatorInfoInit.state)
    , prevGamePhase(estimatorInfoInit.gamePhase)
    , prevPenalty(PENALTY_NONE)
    , prevPickedup(false)
    , penalisedTimer()
    , refPickupTimer()
    , unpenalisedTimer()
    , pickedUpDuringPenalised(false)
{
}

void LocaliserTransitioner::handleTransition(const EstimatorInfoIn &estimatorInfoIn)
{

    uint8_t newCompetitionType = estimatorInfoIn.competitionType;
    uint8_t newGamePhase = estimatorInfoIn.gamePhase;
    uint8_t newGameState = estimatorInfoIn.state;
    uint8_t newPenalty = estimatorInfoIn.penalty;
    bool newPickedup = (estimatorInfoIn.active.body.actionType == ActionCommand::Body::REF_PICKUP);
    bool weAreKickingTeam = estimatorInfoIn.kickingTeam == estimatorInfoInit.teamNumber;

    // Competition Type Changes
    if (prevCompetitionType != newCompetitionType)
    {
        resetToInitialPose();
    }

    if (newCompetitionType == COMPETITION_TYPE_NORMAL || newCompetitionType == COMPETITION_TYPE_1VS1_CHALLENGE)
    {
        if (newGamePhase == GAME_PHASE_PENALTYSHOOT)
        {
            // Reset when we go to PLAYING
            if (prevGameState != newGameState && newGameState == STATE_PLAYING)
            {
                if (estimatorInfoIn.kickingTeam == estimatorInfoInit.teamNumber)
                    resetToPenaltyshootPhasePoseOffense();
                else
                    resetToPenaltyshootPhasePoseDefense();
            }

            // Reset positions when we get penalised / unpenalised so we can see in TCM who the specified taker is
            if (prevPenalty != newPenalty)
            {
                if (newPenalty == PENALTY_NONE)
                    resetToPenaltyshootPhasePlayerSelectedPose();
                else
                    resetToInitialPose();
            }
        }
        else
        {

            // Robot Gets Picked Up
            if (prevPickedup == false && newPickedup == true)
            {
                // Restart refPickupTimer
                refPickupTimer.restart();
            }

            // Robot Gets Placed Down
            if (prevPickedup == true && newPickedup == false)
            {
                // If we think we've been picked up for long enough,
                if (refPickupTimer.elapsed_ms() > REF_PICKUP_TIMER_MIN_MSEC)
                {
                    if (newGameState == STATE_SET)
                    {
                        // If in STATE_SET, reset to manual placement
                        if (weAreKickingTeam)
                            resetToManualPlacementPoseOffense();
                        else
                            resetToManualPlacementPoseDefense();
                    }
                    else if (newPenalty != PENALTY_NONE)
                    {
                        // If penalised, enable picked up during penalised flag
                        pickedUpDuringPenalised = true;
                    }
                    else if (unpenalisedTimer.elapsed_ms() < REF_FORGOT_TO_TURN_ROBOT_AROUND_MSEC &&
                        estimatorInfoInit.handleRefereeMistakes)
                    {
                        // Handle case where side ref forgot to turn robot around before it being unpenalised.
                        // Commonly, the side refs pick up the robot, turn it around and face it towards the field, even
                        // after it was unpenalised. To handle such cases, if we got unpenalised recently, and get picked up
                        // and placed back, we reset to unpenalised pose
                        if (newGameState == STATE_INITIAL) {
                            resetToInitialPose();
                        } else {
                            resetToUnpenalisedPose();
                        }
                    }
                }
            }

            // Robot Gets Penalised
            if (prevPenalty != newPenalty && newPenalty != PENALTY_NONE)
            {
                // Restart penalisedTimer
                penalisedTimer.restart();

                // Reset pickedUpDuringPenalised flag
                pickedUpDuringPenalised = false;
            }

            // Robot Gets Unpenalised
            if (prevPenalty != newPenalty && newPenalty == PENALTY_NONE)
            {
                if (prevPenalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)
                {
                    // If robot gets penalised in set, do not reset.
                    if (pickedUpDuringPenalised)
                    {
                        // If robot gets picked up, reset, because we were probably manually placed.
                        if (weAreKickingTeam)
                            resetToManualPlacementPoseOffense();
                        else
                            resetToManualPlacementPoseDefense();
                    }
                }
                else
                {
                    if (estimatorInfoInit.handleRefereeMistakes)
                    {
                        // To handle an accidental referee penalise, only reset if:
                        // - Robot was penalised for long enough (in case referee
                        //   accidentally penalises robot and immediately unpenalises)
                        // OR
                        // - Robot was picked up by referee during penalised
                        if (penalisedTimer.elapsed_ms() > PENALISED_TIMER_MIN_MSEC ||
                            pickedUpDuringPenalised)
                        {
                            if (newGameState == STATE_INITIAL) {
                                resetToInitialPose();
                            } else {
                                // Reset to unpenalised pose                                
                                resetToUnpenalisedPose();
                            }

                            // Restart unpenalisedTimer
                            unpenalisedTimer.restart();
                        }
                    }
                    else
                    {
                        if (newGameState == STATE_INITIAL) {
                            resetToInitialPose();
                        } else {
                            // Reset to unpenalised pose
                            resetToUnpenalisedPose();
                        }

                        // Restart unpenalisedTimer
                        unpenalisedTimer.restart();
                    }
                }
            }
        }
    }

    // Game State transition to STATE_INITIAL
    if (prevGameState != newGameState &&
        newGameState == STATE_INITIAL)
    {
        resetToInitialPose();
    }

    // Update prev variables
    prevCompetitionType = newCompetitionType;
    prevGameState = newGameState;
    prevGamePhase = newGamePhase;
    prevPenalty = newPenalty;
    prevPickedup = newPickedup;
}

void LocaliserTransitioner::resetToInitialPose()
{
    if (estimatorInfoInit.initialPoseType == "GAME")
    {
        resetToGameInitialPose();
    }
    else if (estimatorInfoInit.initialPoseType == "ONEVSONE")
    {
        resetToOneVsOneInitialPose();
    }
    else if (estimatorInfoInit.initialPoseType == "SPECIFIED")
    {
        resetToSpecifiedInitialPose();
    }
    else if (estimatorInfoInit.initialPoseType == "UNPENALISED")
    {
        resetToUnpenalisedPose();
    }
    else
    {
        std::cout << "(LocaliserTransitioner) initialPoseType: '"
                  << estimatorInfoInit.initialPoseType
                  << "' not recognised. Defaulting to GAME initial pose"
                  << std::endl;

        resetToGameInitialPose();
    }
}
