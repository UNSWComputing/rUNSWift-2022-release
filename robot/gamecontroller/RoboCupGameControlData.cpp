#include "gamecontroller/RoboCupGameControlData.hpp"


/** 
 * NOTE: Remember to change these strings if any of the penalty / state / phase names in 
 * RoboCupGameControlData.hpp change
 */

const char *gameControllerGamePhaseNames[] = {
   " ",  // Don't care about saying normal
   "shootout ",
   "overtime ",
   "timeout "
   // Note extra space at end so we don't merge words
};

const char *gameControllerStateNames[] = {
   "initial",
   "ready",
   "set",
   "playing",
   "finished",
};

const char *gameControllerPenaltyNames[] = {
   "none",
   "illegal ball contact",
   "player pushing",
   "motion in set",
   "inactive player",
   "illegal position",
   "leaving the field",
   "request for pickup",
   "local game stuck",
   "illegal position in set",
   "invalid",
   "invalid",
   "invalid",
   "invalid",
   "substitute penalty",
   "manual penalty",
};
