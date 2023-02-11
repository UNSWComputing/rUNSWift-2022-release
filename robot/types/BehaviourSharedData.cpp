//
// Created by jayen on 25/01/19.
//

#include "BehaviourSharedData.hpp"
#include "utils/PositioningDefs.hpp"

BehaviourSharedData::BehaviourSharedData() {
   #ifdef VALGRIND
      // there are holes in the struct causing valgrind to throw warnings when we transmit this struct as raw bytes
      bzero(this, sizeof(BehaviourSharedData));
   #endif
   secondsSinceLastKick = -1;
   role = POSITIONING_NONE;
   playingBall = false;
   needAssistance = false;
   isAssisting = false;
   isKickedOff = false;
   walkingToX = 0.f;
   walkingToY = 0.f;
   walkingToH = 0.f;
   kickNotification = false;
}
