#include "types/BroadcastData.hpp"
#include "utils/SPLDefs.hpp"


BroadcastData::BroadcastData(const int &playerNum,
                             const AbsCoord &RobotPos,
                             const AbsCoord &ballPosAbs,
                             const RRCoord &ballPosRR,
                             const SharedStateEstimationBundle &sharedStateEstimationBundle,
                             const BehaviourSharedData &behaviourSharedData,
                             const ActionCommand::Body::ActionType &acB,
                             const float &uptime,
                             const uint8_t &gameState) {
   #ifdef VALGRIND
      // there is padding in the struct causing valgrind to throw warnings when we transmit this struct as raw bytes
      bzero(this, sizeof(BroadcastData));
   #endif
   this->playerNum = playerNum;
   this->ballPosAbs = ballPosAbs;
   this->ballPosRR = ballPosRR;
   this->sharedStateEstimationBundle = sharedStateEstimationBundle;
   this->behaviourSharedData = behaviourSharedData;
   this->acB = acB;
   this->uptime = uptime;
   this->gameState = gameState;
   robotPos[0] = RobotPos.x();
   robotPos[1] = RobotPos.y();
   robotPos[2] = RobotPos.theta();
}

bool BroadcastData::sanityCheck()
{
    if (playerNum < 1)
    {
        std::cout << "received playerNum less than 1" << std::endl;
        return false;
    }

    if (playerNum > ROBOTS_PER_TEAM)
    {
        std::cout << "received playerNum greater than ROBOTS_PER_TEAM" << std::endl;
        return false;
    }

    if (abs(ballPosAbs.x()) > FULL_FIELD_LENGTH / 2 ||
        abs(ballPosAbs.y()) > FULL_FIELD_WIDTH / 2)
    {
        std::cout << "received ballPosAbs off the field" << std::endl;
        return false;
    }

    if (pow(ballPosRR.distance(), 2) > pow(FULL_FIELD_LENGTH, 2) + pow(FULL_FIELD_WIDTH, 2)){
        std::cout << "received ballPosRR with distance longer than diagonal of the field" << std::endl;
        return false;
    }

    if (abs(ballPosRR.heading()) > 2*M_PI)
    {
        std::cout << "received ballPosRR heading greater or less than 2*pi" << std::endl;
        return false;
    }

    if (abs(ballPosRR.orientation()) > 2*M_PI)
    {
        std::cout << "received ballPosRR orientation greater or less than 2*pi" << std::endl;
        return false;
    }

    if (!sharedStateEstimationBundle.sanityCheck())
    {
        std::cout << "received sharedStateEstimationBundle that didn't pass sanity check" << std::endl;
        return false;
    }

    if (!behaviourSharedData.sanityCheck())
    {
        return false;
    }

    return true;
}
