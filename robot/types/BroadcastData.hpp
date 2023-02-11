#pragma once

#include "types/AbsCoord.hpp"
#include "types/RRCoord.hpp"
#include "types/ActionCommand.hpp"
#include "types/BehaviourSharedData.hpp"
#include "types/SharedStateEstimationBundle.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

class BroadcastData {
  public:
    BroadcastData()
        : playerNum(0),
          ballPosAbs(),
          ballPosRR(),
          sharedStateEstimationBundle(),
          behaviourSharedData(),
          acB(ActionCommand::Body::LIMP),
          uptime(0.0),
          gameState(STATE_INITIAL)
    {
        robotPos[0] = 0.f;
        robotPos[1] = 0.f;
        robotPos[2] = 0.f;
    }

    BroadcastData(const BroadcastData& bd)
        : playerNum(bd.playerNum),
          ballPosAbs(bd.ballPosAbs),
          ballPosRR(bd.ballPosRR),
          sharedStateEstimationBundle(bd.sharedStateEstimationBundle),
          behaviourSharedData(bd.behaviourSharedData),
          acB(bd.acB),
          uptime(bd.uptime),
          gameState(bd.gameState)
    {
        robotPos[0] = bd.robotPos[0];
        robotPos[1] = bd.robotPos[1];
        robotPos[2] = bd.robotPos[2];
    }

    BroadcastData(const int &playerNum,
        const AbsCoord &RobotPos,
        const AbsCoord &ballPosAbs,
        const RRCoord &ballPosRR,
        const SharedStateEstimationBundle &sharedStateEstimationBundle,
        const BehaviourSharedData &behaviourSharedData,
        const ActionCommand::Body::ActionType &acB,
        const float &uptime,
        const uint8_t &gameState);

    bool sanityCheck();

    int playerNum;

    float robotPos[3];
    AbsCoord ballPosAbs;
    RRCoord ballPosRR;

    SharedStateEstimationBundle sharedStateEstimationBundle;

    // Data set by the Python behaviours that is shared with other robots.
    BehaviourSharedData behaviourSharedData;

    ActionCommand::Body::ActionType acB;
    float uptime;
    uint8_t gameState;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        ar & playerNum;
        if (file_version == 0) {
            int team;
            AbsCoord robotPos;
            ar & team;
            ar & robotPos;
        } else {
            ar & robotPos;
        }
        ar & ballPosAbs;
        ar & ballPosRR;
        if (file_version < 2) {
            uint32_t tmpLostCount;
            ar & tmpLostCount;
        }
        ar & sharedStateEstimationBundle;
        ar & behaviourSharedData;
        ar & acB;
        ar & uptime;
        ar & gameState;
    }
};
BOOST_CLASS_VERSION(BroadcastData, 2);
