#ifndef ESTIMATOR_INFO_INIT_HPP
#define ESTIMATOR_INFO_INIT_HPP

#include <iostream>
#include "types/AbsCoord.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"
#ifndef Q_MOC_RUN
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/string.hpp>
#endif

class EstimatorInfoInit
{
  public:
    EstimatorInfoInit(
        const int &playerNumber,
        const int &teamNumber,
        const std::string &initialPoseType,
        const AbsCoord &specifiedInitialPose,
        const std::string &skill,
        const uint8_t &competitionType,
        const uint8_t &state,
        const uint8_t &gamePhase,
        const uint8_t &setPlay,
        const bool &handleRefereeMistakes)
        : playerNumber(playerNumber)
        , teamNumber(teamNumber)
        , initialPoseType(initialPoseType)
        , specifiedInitialPose(specifiedInitialPose)
        , skill(skill)
        , competitionType(competitionType)
        , state(state)
        , gamePhase(gamePhase)
        , setPlay(setPlay)
        , handleRefereeMistakes(handleRefereeMistakes)
        {};

    EstimatorInfoInit(){};

    int playerNumber;
    int teamNumber;
    std::string initialPoseType;
    AbsCoord specifiedInitialPose;
    std::string skill;
    uint8_t competitionType;
    uint8_t state;
    uint8_t gamePhase;
    uint8_t setPlay;
    bool handleRefereeMistakes;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version)
    {
        ar & playerNumber;
        ar & teamNumber;
        ar & initialPoseType;
        ar & specifiedInitialPose;
        ar & skill;
        ar & competitionType;
        ar & state;
        ar & gamePhase;
        ar & setPlay;
        ar & handleRefereeMistakes;
    }
};

#endif // ESTIMATOR_INFO_INIT_HPP
