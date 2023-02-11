#ifndef KICK_CALIBRATION_SKILL_HPP
#define KICK_CALIBRATION_SKILL_HPP


#include "types/BehaviourRequest.hpp"
#include "types/CleverNaoInfo.hpp"

class AbsCoord;
class BallInfo;

class KickCalibrationSkill {
    public:
        KickCalibrationSkill();
        BehaviourRequest execute(const AbsCoord &ballRel,
            const std::vector<BallInfo> &detectedBalls,
            ActionCommand::Body::Foot foot,
            CleverNaoInfoBehaviour& cleverNaoInfo,
            CleverNaoInfoMotion& cleverNaoInfoMotion,
            const std::string& bodyName);
    private:
        long ballLostCount;
        //bool randomLeanGenerated;
};


#endif //KICK_CALIBRATION_SKILL_HPP
