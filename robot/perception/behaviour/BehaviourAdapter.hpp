#pragma once

#include <Python.h>
#include "perception/behaviour/SafetySkill.hpp"
#include "blackboard/Adapter.hpp"
#ifndef Q_MOC_RUN
#include "boost/shared_ptr.hpp"
#endif
#include "perception/behaviour/python/PythonSkill.hpp"
#include "IMUCalibrationSkill.hpp"
#include "KickCalibrationSkill.hpp"
#include "utils/clevernao/CleverNao.hpp"

class Skill;

/* Adapter that allows Behaviour to communicate with the Blackboard */
class BehaviourAdapter : Adapter {
    public:
        /* Constructor */
        BehaviourAdapter(Blackboard *bb);
        /* Destructor */
        ~BehaviourAdapter();
        /* One cycle of this thread */
        void tick();
    private:
        PythonSkill *pythonSkill;
        CleverNao *cleverNao;
        boost::shared_ptr<Skill> headSkill;
        IMUCalibrationSkill imuCalibrationSkill;
        KickCalibrationSkill kickCalibrationSkill;
        SafetySkill safetySkill;
        bool remoteControlActive;
        bool runningIMUCalibrationSkill;
        bool runningKickCalibrationSkill;
        ActionCommand::Body::Foot kickCalibrationFoot;
        void readOptions(const boost::program_options::variables_map& config);
};
