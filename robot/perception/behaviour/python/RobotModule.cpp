#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <vector>

#include <blackboard/Blackboard.hpp>

#include <gamecontroller/RoboCupGameControlData.hpp>

#include <types/AbsCoord.hpp>
#include <types/ActionCommand.hpp>
#include <types/BallInfo.hpp>
#include <types/BBox.hpp>
#include <types/BehaviourRequest.hpp>
#include <types/BehaviourSharedData.hpp>
#include <types/JointValues.hpp>
#include <types/RobotVisionInfo.hpp>
#include <types/RobotObstacle.hpp>
#include <types/RRCoord.hpp>
#include <types/XYZ_Coord.hpp>
#include <types/SensorValues.hpp>

#include <perception/kinematics/Parameters.hpp>

#include <utils/body.hpp>
#include <utils/speech.hpp>
#include <utils/SPLDefs.hpp>
#include <utils/PositioningDefs.hpp>

#include "RegisterConverters.hpp"

using namespace boost::python;

typedef float (RRCoord ::* rr_get_v_const_type)() const;
typedef float (AbsCoord::*abs_get_v_const_type)() const;

void py_say(const std::string &text) { SAY(text); }

BOOST_PYTHON_MODULE(robot)
{
   register_python_converters();

   #include "wrappers/AbsCoord_wrap.cpp"
   #include "wrappers/ActionCommand_wrap.cpp"
   #include "wrappers/BallInfo_wrap.cpp"
   #include "wrappers/BBox_wrap.cpp"
   #include "wrappers/BehaviourBlackboard_wrap.cpp"
   #include "wrappers/BehaviourRequest_wrap.cpp"
   #include "wrappers/BehaviourSharedData_wrap.cpp"
   #include "wrappers/KinematicsParameters_wrap.cpp"
   #include "wrappers/Blackboard_wrap.cpp"
   #include "wrappers/body_wrap.cpp"
   #include "wrappers/BroadcastData_wrap.cpp"
   #include "wrappers/GameController_wrap.cpp"
   #include "wrappers/GameControllerBlackboard_wrap.cpp"
   #include "wrappers/JointValues_wrap.cpp"
   #include "wrappers/StateEstimationBlackboard_wrap.cpp"
   #include "wrappers/MotionBlackboard_wrap.cpp"
   #include "wrappers/ReceiverBlackboard_wrap.cpp"
   #include "wrappers/RobotVisionInfo_wrap.cpp"
   #include "wrappers/RobotObstacle_wrap.cpp"
   #include "wrappers/RRCoord_wrap.cpp"
   #include "wrappers/XYZ_Coord_wrap.cpp"
   #include "wrappers/say_wrap.cpp"
   #include "wrappers/SensorValues_wrap.cpp"
   #include "wrappers/SPLDefs_wrap.cpp"
   #include "wrappers/SPLStandardMessage_wrap.cpp"
   #include "wrappers/VisionDefinitions_wrap.cpp"
   #include "wrappers/vector_wrap.cpp"
   #include "wrappers/VisionBlackboard_wrap.cpp"
   #include "wrappers/KinematicsBlackboard_wrap.cpp"
   #include "wrappers/BehaviourDebugInfo_wrap.cpp"
   #include "wrappers/PositioningDefs_wrap.cpp"
   #include "wrappers/SharedStateEstimationBundle_wrap.cpp"
}
