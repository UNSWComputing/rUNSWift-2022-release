#ifndef VISION_INFO_IN_HPP
#define VISION_INFO_IN_HPP

#include "perception/vision/camera/CameraToRR.hpp"
#include "types/CameraSettings.hpp"
#include "perception/kinematics/Pose.hpp"
#include "types/ActionCommand.hpp"
#include "types/AbsCoord.hpp"

struct VisionInfoIn {
   uint8_t const* top_frame;
   uint8_t const* bot_frame;
   CameraSettings top_camera_settings;
   CameraSettings bot_camera_settings;
   CameraToRR cameraToRR;
   Pose pose;

   // NEED TO MAKE SURE THIS IS SET EACH FRAME AS IT WAS IN THE OLD SYSTEM.
   // This accounts for leaning that is not properly considered by
   // imageToRobotXY. imageToRobotXY either doesn't account for leaning of the
   // whole robot OR doesn't account for some kind of delay in gyro reading.
   float latestAngleX;

   AbsCoord robotPose; // pose of robot
};

#endif
