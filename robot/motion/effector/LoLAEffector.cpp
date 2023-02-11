#include "motion/effector/LoLAEffector.hpp"

#include <fstream>
#include <map>
#include <msgpack/v2/object_fwd.hpp>
#include <unistd.h>

#include "motion/MotionDefs.hpp"
#include "motion/LoLAData.hpp"
#include "motion/touch/LoLATouch.hpp"
#include "perception/kinematics/Kinematics.hpp"
#include "utils/Logger.hpp"

using namespace std;
using namespace LoLAData;

const int   SIT_MIDDLE_CYCLES                           = 167; // 2s
const float sit_angles_middle[Joints::NUMBER_OF_JOINTS] = {
      0.0,                // HeadYaw
      0.0,                // HeadPitch

      DEG2RAD(70),        // LShoulderPitch
      DEG2RAD(25),        // LShoulderRoll
      DEG2RAD(-92),       // LElbowYaw
      DEG2RAD(-61),       // LElbowRoll
      DEG2RAD(54),        // LWristYaw

      DEG2RAD(0),         // LHipYawPitch
      DEG2RAD(0),         // LHipRoll
      DEG2RAD(-50),       // LHipPitch
      DEG2RAD(125),       // LKneePitch
      DEG2RAD(-70),       // LAnklePitch
      0.0,                // LAnkleRoll

      DEG2RAD(0),         // RHipRoll
      DEG2RAD(-50),       // RHipPitch
      DEG2RAD(125),       // RKneePitch
      DEG2RAD(-70),       // RAnklePitch
      0.0,                // RAnkleRoll

      DEG2RAD(70),        // RShoulderPitch
      DEG2RAD(-25),       // RShoulderRoll
      DEG2RAD(92),        // RElbowYaw
      DEG2RAD(61),        // RElbowRoll
      DEG2RAD(-54),       // RWristYaw
      0.0,                // LHand
      0.0                 // RHand
};

const int   SIT_CYCLES                           = 83; // 1s
const float sit_angles[Joints::NUMBER_OF_JOINTS] = {
      0.0,                // HeadYaw
      0.0,                // HeadPitch

      DEG2RAD(72),        // LShoulderPitch
      DEG2RAD(2),         // LShoulderRoll
      DEG2RAD(-33),       // LElbowYaw
      DEG2RAD(-47),       // LElbowRoll
      DEG2RAD(55),        // LWristYaw

      DEG2RAD(0),         // LHipYawPitch
      DEG2RAD(0),         // LHipRoll
      DEG2RAD(-50),       // LHipPitch
      DEG2RAD(125),       // LKneePitch
      DEG2RAD(-70),       // LAnklePitch
      0.0,                // LAnkleRoll

      DEG2RAD(0),         // RHipRoll
      DEG2RAD(-50),       // RHipPitch
      DEG2RAD(125),       // RKneePitch
      DEG2RAD(-70),       // RAnklePitch
      0.0,                // RAnkleRoll

      DEG2RAD(72),        // RShoulderPitch
      DEG2RAD(-2),        // RShoulderRoll
      DEG2RAD(33),        // RElbowYaw
      DEG2RAD(47),        // RElbowRoll
      DEG2RAD(-55),       // RWristYaw
      0.0,                // LHand
      0.0                 // RHand
};

static void sit() {
   map<string, vector<float>> fMap;
   fMap["Position"]  = vector<float>(Joints::NUMBER_OF_JOINTS, 0.f);
   fMap["Stiffness"] = vector<float>(Joints::NUMBER_OF_JOINTS, 0.f);

   /////////////////////////////////////////////////////
   // Jayen's attempt to copy lost contact from libagent

   // crouch with arms forward, so they dont get caught on the body
   for (int sit_step = 0; sit_step < SIT_MIDDLE_CYCLES; ++sit_step) {
      float k = sit_step / (float) SIT_MIDDLE_CYCLES;
      k = (1.0f - cosf(k * M_PI)) / 2.0f;
      for (uint8_t i = Joints::HeadYaw; i < Joints::NUMBER_OF_JOINTS; ++i) {
         fMap["Position"][i]  = (1.0f - k) * sensors.joints.angles[i] + k * sit_angles_middle[i];
         fMap["Stiffness"][i] = 1.0f;
      }
      LoLAData::targetAngles = fMap["Position"];
      LoLAData::write(fMap);

      // can't do the read in LoLATouch as it's destroyed first
      // need to read, otherwise lola's write buffer fills up and it ignores us until we disconnect
      LoLAData::read();
   }

   // sit fully, place hands on knees to avoid robot falling forward and face planting
   for (int sit_step = 0; sit_step < SIT_CYCLES; ++sit_step) {
      float k = sit_step / (float) SIT_CYCLES;
      k = (1.0f - cosf(k * M_PI)) / 2.0f;
      for (uint8_t i = Joints::HeadYaw; i < Joints::NUMBER_OF_JOINTS; ++i) {
         fMap["Position"][i]  = (1.0f - k) * sit_angles_middle[i] + k * sit_angles[i];
         fMap["Stiffness"][i] = (1.0f - k) * 1.0f + k * 0.2f;
      }
      LoLAData::targetAngles = fMap["Position"];
      LoLAData::write(fMap);

      // can't do the read in LoLATouch as it's destroyed first
      // need to read, otherwise lola's write buffer fills up and it ignores us until we disconnect
      LoLAData::read();
   }

   // go completely limp
   for (uint8_t i = Joints::HeadYaw; i < Joints::NUMBER_OF_JOINTS; ++i) {
      fMap["Position"][i]  = sit_angles[i];
      fMap["Stiffness"][i] = 0.f;
   }
   LoLAData::targetAngles = fMap["Position"];
   LoLAData::write(fMap);
   
   // can't do the read in LoLATouch as it's destroyed first
   // need to read, otherwise lola's write buffer fills up and it ignores us until we disconnect
   LoLAData::read();
}

float getFlashBrightness() {
   timeval tmp = {};
   gettimeofday(&tmp, nullptr);
   double time       = tmp.tv_sec + tmp.tv_usec / 1000000.;
   float  brightness = (1 - cos(2 * M_PI * time / 0.4)) / 2;
   return brightness;
}

void write(map<string, vector<float>> &fMap) {
   static bool newlyLimp = false;
   if (!limp) {
      newlyLimp = true;
   }
   if (fMap.find("Position") != fMap.end()) {
      LoLAData::targetAngles = fMap["Position"];
   }
   if (fMap.find("Stiffness") != fMap.end()) {
      vector<float> &stiffness = fMap["Stiffness"];
      if (limp || standing) {
         fill(stiffness.begin(), stiffness.end(), 0.f);
         if (newlyLimp) {
            sit();
            newlyLimp = false;
         }

         float brightness = getFlashBrightness();
         if (standing) {
            fMap["Chest"] = vector<float>({brightness, brightness, 0});
         } else {
            fMap["Chest"] = vector<float>({0, brightness, 0});
         }
      }
      if (head_limp) {
         float brightness             = getFlashBrightness();
         if (fMap.find("LEye") != fMap.end()) {
            fMap["LEye"][LEDs::EyeGreen8] = brightness;
         }
         if (fMap.find("REye") != fMap.end()) {
            fMap["REye"][LEDs::EyeGreen8] = brightness;
         }
         stiffness[Joints::HeadPitch] = 0.f;
         stiffness[Joints::HeadYaw]   = 0.f;
      }
   }
   LoLAData::write(fMap);
}

/*-----------------------------------------------------------------------------
 * LoLA effector constructor
 *---------------------------------------------------------------------------*/
LoLAEffector::LoLAEffector(int team, int player_number) {
   llog(INFO) << "LoLAEffector constructed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * LoLA effector destructor
 *---------------------------------------------------------------------------*/
LoLAEffector::~LoLAEffector() {
   llog(INFO) << "LoLAEffector destroyed" << std::endl;

   if (!limp) {
      sit();
   }
}

static vector<float> ear(uint16_t ear) {
   vector<float> retval;
   retval.reserve(10);
   for (int i = 0; i < 10; ++i) {
      retval.push_back(ear & 1);
      ear >>= 1;
   }
   return retval;
}

void setEye(ActionCommand::rgb led, vector<float> &vec) {
   for (int i = 0; i < 8; i++) {
      vec[i]      = led.red;
      vec[i + 8]  = led.green;
      vec[i + 16] = led.blue;
   }
}

void setFoot(ActionCommand::rgb led, vector<float> &vec) {
   vec[0] = led.red;
   vec[1] = led.green;
   vec[2] = led.blue;
}

/**
 * Stiffen commands enable the robot to remotely stiffen on 'Ready' packet
 */
void doStiffen(ActionCommand::Stiffen stf) {
   if (stf == ActionCommand::STIFFEN) {
      limp     = false;
      standing = true;
   }
   // Otherwise it is ActionCommand::Stiffen::NONE
}

/*-----------------------------------------------------------------------------
 * LoLA effector - actuate the joints to the desired position
 *---------------------------------------------------------------------------*/
void LoLAEffector::actuate(JointValues joints, ActionCommand::LED leds, ActionCommand::Stiffen stiffen) {
   map<string, vector<float>> fMap;
   fMap["Position"]  = vector<float>(joints.angles, joints.angles + Joints::NUMBER_OF_JOINTS);
   fMap["Stiffness"] = vector<float>(joints.stiffnesses, joints.stiffnesses + Joints::NUMBER_OF_JOINTS);

   vector<float> eye(24), foot(3);

   setEye(leds.leftEye, eye);
   fMap["LEye"] = eye;
   setEye(leds.rightEye, eye);
   fMap["REye"] = eye;

   setFoot(leds.leftFoot, foot);
   fMap["LFoot"] = foot;
   setFoot(leds.rightFoot, foot);
   fMap["RFoot"] = foot;


   // apparently the chest LEDs take a brightness but we only have bools, so we convert them to 0.0 or 1.0
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
   float chest[] = {leds.chestButton.red, leds.chestButton.green, leds.chestButton.blue};
#pragma GCC diagnostic pop
   fMap["Chest"] = vector<float>(chest, chest + 3);

   fMap["LEar"] = ear(leds.leftEar);
   fMap["REar"] = ear(leds.rightEar);

   // copied from AgentEffector.  Not sure how it works since it seems to just set standing = false;
   static bool kill_standing = false;
   // effector needs to set standing to false if we got standing
   // we need to wait one cycle in case standing was set after AgentTouch is run
   standing                             = kill_standing;
   if (kill_standing) {
      kill_standing = false;
      standing      = false;
   } else {
      kill_standing = true;
   }

   doStiffen(stiffen);

   // Send off fMap
   write(fMap);

   map<string, vector<bool>> bMap;
   bool                      leftSonar  = true;
   bool                      rightSonar = true;
   std::vector<bool>         sonars     = {leftSonar, rightSonar};
   bMap["Sonar"] = sonars;

   // Send off bMap
   LoLAData::write(bMap);
}
