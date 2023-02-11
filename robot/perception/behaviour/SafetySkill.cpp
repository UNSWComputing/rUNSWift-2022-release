#include "perception/behaviour/SafetySkill.hpp"
#include "utils/basic_maths.hpp"
#include "utils/Logger.hpp"
#include "blackboard/Blackboard.hpp"

#define MIN_STANDING_WEIGHT 0.55f
// FALLING_ANG must be less than FALLEN_ANG
#define FALLEN_ANG 50
#define FUTURE_TIME 0.2
#define FALLING_CONFIRM_FRAMES 3
#define FALLING_ANG_FORWARD 40
#define FALLING_ANG_BACK 35
#define FALLING_ANG_Y 45


SafetySkill::SafetySkill() {
   filtered_fsr_sum     = 5.0;  // assume we start standing
   blink                = 0;
   cor_angular_velocity = 0;
   sag_angular_velocity = 0;
   prev_angles[0] = 0;
   prev_angles[1] = 0;
   fallingCounter = 0;
   llog(INFO) << "SafetySkill constructed" << std::endl;
}

SafetySkill::~SafetySkill() {
   llog(INFO) << "SafetySkill destroyed" << std::endl;
}

void SafetySkill::readOptions(const boost::program_options::variables_map& config)
{
    simulation = config.count("simulation");
}

float calcFutureAngle(float angle, float velocity) {
   return angle + velocity * FUTURE_TIME;
}

BehaviourRequest SafetySkill::wrapRequest(const BehaviourRequest &request, const SensorValues &s, bool ukemiEnabled, Blackboard *bb) {
   BehaviourRequest r = request;
   useGetups = bb->behaviour.useGetups;

   // If we're in the simulator, don't do ukemi or getups because it goes nuts
   if (simulation)
      return r;

   if (r.actions.body.actionType == ActionCommand::Body::MOTION_CALIBRATE) {
      return r;
   }

   float fsr_sum = s.sensors[Sensors::LFoot_FSR_FrontLeft]
                   + s.sensors[Sensors::LFoot_FSR_FrontRight]
                   + s.sensors[Sensors::LFoot_FSR_RearLeft]
                   + s.sensors[Sensors::LFoot_FSR_RearRight]
                   + s.sensors[Sensors::RFoot_FSR_FrontLeft]
                   + s.sensors[Sensors::RFoot_FSR_FrontRight]
                   + s.sensors[Sensors::RFoot_FSR_RearLeft]
                   + s.sensors[Sensors::RFoot_FSR_RearRight];

   if (!std::isnan(fsr_sum)) {
      filtered_fsr_sum = filtered_fsr_sum + 0.2 * (fsr_sum - filtered_fsr_sum);
   }

   blink = !blink;
   float ang[2] = {RAD2DEG(s.sensors[Sensors::InertialSensor_AngleX]),
                   RAD2DEG(s.sensors[Sensors::InertialSensor_AngleY])};

   float futureAng[2] = {
      RAD2DEG(calcFutureAngle(
         s.sensors[Sensors::InertialSensor_AngleX],
         s.sensors[Sensors::InertialSensor_GyroscopeX]
      )),
      RAD2DEG(calcFutureAngle(
         s.sensors[Sensors::InertialSensor_AngleY],
         s.sensors[Sensors::InertialSensor_GyroscopeY]
      ))
   };

   // choose a safety skill
   if (ang[1] < -FALLEN_ANG) {
      fallingCounter = 0;
      // if fallen back more than FALLEN_ANG
      r.actions.leds.leftEye  = ActionCommand::rgb(blink, 0);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
      r.actions.body          = (useGetups) ? ActionCommand::Body::GETUP_BACK : ActionCommand::Body::LIMP;
   } else if (ang[1] > FALLEN_ANG) {
      fallingCounter = 0;
      // if fallen back more than FALLEN_ANG
      r.actions.leds.leftEye  = ActionCommand::rgb(blink, 0);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
      r.actions.body          = (useGetups) ? ActionCommand::Body::GETUP_FRONT : ActionCommand::Body::LIMP;
   } else if (ang[0] > FALLEN_ANG || ang[0] < -FALLEN_ANG) {
      fallingCounter = 0;
      // if fallen on either side
      r.actions.leds.leftEye  = ActionCommand::rgb(blink, 0);
      r.actions.leds.rightEye = ActionCommand::rgb(blink, 0);
      r.actions.body          = ActionCommand::Body::TIP_OVER;
   } else if (futureAng[1] > FALLING_ANG_FORWARD && ukemiEnabled) {
      fallingCounter++;
      if (fallingCounter >= FALLING_CONFIRM_FRAMES) {
         r.actions.leds.leftEye  = ActionCommand::rgb(blink, blink);
         r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
         r.actions.body          = ActionCommand::Body::UKEMI_FRONT;
      }
   } else if (futureAng[1] < -FALLING_ANG_BACK) {
      fallingCounter++;
      if (fallingCounter >= FALLING_CONFIRM_FRAMES && ukemiEnabled) {
         r.actions.leds.leftEye  = ActionCommand::rgb(blink, blink);
         r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
         r.actions.body          = ActionCommand::Body::UKEMI_BACK;
      }
   } else if (ABS(futureAng[0]) > FALLING_ANG_Y) {
      fallingCounter++;
      // still falling!
      if (fallingCounter >= FALLING_CONFIRM_FRAMES) {
         r.actions.leds.leftEye  = ActionCommand::rgb(blink, blink);
         r.actions.leds.rightEye = ActionCommand::rgb(blink, blink);
         r.actions.body          = ActionCommand::Body::LIMP;
      }
   } else if (filtered_fsr_sum < MIN_STANDING_WEIGHT
               && r.actions.body.actionType != ActionCommand::Body::GOALIE_STAND) {
      fallingCounter = 0;
      // upright but not on the ground
      r.actions.leds.leftEye  = ActionCommand::rgb(0, blink);
      r.actions.leds.rightEye = ActionCommand::rgb(0, blink);
      r.actions.body          = ActionCommand::Body::REF_PICKUP;
   } else {
      fallingCounter = 0;
   }

   return r;
}
