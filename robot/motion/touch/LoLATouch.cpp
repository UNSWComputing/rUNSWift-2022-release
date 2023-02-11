#include "motion/touch/LoLATouch.hpp"

#include <bitset>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <map>
#include <msgpack.hpp>
// not sure why i can't include these instead
//#include <msgpack/v1/object.hpp>
//#include <msgpack/v2/object_fwd.hpp>

#include "motion/LoLAData.hpp"
#include "motion/MotionDefs.hpp"
#include "utils/Logger.hpp"
#include "utils/speech.hpp"

#define MAX_CLICK_INTERVAL 15
// number of seconds it takes to sit.  can have decimal point.
#define KILL_SLEEP_SECONDS "4"
// number of cycles of consistent lost-stiffness before we declare it really lost
#define LOST_STIFFNESS_CYCLES 83

#define BUTTON_PRESS_LENGTH 80
#define BUTTON_TAP_LENGTH 10

using namespace std;
using namespace LoLAData;

LoLATouch::LoLATouch(int team, int player_number) {
   llog(INFO) << "LoLATouch constructed" << std::endl;
}

LoLATouch::~LoLATouch() {
   llog(INFO) << "LoLATouch destroyed" << std::endl;
}

static void load(const vector<msgpack::v2::object> &vector, float *array) {
   transform(vector.begin(), vector.end(), array, bind(&msgpack::v2::object::as<float>, placeholders::_1));
}


// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
/**
 * @param charge from 0 to 1
 */
static void doBattery(float charge, int status) {
   // Start complaining if battery < 30%
   // initialisation of static variables only happens once
   static float old_charge = charge;
   // if charge decreasing & <= 30%
   if (old_charge > charge && charge <= 0.3f) {
      SAY("battery " + boost::lexical_cast<string>(charge * 100) + " percent");
   }
   old_charge = charge;

   bitset<32>        b                  = status;
   static bitset<32> old_battery_status = b;
   // Bitset uses reverse-ordering, so the two lines below explain the indices for each bit
   // in the discharging and charging examples below it.
   //              v    (Index of first bit is 31, number from top line is 3, and then number from the bottom line is 1 - makes 31)
   //              3322222222221111111111
   //              10987654321098765432109876543210
   // discharging: 11000110111111111010100000000000
   // charging:    11000110111111101110100000000000
   if (old_battery_status[14] && !b[14]) {
      llog(INFO) << "Discharging";
   } else if (charge > 0.99f) {
      llog(INFO) << "Fully Charged";
   } else if (!old_battery_status[14] && b[14]) {
      llog(INFO) << "Charging";
   }
   old_battery_status = b;
}

// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
static void doTemps(JointValues joints) {
   static int t = 0;
   if (t % 100 == 0 &&
       joints.temperatures[t / 100] > 75 &&
       !limp && !standing &&
       joints.stiffnesses[t / 100] > 0) {
      SAY("OVERHEATING: " + Joints::fliteJointNames[t / 100]);
   }
   t = (t + 1) % (Joints::NUMBER_OF_JOINTS * 100);
}

static void stiffen(bool status) {
   head_limp = !status;
   limp = !status;
   standing = status;
}

// Jayen's not a big fan of making system calls (SAY, llog, ...) in this thread
// Consider moving this to perception or another low-priority thread
static inline ButtonPresses doButtons(
   bool chest, bool left, bool right,
   bool head_front, bool head_middle, bool head_rear,
   bool falling
) {

   static int head_buttons_count = 0, calibrate_buttons_count = 0, challenge_buttons_count = 0, any_head_count = 0;
   if (head_front && head_middle && head_rear) {
      head_buttons_count++;
      // Toggles limpness
      if (head_buttons_count == BUTTON_PRESS_LENGTH) {
         stiffen(limp);
      }
   } else if (head_buttons_count >= BUTTON_PRESS_LENGTH) {
      head_buttons_count = 2 * BUTTON_TAP_LENGTH;
   } else if (head_buttons_count > 0) {
      head_buttons_count--;
   }

   if (head_front && chest) {
      calibrate_buttons_count++;
      if (calibrate_buttons_count == BUTTON_PRESS_LENGTH) {
         SAY("Auto calibrating");
         // insert auto-calibration code
      }
   } else if (calibrate_buttons_count >= BUTTON_PRESS_LENGTH) {
      calibrate_buttons_count = 2 * BUTTON_TAP_LENGTH;
   } else if (calibrate_buttons_count > 0) {
      calibrate_buttons_count--;
   }

   if (head_rear && chest) {
      challenge_buttons_count++;
      if (challenge_buttons_count == BUTTON_PRESS_LENGTH) {
         SAY("Auto calibration challenge mode");
         // insert auto-callibration challenge
      }
   } else if (challenge_buttons_count >= BUTTON_PRESS_LENGTH) {
      challenge_buttons_count = 2 * BUTTON_TAP_LENGTH;
   } else if (challenge_buttons_count > 0) {
      challenge_buttons_count--;
   }

   if (chest && !calibrate_buttons_count && !challenge_buttons_count) {
      stiffen(true);
   }

   if ((head_front || head_middle || head_rear) && !chest && !head_buttons_count) {
      any_head_count++;
      if (any_head_count == BUTTON_TAP_LENGTH)  {
         SAY("head button tapped, continuing");
         // insert continuing
      }
   } else {
      any_head_count = 0;
   }

   static int    chest_up = 0, chest_presses = 0, chest_down = 0;
   ButtonPresses buttons;

   // deal with button presses
   if (chest_up > MAX_CLICK_INTERVAL && chest_presses) {
      buttons = chest_presses;
      chest_presses = 0;
      if (buttons.remove(ButtonPresses::DOUBLE_CHEST_TAP)) {
         if (left || right) { // if foot bumper
            // falling sometimes triggers button presses
            if (!falling) {
               head_limp = !head_limp;
               SAY(std::string("head ") + (head_limp ? "limp" : "stiff"));
            }
         } else {
            if (limp) {
               limp     = false;
               standing = true;
            } else {
               // fallling sometimes triggers button presses
               if (!falling) {
                  limp     = true;
                  standing = false;
               }
            }
            SAY(std::string("body ") + (limp ? "limp" : "stiff"));
         }
      } else if (buttons.remove(ButtonPresses::TRIPLE_CHEST_TAP)) {
         if (left || right) {
            SAY("Restarting now key");  // yay transliteration
            // Runlevel a is set up to run nao restart on demand.
            // See man pages for inittab, init for details
            system("sudo /sbin/init a");
            // should not get here
         } else {
            // falling sometimes triggers button presses
            if(!falling) {
               SAY("killed runswift");
   //            kill -9 me in case i don't die
               system("/usr/bin/killall runswift & (sleep " KILL_SLEEP_SECONDS "; /usr/bin/pkill -9 runswift) &");
            }
         }
      } else if (buttons.remove(ButtonPresses::QUADRUPLE_CHEST_TAP)) {
         if(!falling) {
            SAY("Restart wifi");
            system("sudo /home/nao/bin/changeField.py");
            // changeField.py already does this, but libagent did this so we do it
            system("sudo /etc/init.d/runswiftwireless.sh restart");
         }
      }
   }

   // special shutdown handler
   // we set chest_down to int_min so only one shutdown will happen
   if (chest_down > 300) {  // 3 seconds
      SAY("Shutting down");
      system("sudo /sbin/halt");
      limp       = true;
      chest_down = std::numeric_limits<int>::min();
   }

   // update counters
   if (chest && !calibrate_buttons_count && !challenge_buttons_count) {
      if (chest_down >= 0) {
         chest_down++;
      }
      chest_up = 0;
   } else {
      chest_up++;
      if (chest_down > 0) {
         chest_presses++;
         chest_down = 0;
      }
   }
   return buttons;
}

void LoLATouch::loadCache() {
   const map<string, vector<msgpack::v2::object> > sensorMap = read();
   if (sensorMap.size() == 0)
      return;

   // Cout the whole map
   // for(map<string, vector<msgpack::v2::object>::iterator it = sensorMap.begin(); it != sensorMap.end(); ++it)
   // {
   //    std::cout << it->first << " " << it->second << "\n";
   // }

   load(sensorMap.at("Position"), sensors.joints.angles);
   load(sensorMap.at("Stiffness"), sensors.joints.stiffnesses);
   load(sensorMap.at("Temperature"), sensors.joints.temperatures);
   load(sensorMap.at("Current"), sensors.joints.currents);

   // = {} is redudunant as static arrays are zero-initialised anyway
   static uint8_t lostStiffnessCounters[Joints::NUMBER_OF_JOINTS] = {};
   for (int       jointIndex                                      = 0;
        jointIndex < Joints::NUMBER_OF_JOINTS;
        ++jointIndex) {

      // LWristYaw and RWristYaw can throw false positives joint issues, but ignore them
      if (jointIndex != Joints::LWristYaw &&
          jointIndex != Joints::RWristYaw &&
          sensors.joints.currents[jointIndex] == 0.f &&
          sensors.joints.stiffnesses[jointIndex] > 0.f &&
          abs(sensors.joints.angles[jointIndex] - LoLAData::targetAngles[jointIndex]) > DEG2RAD(2))
      {
         ++lostStiffnessCounters[jointIndex];
         if (lostStiffnessCounters[jointIndex] > LOST_STIFFNESS_CYCLES) {
            SAY("cannot move " + Joints::fliteJointNames[jointIndex]);
         }
      } else {
         lostStiffnessCounters[jointIndex] = 0;
      }
   }

   load(sensorMap.at("Battery"), sensors.sensors + Sensors::SensorCodesEnum::Battery_Charge);
   load(sensorMap.at("Accelerometer"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_AccelerometerX);
   load(sensorMap.at("Gyroscope"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_GyroscopeX);
   load(sensorMap.at("Angles"), sensors.sensors + Sensors::SensorCodesEnum::InertialSensor_AngleX);
   load(sensorMap.at("Sonar"), sensors.sensors + Sensors::SensorCodesEnum::SonarLeft);
   load(sensorMap.at("FSR"), sensors.sensors + Sensors::LFoot_FSR_FrontLeft);
   load(sensorMap.at("Touch"), sensors.sensors + Sensors::ChestBoard_Button);

   // Cout all sensors
   // for (unsigned i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
   // {
   //    std::cout << Sensors::sensorNames[i] << ": " << sensors.sensors[i] << std::endl;
   // }

   float *status = &sensors.sensors[Sensors::Battery_Status];
   doBattery(sensors.sensors[Sensors::Battery_Charge],
         // treat the float like an int, and pray it works
             *(int *) status);
   doTemps(sensors.joints);
}

SensorValues LoLATouch::getSensors(Kinematics &kinematics) {
   return sensors;
}

bool LoLATouch::getStanding() {
   return standing;
}

ButtonPresses LoLATouch::getButtons() {

   bool falling = (sensors.sensors[Sensors::InertialSensor_AngleY] > 0.60) ||
                  (sensors.sensors[Sensors::InertialSensor_AngleY] < -0.60);

   return doButtons(static_cast<bool>(sensors.sensors[Sensors::ChestBoard_Button]),
                    static_cast<bool>(sensors.sensors[Sensors::LFoot_Bumper_Left]) ||
                    static_cast<bool>(sensors.sensors[Sensors::LFoot_Bumper_Right]),
                    static_cast<bool>(sensors.sensors[Sensors::RFoot_Bumper_Left]) ||
                    static_cast<bool>(sensors.sensors[Sensors::RFoot_Bumper_Right]),
                    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Front]),
                    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Middle]),
                    static_cast<bool>(sensors.sensors[Sensors::Head_Touch_Rear]),
                    falling);
}

bool LoLATouch::getLimp() {
   return limp;
}
