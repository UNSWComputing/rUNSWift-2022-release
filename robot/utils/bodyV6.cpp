#include "body.hpp"

#include "motion/MotionDefs.hpp"

// Calculate Rad/MOTION_DT, so how much the joint can move in frame
#define CALCULATE_RAD_P_MOTION_DT(rpm, ratio) ((rpm * (2 * M_PI) / 60) * MOTION_DT) / ratio

namespace V6Joints {
    namespace Radians {
        // Speed of motors in RPM
        const float MOTOR1_NO_LOAD_SPEED_RPM = 8700;
        const float MOTOR2_NO_LOAD_SPEED_RPM = 8400;
        const float MOTOR3_NO_LOAD_SPEED_RPM = 10700;
        const float MOTOR4_NO_LOAD_SPEED_RPM = 11400;
        const float MOTOR5_NO_LOAD_SPEED_RPM = 8700;

        // Gear Reduction Ratios (no units)
        const float MOTOR1_REDUCTIONA_RATIO = 201.3;
        const float MOTOR2_REDUCTIONA_RATIO = 50.61;
        const float MOTOR3_REDUCTIONA_RATIO = 150.27;
        const float MOTOR4_REDUCTIONA_RATIO = 150.27;
        const float MOTOR5_REDUCTIONA_RATIO = 130.85;
        const float MOTOR2_REDUCTIONB_RATIO = 36.24;
        const float MOTOR3_REDUCTIONB_RATIO = 173.22;

        // Maximum rotation speed (rad per MOTION_DT)
        const float HeadPitchSpeed       = CALCULATE_RAD_P_MOTION_DT(MOTOR3_NO_LOAD_SPEED_RPM, MOTOR3_REDUCTIONB_RATIO);
        const float HeadYawSpeed         = CALCULATE_RAD_P_MOTION_DT(MOTOR3_NO_LOAD_SPEED_RPM, MOTOR3_REDUCTIONA_RATIO);
        const float ShoulderPitchSpeed   = CALCULATE_RAD_P_MOTION_DT(MOTOR4_NO_LOAD_SPEED_RPM, MOTOR4_REDUCTIONA_RATIO);
        const float ShoulderRollSpeed    = CALCULATE_RAD_P_MOTION_DT(MOTOR3_NO_LOAD_SPEED_RPM, MOTOR3_REDUCTIONB_RATIO);
        const float ElbowYawSpeed        = CALCULATE_RAD_P_MOTION_DT(MOTOR3_NO_LOAD_SPEED_RPM, MOTOR3_REDUCTIONA_RATIO);
        const float ElbowRollSpeed       = CALCULATE_RAD_P_MOTION_DT(MOTOR3_NO_LOAD_SPEED_RPM, MOTOR3_REDUCTIONB_RATIO);
        const float WristYawSpeed        = CALCULATE_RAD_P_MOTION_DT(MOTOR2_NO_LOAD_SPEED_RPM, MOTOR2_REDUCTIONA_RATIO);
        const float HandSpeed            = CALCULATE_RAD_P_MOTION_DT(MOTOR2_NO_LOAD_SPEED_RPM, MOTOR2_REDUCTIONB_RATIO);
        const float HipYawPitchSpeed     = CALCULATE_RAD_P_MOTION_DT(MOTOR1_NO_LOAD_SPEED_RPM, MOTOR1_REDUCTIONA_RATIO);
        const float HipRollSpeed         = CALCULATE_RAD_P_MOTION_DT(MOTOR1_NO_LOAD_SPEED_RPM, MOTOR1_REDUCTIONA_RATIO);
        const float HipPitchSpeed        = CALCULATE_RAD_P_MOTION_DT(MOTOR5_NO_LOAD_SPEED_RPM, MOTOR5_REDUCTIONA_RATIO);
        const float KneePitchSpeed       = CALCULATE_RAD_P_MOTION_DT(MOTOR5_NO_LOAD_SPEED_RPM, MOTOR5_REDUCTIONA_RATIO);
        const float AnklePitchSpeed      = CALCULATE_RAD_P_MOTION_DT(MOTOR5_NO_LOAD_SPEED_RPM, MOTOR5_REDUCTIONA_RATIO);
        const float AnkleRollSpeed       = CALCULATE_RAD_P_MOTION_DT(MOTOR1_NO_LOAD_SPEED_RPM, MOTOR1_REDUCTIONA_RATIO);

        const float MaxSpeed[NUMBER_OF_JOINTS] = {
                HeadYawSpeed,
                HeadPitchSpeed,
                ShoulderPitchSpeed,  // Left arm
                ShoulderRollSpeed,
                ElbowYawSpeed,
                ElbowRollSpeed,
                WristYawSpeed,
                HipYawPitchSpeed,    // Left leg
                HipRollSpeed,
                HipPitchSpeed,
                KneePitchSpeed,
                AnklePitchSpeed,
                AnkleRollSpeed,
                HipRollSpeed,        // Right leg
                HipPitchSpeed,
                KneePitchSpeed,
                AnklePitchSpeed,
                AnkleRollSpeed,
                ShoulderPitchSpeed,  // Right arm
                ShoulderRollSpeed,
                ElbowYawSpeed,
                ElbowRollSpeed,
                WristYawSpeed,
                HandSpeed,
                HandSpeed
        };
    }

   void outputValues(std::ostream &os, const float values[NUMBER_OF_JOINTS]) {
      for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
         os << jointNames[j] << ":\t" << values[j] << '\n';
      }
   }
}
