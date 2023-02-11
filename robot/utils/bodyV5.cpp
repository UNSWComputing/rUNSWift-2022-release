#include "body.hpp"

namespace V5Joints {
  namespace Radians {
        // Maximum rotation speed (Radians) per cycle (10ms)
        const float MOTOR1_REDUCTIONA_RAD = DEG2RAD(2.23);
        const float MOTOR1_REDUCTIONB_RAD = DEG2RAD(3.43);
        const float MOTOR2_REDUCTIONA_RAD = DEG2RAD(8.96);
        const float MOTOR2_REDUCTIONB_RAD = DEG2RAD(12.52);
        const float MOTOR3_REDUCTIONA_RAD = DEG2RAD(3.85);
        const float MOTOR3_REDUCTIONB_RAD = DEG2RAD(3.34);

        const float HeadPitchSpeed       = MOTOR3_REDUCTIONB_RAD;
        const float HeadYawSpeed         = MOTOR3_REDUCTIONA_RAD;
        const float ShoulderPitchSpeed   = MOTOR3_REDUCTIONA_RAD;
        const float ShoulderRollSpeed    = MOTOR3_REDUCTIONB_RAD;
        const float ElbowYawSpeed        = MOTOR3_REDUCTIONA_RAD;
        const float ElbowRollSpeed       = MOTOR3_REDUCTIONB_RAD;
        const float WristYawSpeed        = MOTOR2_REDUCTIONA_RAD;
        const float HandSpeed            = MOTOR2_REDUCTIONB_RAD;
        const float HipYawPitchSpeed     = MOTOR1_REDUCTIONA_RAD;
        const float HipRollSpeed         = MOTOR1_REDUCTIONA_RAD;
        const float HipPitchSpeed        = MOTOR1_REDUCTIONB_RAD;
        const float KneePitchSpeed       = MOTOR1_REDUCTIONB_RAD;
        const float AnklePitchSpeed      = MOTOR1_REDUCTIONB_RAD;
        const float AnkleRollSpeed       = MOTOR1_REDUCTIONA_RAD;

        const float MaxSpeed[NUMBER_OF_JOINTS] = {
                HeadYawSpeed,
                HeadPitchSpeed,
                ShoulderPitchSpeed,  // Left arm
                ShoulderRollSpeed,
                ElbowYawSpeed,
                ElbowRollSpeed,
                WristYawSpeed,
                HandSpeed,
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
                HandSpeed
        };
    }

   void outputValues(std::ostream &os, const float values[NUMBER_OF_JOINTS]) {
      for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
         os << jointNames[j] << ":\t" << values[j] << '\n';
      }
   }
}
