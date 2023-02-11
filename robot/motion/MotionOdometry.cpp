#include "motion/MotionOdometry.hpp"

#include "utils/body.hpp"
#include "motion/MotionDefs.hpp"


#ifdef CTC_2_1
#define TURN_MULTIPLIER 1.00f // Amount to multiply gyroscopez by to get actual results
#else
#define TURN_MULTIPLIER 1.05f // Amount to multiply gyroscopez by to get actual results
#endif // CTC_2_1

using namespace std;

MotionOdometry::MotionOdometry() {
    reset();
}

Odometry MotionOdometry::updateOdometry(const SensorValues& sensors, Odometry walkChange) {
    float gyroZ = sensors.sensors[Sensors::InertialSensor_GyroscopeZ];
    // convert to radians per frame, same direction as walkChange
    gyroZ *= - MOTION_DT * TURN_MULTIPLIER;
    walkChange.turn = gyroZ;
    return walkChange;
}

void MotionOdometry::reset() {
}
