#pragma once

#include "types/SensorValues.hpp"
#include "types/Odometry.hpp"

class MotionOdometry {
  public:
    MotionOdometry();
    Odometry updateOdometry(const SensorValues &sensors, Odometry walkChange);
    void reset();
};
