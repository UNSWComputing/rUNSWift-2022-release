#ifndef IMU_CALIBRATION_SKILL_HPP
#define IMU_CALIBRATION_SKILL_HPP

#include "types/BehaviourRequest.hpp"

class SensorValues;

class IMUCalibrationSkill
{
  public:
    IMUCalibrationSkill();
    BehaviourRequest execute(const SensorValues &sensors);
  private:
    int num_frames;
    float sumAngleX;
    float sumAngleY;
    float sumGyroscopeX;
    float sumGyroscopeY;

};


#endif // IMU_CALIBRATION_SKILL_HPP
