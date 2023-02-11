#include "IMUCalibrationSkill.hpp"
#include "types/SensorValues.hpp"

#define NUM_FRAMES_TO_CALIBRATE 250 // number of frames to take average
#define ANGLE_TOO_WRONG 7.0 // maximum offset an angle can be, anything above this, the robot probably isn't standing
#define GYROSCOPE_TOO_WRONG 1.5 // maximum offset a gyroscope can be, anything above this, the robot probably isn't stable

IMUCalibrationSkill::IMUCalibrationSkill()
  : num_frames(0),
    sumAngleX(0),
    sumAngleY(0),
    sumGyroscopeX(0),
    sumGyroscopeY(0)
{
}

BehaviourRequest IMUCalibrationSkill::execute(const SensorValues &sensors)
{
    // Note: these values are converted so they can be pasted straight into config files.
    //       since they are offsets, they have to be the negative of the bias.
    //       for readability, we use degrees in the cfg files.
    float angleX = -RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]);
    float angleY = -RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY]);
    float gyroscopeX = -RAD2DEG(sensors.sensors[Sensors::InertialSensor_GyroscopeX]);
    float gyroscopeY = -RAD2DEG(sensors.sensors[Sensors::InertialSensor_GyroscopeY]);

    if (abs(angleX) > ANGLE_TOO_WRONG ||
        abs(angleY) > ANGLE_TOO_WRONG ||
        abs(gyroscopeX) > GYROSCOPE_TOO_WRONG ||
        abs(gyroscopeY) > GYROSCOPE_TOO_WRONG)
    {
        std::cout << "Started IMU Calibration, please stiffen robot to stand up, place on even ground, and do not touch" << std::endl;
        num_frames = 0;
    }
    else if (num_frames == 0)
    {
        std::cout << "Calculating offsets....." << std::endl;
        num_frames++;
    }
    else if (num_frames < NUM_FRAMES_TO_CALIBRATE)
    {
        sumAngleX += angleX;
        sumAngleY += angleY;
        sumGyroscopeX += gyroscopeX;
        sumGyroscopeY += gyroscopeY;
        num_frames++;
    }
    else
    {
        std::cout << "Finished Calibration. Please paste / replace following in the body cfg file of the robot" << std::endl;
        std::cout << "[touch]" << std::endl
                  << "angleXOffset=" << sumAngleX / NUM_FRAMES_TO_CALIBRATE << std::endl
                  << "angleYOffset=" << sumAngleY / NUM_FRAMES_TO_CALIBRATE << std::endl
                  << "gyroscopeXOffset=" << sumGyroscopeX / NUM_FRAMES_TO_CALIBRATE << std::endl
                  << "gyroscopeYOffset=" << sumGyroscopeY / NUM_FRAMES_TO_CALIBRATE << std::endl;
        num_frames = 0;
        sumAngleX = 0;
        sumAngleY = 0;
        sumGyroscopeX = 0;
        sumGyroscopeY = 0;
    }

    BehaviourRequest request;
    request.actions.body = ActionCommand::Body::MOTION_CALIBRATE;
    return request;

}
