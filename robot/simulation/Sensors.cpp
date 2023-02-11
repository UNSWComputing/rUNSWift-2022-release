#include "Sensors.hpp"

namespace Simulation
{
    bool Sensors::toSensorValues(SensorValues* s_out)
    {
        SensorValues& s = *s_out;
        int i;
        for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
        {
            if (sptrs_[i] != &na_)
            {
                s.sensors[i] = *sptrs_[i];
            }
            else
            {
                s.sensors[i] = 0.0f;
            }
        }

        // ACCELEROMETER
        // Invert AccY (sims AccY is inverted)
        // TODO @jez debug DOUBLE CHECK SIM ACCY
        s.sensors[::Sensors::InertialSensor_AccelerometerY] = -s.sensors[::Sensors::InertialSensor_AccelerometerY];

        // Invert AccZ (sims AccZ is inverted)
        s.sensors[::Sensors::InertialSensor_AccelerometerZ] = -s.sensors[::Sensors::InertialSensor_AccelerometerZ];

        // Swap AccX and AccY (sim AccY is Nao AccX)
        float temp = s.sensors[::Sensors::InertialSensor_AccelerometerX];
        s.sensors[::Sensors::InertialSensor_AccelerometerX] = s.sensors[::Sensors::InertialSensor_AccelerometerY];
        s.sensors[::Sensors::InertialSensor_AccelerometerY] = temp;

        // GYROSCOPE
        // Convert gyro readings to radians
        s.sensors[::Sensors::InertialSensor_GyroscopeX] = DEG2RAD(s.sensors[::Sensors::InertialSensor_GyroscopeX]);
        s.sensors[::Sensors::InertialSensor_GyroscopeY] = DEG2RAD(s.sensors[::Sensors::InertialSensor_GyroscopeY]);
        s.sensors[::Sensors::InertialSensor_GyroscopeZ] = DEG2RAD(s.sensors[::Sensors::InertialSensor_GyroscopeZ]);

        // Invert GyroX (sims GyroX angle is inverted)
        s.sensors[::Sensors::InertialSensor_GyroscopeX] = -s.sensors[::Sensors::InertialSensor_GyroscopeX];

        // Invert GyroZ (sims GyroZ angle is possibly inverted)
        s.sensors[::Sensors::InertialSensor_GyroscopeZ] = -s.sensors[::Sensors::InertialSensor_GyroscopeZ];

        // Swap our GyroX and GyroY (opposite on robot / sim)
        temp = s.sensors[::Sensors::InertialSensor_GyroscopeX];
        s.sensors[::Sensors::InertialSensor_GyroscopeX] = s.sensors[::Sensors::InertialSensor_GyroscopeY];
        s.sensors[::Sensors::InertialSensor_GyroscopeY] = temp;

        // Don't have sonar currently in simulator, future implementation
        s.sensors[::Sensors::SonarLeft] = std::numeric_limits<float>::infinity();
        s.sensors[::Sensors::SonarRight] = std::numeric_limits<float>::infinity();

        return true;
    }

};
