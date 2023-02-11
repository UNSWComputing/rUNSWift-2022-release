#pragma once

#include "utils/body.hpp"
#include "types/JointValues.hpp"
#include <boost/serialization/version.hpp>

/**
 * A container for joint values, IMU values, FSR values, buttons and sonar
 * readings obtained from the Motion subsystem.
 */
struct SensorValues {
   SensorValues() {
      for (int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
         sensors[i] = NAN;
      }
   }

   SensorValues(bool zero) {
      joints = JointValues(true);
      for (int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
         sensors[i] = 0;
      }
   }

   JointValues joints;
   float sensors[Sensors::NUMBER_OF_SENSORS];

   template<class Archive>
   void serialize(Archive &ar, const unsigned int file_version) {
      ar & joints;
      if (file_version < 1)
      {
         // There used be 6 more sensors for the V4.
         // There is no attempt to read the sensor values from old dumps.
         // The following is just to keep dumps compatible for offnao.
         float tmp[Sensors::NUMBER_OF_SENSORS + 6];
         ar & tmp;
      } else {
         ar & sensors;
      }
   }
};

BOOST_CLASS_VERSION(SensorValues, 1);
