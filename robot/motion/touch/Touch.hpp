#pragma once

#include "types/ButtonPresses.hpp"
#include "types/SensorValues.hpp"
#include "perception/kinematics/Kinematics.hpp"
#ifndef Q_MOC_RUN
#include <boost/program_options/variables_map.hpp>
#endif

/**
 * Touch - repsonsible for reporting on the state of all non-vision senses
 *
 * NOTE - When adding any virtual method, be sure to override it in FilteredTouch to proxy to the real Touch instance
 */
class Touch {
   public:
      virtual ~Touch() {}
      /**
       * getSensors - read robot sensors
       * @return current values of sensors
       */
      virtual void readOptions(const boost::program_options::variables_map& config){}
      virtual void loadCache(){}
      virtual SensorValues getSensors(Kinematics &kinematics) = 0;

      virtual bool getStanding() = 0;
      virtual ButtonPresses getButtons() = 0;
      
      // Is only correctly implemented for LoLATouch
      virtual bool getLimp() = 0;
};
