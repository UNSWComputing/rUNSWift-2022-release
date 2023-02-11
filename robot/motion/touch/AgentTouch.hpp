#pragma once

#include <semaphore.h>
#include "motion/touch/Touch.hpp"
#include "libagent/AgentData.hpp"


class AgentTouch : Touch {
   public:
      explicit AgentTouch(int team, int player_number, bool simulation);
      ~AgentTouch();
      SensorValues getSensors(Kinematics &kinematics);
      bool getStanding();
      ButtonPresses getButtons();
      bool getLimp();

   private:
      int shared_fd;
      AgentData* shared_data;
      sem_t* semaphore;
};
