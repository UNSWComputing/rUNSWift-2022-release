#pragma once

#include "blackboard/Blackboard.hpp"
#include "types/BehaviourRequest.hpp"
#include "types/SensorValues.hpp"
#include <boost/program_options/variables_map.hpp>

class SafetySkill {
   public:
      SafetySkill();
      ~SafetySkill();
      BehaviourRequest wrapRequest(const BehaviourRequest &request, const SensorValues &s, bool ukemiEnabled, Blackboard *bb);
      void readOptions(const boost::program_options::variables_map& config);
   private:
      float filtered_fsr_sum;
      int blink;
      float sag_angular_velocity;
      float cor_angular_velocity;
      float prev_angles[2];
      int fallingCounter;
      bool useGetups;
      bool simulation;
};
