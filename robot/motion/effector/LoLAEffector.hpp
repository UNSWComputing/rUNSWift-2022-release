#pragma once

#include "motion/effector/Effector.hpp"

class LoLAEffector : Effector {
public:
   explicit LoLAEffector(int team, int player_number);

   ~LoLAEffector() override;

   void actuate(JointValues joints, ActionCommand::LED leds, ActionCommand::Stiffen stiffen) override;
};
