#ifndef KENJI_BALANCE_GENERATOR_HPP
#define KENJI_BALANCE_GENERATOR_HPP

#include "motion/generator/Generator.hpp"
class Blackboard;

class KenjiBalanceGenerator : Generator
{
  public:
    explicit KenjiBalanceGenerator(Blackboard *bb);
    ~KenjiBalanceGenerator();
    JointValues makeJoints(
        ActionCommand::All *request,
        Odometry *odometry,
        const SensorValues &sensors,
        BodyModel &bodyModel,
        float ballX,
        float ballY,
        MotionDebugInfo &motionDebugInfo);
    bool isActive();
    void reset();
    void stop();
};

#endif // KENJI_BALANCE_GENERATOR_HPP
