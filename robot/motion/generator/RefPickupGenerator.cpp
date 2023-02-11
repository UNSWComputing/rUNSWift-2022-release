#include "motion/generator/RefPickupGenerator.hpp"
#include "motion/generator/ActionGenerator.hpp"
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"

using boost::program_options::variables_map;

RefPickupGenerator::RefPickupGenerator()
   : t(100), stopping(false), stopped(false) {
   llog(INFO) << "RefPickupGenerator constructed" << std::endl;
   standGen = (Generator *)(new ActionGenerator("stand"));
}

RefPickupGenerator::~RefPickupGenerator() {
   llog(INFO) << "RefPickupGenerator destroyed" << std::endl;
   delete standGen;
}

JointValues RefPickupGenerator::makeJoints(ActionCommand::All* request,
                                           Odometry* odometry,
                                           const SensorValues &sensors,
                                           BodyModel &bodyModel,
                                           float ballX,
                                           float ballY,
                                           MotionDebugInfo &motionDebugInfo) {
   if (stopping)
      if (!(--t))
         stopping = !(stopped = true);
   return standGen->makeJoints(request,odometry,sensors,bodyModel,ballX,ballY, motionDebugInfo);
}

bool RefPickupGenerator::isActive() {
   return !stopped || standGen->isActive();
}

void RefPickupGenerator::reset() {
   t = 100;
   stopping = stopped = false;
   standGen->reset();
}

void RefPickupGenerator::stop() {
   standGen->stop();
   stopping = true;
}

void RefPickupGenerator::readOptions(const boost::program_options::variables_map &config) {
   standGen->readOptions(config);
}
