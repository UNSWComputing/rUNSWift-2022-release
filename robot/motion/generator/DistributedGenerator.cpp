#include "motion/generator/DistributedGenerator.hpp"
#include "motion/generator/ActionGenerator.hpp"
#include "motion/generator/DeadGenerator.hpp"
#include "motion/generator/HeadGenerator.hpp"
#include "motion/generator/NullGenerator.hpp"
#include "motion/generator/RefPickupGenerator.hpp"
#include "motion/generator/WalkEnginePreProcessor.hpp"
#include "motion/generator/GetupGenerator.hpp"

#include "utils/body.hpp"
#include "utils/Logger.hpp"

using ActionCommand::Body;
using boost::program_options::variables_map;


/*-----------------------------------------------------------------------------
 * Distributed Generator
 * ---------------------
 * This generator switches between all required generators as requested.
 *---------------------------------------------------------------------------*/
DistributedGenerator::DistributedGenerator(bool simulation)
   : isStopping(false),
     current_generator(Body::NONE),
     prev_generator(Body::NONE),
     requestedDive(Body::NONE) {

   headGenerator = (Generator*)(new HeadGenerator());
   if (!headGenerator)
      llog(FATAL) << "headGenerator is NULL!" << std::endl;

   // TODO(dpad): Rewrite these ugly llogs to simply loop through bodyGenerators
   // and print out the string name
   bodyGenerators[Body::NONE] = (Generator*)(new NullGenerator());
   if (!bodyGenerators[Body::NONE])
      llog(FATAL) << "bodyGenerators[NONE] is NULL!" << std::endl;

   bodyGenerators[Body::STAND] = (Generator*)(new ActionGenerator("stand"));

   if (!bodyGenerators[Body::STAND])
      llog(FATAL) << "bodyGenerators[STAND] is NULL!" << std::endl;

   bodyGenerators[Body::MOTION_CALIBRATE] =
           (Generator*)(new ActionGenerator("standStraight"));

   if (!bodyGenerators[Body::MOTION_CALIBRATE])
      llog(FATAL) << "bodyGenerators[MOTION_CALIBRATE] is NULL!" << std::endl;

   bodyGenerators[Body::STAND_STRAIGHT] =
           (Generator*)(new ActionGenerator("standStraight"));

   if (!bodyGenerators[Body::STAND_STRAIGHT])
      llog(FATAL) << "bodyGenerators[STAND_STRAIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::WALK] = (Generator*)(new WalkEnginePreProcessor());
   if (!bodyGenerators[Body::WALK])
      llog(FATAL) << "bodyGenerators[WALK] is NULL!" << std::endl;

   bodyGenerators[Body::KICK] = bodyGenerators[Body::WALK];

   bodyGenerators[Body::LINE_UP] = bodyGenerators[Body::WALK];

   bodyGenerators[Body::TURN_DRIBBLE] = bodyGenerators[Body::WALK];

   if (simulation) {
      bodyGenerators[Body::GETUP_FRONT] = new ActionGenerator("FRONT");
   } else {
      bodyGenerators[Body::GETUP_FRONT] = new GetupGenerator("FRONT");
   }
   if (!bodyGenerators[Body::GETUP_FRONT])
      llog(FATAL) << "bodyGenerators[GETUP_FRONT] is NULL!" << std::endl;

   if (simulation) {
      bodyGenerators[Body::GETUP_BACK] = new ActionGenerator("BACK");
   } else {
      bodyGenerators[Body::GETUP_BACK] = new GetupGenerator("BACK");
   }
   if (!bodyGenerators[Body::GETUP_BACK])
      llog(FATAL) << "bodyGenerators[GETUP_BACK] is NULL!" << std::endl;

   bodyGenerators[Body::TIP_OVER] = (Generator*)
                                       (new ActionGenerator("tipOver"));
   if (!bodyGenerators[Body::TIP_OVER])
      llog(FATAL) << "bodyGenerators[TIP_OVER] is NULL!" << std::endl;

   bodyGenerators[Body::INITIAL] = (Generator*)
                                   (new ActionGenerator("initial"));

   if (!bodyGenerators[Body::INITIAL])
      llog(FATAL) << "bodyGenerators[INITIAL] is NULL!" << std::endl;

   bodyGenerators[Body::LIMP] = (Generator*)(new DeadGenerator());
   if (!bodyGenerators[Body::LIMP])
      llog(FATAL) << "bodyGenerators[LIMP] is NULL!" << std::endl;

   bodyGenerators[Body::REF_PICKUP] = (Generator*)(new RefPickupGenerator());
   if (!bodyGenerators[Body::REF_PICKUP])
      llog(FATAL) << "bodyGenerators[REF_PICKUP] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_SIT] = (Generator*)
                                      (new ActionGenerator("goalieSit"));
   if (!bodyGenerators[Body::GOALIE_SIT])
      llog(FATAL) << "bodyGenerators[GOALIE_SIT] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_DIVE_LEFT] = (Generator*)
                                            (new ActionGenerator("goalieDiveLeft"));
   if (!bodyGenerators[Body::GOALIE_DIVE_LEFT])
      llog(FATAL) << "bodyGenerators[GOALIE_DIVE_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_DIVE_RIGHT] = (Generator*)
                                             (new ActionGenerator("goalieDiveRight"));
   if (!bodyGenerators[Body::GOALIE_DIVE_RIGHT])
      llog(FATAL) << "bodyGenerators[GOALIE_DIVE_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_CENTRE] = (Generator*)
                                             (new ActionGenerator("goalieCentre")); //goalieCentre
   if (!bodyGenerators[Body::GOALIE_CENTRE])
      llog(FATAL) << "bodyGenerators[GOALIE_CENTRE] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_UNCENTRE] = (Generator*)
                                             (new ActionGenerator("goalieUncentre"));
   if (!bodyGenerators[Body::GOALIE_UNCENTRE])
      llog(FATAL) << "bodyGenerators[GOALIE_UNCENTRE] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_INITIAL] = (Generator*)
                                             (new ActionGenerator("goalieInitial"));
   if (!bodyGenerators[Body::GOALIE_INITIAL])
      llog(FATAL) << "bodyGenerators[GOALIE_INITIAL] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_AFTERSIT_INITIAL] = (Generator*)
                                             (new ActionGenerator("goalieInitial"));
   if (!bodyGenerators[Body::GOALIE_AFTERSIT_INITIAL])
      llog(FATAL) << "bodyGenerators[GOALIE_AFTERSIT_INITIAL] is NULL!" << std::endl;

   bodyGenerators[Body::DEFENDER_CENTRE] = (Generator*)
                                          (new ActionGenerator("defenderCentre"));
   if (!bodyGenerators[Body::DEFENDER_CENTRE])
     llog(FATAL) << "bodyGenerators[DEFENDER_CENTRE] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_FAST_SIT] = (Generator*)
                                          (new ActionGenerator("goalieFastSit"));
   if (!bodyGenerators[Body::GOALIE_FAST_SIT])
     llog(FATAL) << "bodyGenerators[GOALIE_FAST_SIT] is NULL!" << std::endl;

   bodyGenerators[Body::TEST_ARMS] = (Generator*)
                                          (new ActionGenerator("testArms"));
   if (!bodyGenerators[Body::TEST_ARMS])
     llog(FATAL) << "bodyGenerators[TEST_ARMS] is NULL!" << std::endl;

   bodyGenerators[Body::RAISE_ARM] = (Generator*)
                                          (new ActionGenerator("raiseArm"));
   if (!bodyGenerators[Body::RAISE_ARM])
     llog(FATAL) << "bodyGenerators[RAISE_ARM] is NULL!" << std::endl;

   bodyGenerators[Body::UKEMI_FRONT] = (Generator*)
                                          (new ActionGenerator("ukemiFront"));
   if (!bodyGenerators[Body::UKEMI_FRONT])
     llog(FATAL) << "bodyGenerators[UKEMI_FRONT] is NULL!" << std::endl;

   bodyGenerators[Body::UKEMI_BACK] = (Generator*)
                                          (new ActionGenerator("ukemiBack"));
   if (!bodyGenerators[Body::UKEMI_BACK])
     llog(FATAL) << "bodyGenerators[UKEMI_BACK] is NULL!" << std::endl;

   bodyGenerators[Body::GOALIE_STAND] = (Generator*)
                                          (new ActionGenerator("goalieStand"));
   if (!bodyGenerators[Body::GOALIE_STAND])
     llog(FATAL) << "bodyGenerators[GOALIE_STAND] is NULL!" << std::endl;

   bodyGenerators[Body::SIT] = (Generator*)
                                          (new ActionGenerator("sit"));
   if (!bodyGenerators[Body::SIT])
     llog(FATAL) << "bodyGenerators[SIT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_KICK_IN_RIGHT] = (Generator*)
                                          (new ActionGenerator("signalKickInRight"));
   if (!bodyGenerators[Body::SIGNAL_KICK_IN_RIGHT])
     llog(FATAL) << "bodyGenerators[SIGNAL_KICK_IN_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_KICK_IN_LEFT] = (Generator*)
                                          (new ActionGenerator("signalKickInLeft"));
   if (!bodyGenerators[Body::SIGNAL_KICK_IN_LEFT])
     llog(FATAL) << "bodyGenerators[SIGNAL_KICK_IN_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_GOAL_KICK_RIGHT] = (Generator*)
                                          (new ActionGenerator("signalGoalKickRight"));
   if (!bodyGenerators[Body::SIGNAL_GOAL_KICK_RIGHT])
     llog(FATAL) << "bodyGenerators[SIGNAL_GOAL_KICK_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_GOAL_KICK_LEFT] = (Generator*)
                                          (new ActionGenerator("signalGoalKickLeft"));
   if (!bodyGenerators[Body::SIGNAL_GOAL_KICK_LEFT])
     llog(FATAL) << "bodyGenerators[SIGNAL_GOAL_KICK_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_CORNER_KICK_RIGHT] = (Generator*)
                                          (new ActionGenerator("signalCornerKickRight"));
   if (!bodyGenerators[Body::SIGNAL_CORNER_KICK_RIGHT])
     llog(FATAL) << "bodyGenerators[SIGNAL_CORNER_KICK_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_CORNER_KICK_LEFT] = (Generator*)
                                          (new ActionGenerator("signalCornerKickLeft"));
   if (!bodyGenerators[Body::SIGNAL_CORNER_KICK_LEFT])
     llog(FATAL) << "bodyGenerators[SIGNAL_CORNER_KICK_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_GOAL_RIGHT] = (Generator*)
                                          (new ActionGenerator("signalGoalRight"));
   if (!bodyGenerators[Body::SIGNAL_GOAL_RIGHT])
     llog(FATAL) << "bodyGenerators[SIGNAL_GOAL_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_GOAL_LEFT] = (Generator*)
                                          (new ActionGenerator("signalGoalLeft"));
   if (!bodyGenerators[Body::SIGNAL_GOAL_LEFT])
     llog(FATAL) << "bodyGenerators[SIGNAL_GOAL_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_PUSHING_FREE_KICK_RIGHT] = (Generator*)
                                          (new ActionGenerator("signalPushingFreeKickRight"));
   if (!bodyGenerators[Body::SIGNAL_PUSHING_FREE_KICK_RIGHT])
     llog(FATAL) << "bodyGenerators[SIGNAL_PUSHING_FREE_KICK_RIGHT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_PUSHING_FREE_KICK_LEFT] = (Generator*)
                                          (new ActionGenerator("signalPushingFreeKickLeft"));
   if (!bodyGenerators[Body::SIGNAL_PUSHING_FREE_KICK_LEFT])
     llog(FATAL) << "bodyGenerators[SIGNAL_PUSHING_FREE_KICK_LEFT] is NULL!" << std::endl;

   bodyGenerators[Body::SIGNAL_FULL_TIME] = (Generator*)
                                          (new ActionGenerator("signalFullTime"));
   if (!bodyGenerators[Body::SIGNAL_FULL_TIME])
     llog(FATAL) << "bodyGenerators[SIGNAL_FULL_TIME] is NULL!" << std::endl;

   llog(INFO) << "DistributedGenerator constructed" << std::endl;


}

/*-----------------------------------------------------------------------------
 * Destructor
 *---------------------------------------------------------------------------*/
DistributedGenerator::~DistributedGenerator() {
   delete headGenerator;
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i)
      if (bodyGenerators[i]) {
         delete bodyGenerators[i];
         for (uint8_t j = i + 1; j < Body::NUM_ACTION_TYPES; ++j)
            if (bodyGenerators[j] == bodyGenerators[i])
               bodyGenerators[j] = NULL;
      }
   llog(INFO) << "DistributedGenerator destroyed" << std::endl;
}

/*-----------------------------------------------------------------------------
 * makeJoints
 * Returns the joint values requested by whichever generator we're using
 *---------------------------------------------------------------------------*/
JointValues DistributedGenerator::makeJoints(ActionCommand::All* request,
                                             Odometry* odometry,
                                             const SensorValues &sensors,
                                             BodyModel &bodyModel,
                                             float ballX,
                                             float ballY,
                                             MotionDebugInfo &motionDebugInfo) {

   // If we're requesting a dive, set requestedDive variable
   if(requestedDive == Body::NONE
         && !(
            current_generator == Body::GOALIE_CENTRE ||
            current_generator == Body::GOALIE_DIVE_LEFT ||
            current_generator == Body::GOALIE_DIVE_RIGHT ||
            current_generator == Body::DEFENDER_CENTRE
            )
         && (
            request->body.actionType == Body::GOALIE_CENTRE ||
            request->body.actionType == Body::GOALIE_DIVE_LEFT ||
            request->body.actionType == Body::GOALIE_DIVE_RIGHT ||
            current_generator == Body::DEFENDER_CENTRE
            )) {
      requestedDive = request->body.actionType;
   }

   JointValues fromBody;
   bool usesHead = false;

   // Check the priority of the requested action compared to the current action
   if (ActionCommand::priorities[request->body.actionType] >
       ActionCommand::priorities[current_generator]) {
      reset();
      isStopping = false;
   }

   if (!bodyGenerators[current_generator]->isActive()) {
      if (bodyGenerators[current_generator] != bodyGenerators[request->body.actionType]
            || isStopping
            || (current_generator == Body::GETUP_FRONT && request->body.actionType == Body::GETUP_FRONT)
            || (current_generator == Body::GETUP_BACK && request->body.actionType == Body::GETUP_BACK)) {
         bodyGenerators[current_generator]->reset();
      }

      if (current_generator == Body::GOALIE_CENTRE && request->body.actionType != Body::GOALIE_CENTRE) {
         current_generator = Body::GOALIE_UNCENTRE;
      } else if (current_generator == Body::SIT && request->body.actionType != Body::SIT) {
         current_generator = Body::INITIAL; // If we're sitting stand up gracefully
      } else {
         current_generator = request->body.actionType;
      }
      isStopping = false;
   } else if (bodyGenerators[current_generator]->isActive() &&
              bodyGenerators[current_generator] !=
              bodyGenerators[request->body.actionType]) {
      // Special case to let kicks continue instead of being interrupted by stand
      if (current_generator != Body::KICK || request->body.actionType != Body::STAND) {
         bodyGenerators[current_generator]->stop();
         isStopping = true;
      }
   }

   if(current_generator == requestedDive){
      requestedDive = Body::NONE;
   }

   switch (current_generator) {
   case Body::NONE:             usesHead = false; break;
   case Body::STAND:            usesHead = false; break;
   case Body::WALK:             usesHead = false; break;
   case Body::GETUP_FRONT:      usesHead = true;  break;
   case Body::GETUP_BACK:       usesHead = true;  break;
   case Body::TIP_OVER:         usesHead = true;  break;
   case Body::INITIAL:          usesHead = true;  break;
   case Body::KICK:             usesHead = false; break;
   case Body::TURN_DRIBBLE:     usesHead = false; break;
   case Body::LIMP:             usesHead = true;  break;
   case Body::REF_PICKUP:       usesHead = false; break;
   case Body::GOALIE_SIT:       usesHead = true; break;
   case Body::GOALIE_FAST_SIT:  usesHead = true; break;
   case Body::GOALIE_DIVE_LEFT: usesHead = true; break;
   case Body::GOALIE_DIVE_RIGHT: usesHead = true; break;
   case Body::GOALIE_CENTRE:    usesHead = true; break;
   case Body::GOALIE_UNCENTRE:  usesHead = true; break;
   case Body::GOALIE_INITIAL:   usesHead = true; break;
   case Body::GOALIE_AFTERSIT_INITIAL: usesHead = true; break;
   case Body::DEFENDER_CENTRE:  usesHead = false; break;
   case Body::MOTION_CALIBRATE: usesHead = false; break;
   case Body::STAND_STRAIGHT:   usesHead = false; break;
   case Body::LINE_UP:          usesHead = false; break;
   case Body::TEST_ARMS:        usesHead = false; break;
   case Body::RAISE_ARM:        usesHead = false; break;
   case Body::UKEMI_FRONT:      usesHead = true;  break;
   case Body::UKEMI_BACK:       usesHead = true;  break;
   case Body::GOALIE_STAND:     usesHead = false; break;
   case Body::SIT:              usesHead = true; break;
   case Body::SIGNAL_KICK_IN_RIGHT: usesHead = false; break;
   case Body::SIGNAL_KICK_IN_LEFT: usesHead = false; break;
   case Body::SIGNAL_GOAL_KICK_RIGHT: usesHead = false; break;
   case Body::SIGNAL_GOAL_KICK_LEFT: usesHead = false; break;
   case Body::SIGNAL_CORNER_KICK_RIGHT: usesHead = false; break;
   case Body::SIGNAL_CORNER_KICK_LEFT: usesHead = false; break;
   case Body::SIGNAL_GOAL_RIGHT: usesHead = false; break;
   case Body::SIGNAL_GOAL_LEFT: usesHead = false; break;
   case Body::SIGNAL_PUSHING_FREE_KICK_RIGHT: usesHead = false; break;
   case Body::SIGNAL_PUSHING_FREE_KICK_LEFT: usesHead = false; break;
   case Body::SIGNAL_FULL_TIME: usesHead = false; break;
   case Body::NUM_ACTION_TYPES: usesHead = false; break;
   }

   // Robot will not stiffen without this
   fromBody = bodyGenerators[current_generator]->
      makeJoints(request, odometry, sensors, bodyModel, ballX, ballY, motionDebugInfo);

   if(current_generator == Body::KICK && request->body.actionType == Body::WALK) {
      current_generator = Body::WALK;
   }
   if (!usesHead) {
      JointValues fromHead = headGenerator->
                             makeJoints(request, odometry, sensors, bodyModel, ballX, ballY, motionDebugInfo);
      for (uint8_t i = Joints::HeadYaw; i <= Joints::HeadPitch; ++i) {
         fromBody.angles[i] = fromHead.angles[i];
         fromBody.stiffnesses[i] = fromHead.stiffnesses[i];
      }
   }
   prev_generator = current_generator;

   return fromBody;
}

bool DistributedGenerator::isActive() {
   return true;
}

void DistributedGenerator::reset() {
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i) {
      bodyGenerators[i]->reset();
   }
   headGenerator->reset();
   current_generator = ActionCommand::Body::NONE;
}

void DistributedGenerator::readOptions(const boost::program_options::variables_map &config) {
   for (uint8_t i = 0; i < Body::NUM_ACTION_TYPES; ++i) {
      bodyGenerators[i]->readOptions(config);
   }
   headGenerator->readOptions(config);
}
