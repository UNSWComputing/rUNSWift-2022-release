#include "motion/generator/WalkEnginePreProcessor.hpp"

#define MAX_FORWARD_STEP 90

#define MAX_LEFT_STEP 50

#define MAX_TURN_STEP DEG2RAD(20)

#define FOOT_LENGTH 100
#define FORWARD_GAP 90
#define LEFT_GAP_KICK 60
#define LEFT_GAP_TURN_DRIBBLE 40

#define FORWARD_THRESHOLD 25
#define LEFT_THRESHOLD 20
#define TURN_THRESHOLD DEG2RAD(30)

#define TURN_DRIBBLE_MIN_TIME 400 //milliseconds
using namespace ActionCommand;
using namespace std;

int sign(float num) {
   int sign = 1;
   if (num < 0) {
      sign = -1;
   }
   return sign;
}

void toWalkRequest(ActionCommand::All* request) {
   request->body.actionType = Body::WALK;
   request->body.power = 0;
   request->body.bend = 1;
   request->body.speed = 1;
}

// LineUpEngine
WalkEnginePreProcessor::LineUpEngine::LineUpEngine(Walk2014Generator* walkEngine) {
   hasStarted = false;
   foot = Body::LEFT;
   this->walkEngine = walkEngine;
}

void WalkEnginePreProcessor::LineUpEngine::start(Body::Foot foot) {
   hasStarted = true;
   this->foot = foot;
}

void WalkEnginePreProcessor::LineUpEngine::reset() {
   hasStarted = false;
   walkEngine->exactStepsRequested = false;
}

bool WalkEnginePreProcessor::LineUpEngine::hasEnded(ActionCommand::All* request, float ballX, float ballY) {
   // Calculate required left gap (needs to be further out for kicks)
   int leftGap = 0;
   if (request->body.actionType == Body::KICK){
      leftGap = LEFT_GAP_KICK;
   } else if (request->body.actionType == Body::TURN_DRIBBLE){
      leftGap = LEFT_GAP_TURN_DRIBBLE;
   }

   int gapY = 0;
   if (foot == Body::LEFT){
      gapY = ballY - leftGap;
   } else if (foot == Body::RIGHT){
      gapY = ballY + leftGap;
   }
   int gapX = ballX - FOOT_LENGTH - FORWARD_GAP - max(walkEngine->forwardL,walkEngine->forwardR)*1000;

   bool forwardCheck = abs(gapX) < FORWARD_THRESHOLD;
   bool leftCheck = abs(gapY) < LEFT_THRESHOLD;
   bool headingCheck = fabs(request->body.turn) < request->body.speed; //speed is overloaded for behaviour input turn threshold

   bool linedUp = (forwardCheck && leftCheck && headingCheck);

   return linedUp;
}

void WalkEnginePreProcessor::LineUpEngine::preProcess(ActionCommand::All* request,
      float ballX,
      float ballY) {

   int forward = ballX - FOOT_LENGTH - FORWARD_GAP - max(walkEngine->forwardL,walkEngine->forwardR)*1000;

   int leftGap = 0;
   if (request->body.actionType == Body::KICK){
      leftGap = LEFT_GAP_KICK;
   } else if (request->body.actionType == Body::TURN_DRIBBLE){
      leftGap = LEFT_GAP_TURN_DRIBBLE;
   }

   int left = 0;
   if (foot == Body::LEFT){
      left = ballY - leftGap;
   } else if (foot == Body::RIGHT){
      left = ballY + leftGap;
   }

   request->body.forward = sign(forward) * min(MAX_FORWARD_STEP, abs(forward));
   request->body.left = sign(left) * min(MAX_LEFT_STEP, abs(left));

   // don't turn further than 30 degrees (TURN_THRESHOLD) away from the ball heading
   float heading = atan2(ballY, ballX);
   if (NORMALISE(request->body.turn - heading) > TURN_THRESHOLD) {
      request->body.turn = NORMALISE(TURN_THRESHOLD + heading);
   } else if (NORMALISE(request->body.turn - heading) < -TURN_THRESHOLD) {
      request->body.turn = NORMALISE(-TURN_THRESHOLD + heading);
   }
   request->body.turn = sign(request->body.turn) * min(MAX_TURN_STEP, fabs(request->body.turn/2));

   toWalkRequest(request);
   walkEngine->exactStepsRequested = true;
}

// TurnDribbleEngine
WalkEnginePreProcessor::TurnDribbleEngine::TurnDribbleEngine(Walk2014Generator* walkEngine) {
   this->walkEngine = walkEngine;
   foot = Body::LEFT;
   turnDribbleState = TurnDribbleEngine::END;
}

void WalkEnginePreProcessor::TurnDribbleEngine::reset() {
   turnDribbleState = TurnDribbleEngine::END;
   walkEngine->exactStepsRequested = false;
}

bool WalkEnginePreProcessor::TurnDribbleEngine::hasEnded() {
   return (turnDribbleState == TurnDribbleEngine::END);
}

void WalkEnginePreProcessor::TurnDribbleEngine::start(Body::Foot foot) {
   turnDribbleState = TurnDribbleEngine::INIT;
   this->foot = foot;
}

void WalkEnginePreProcessor::TurnDribbleEngine::preProcess(ActionCommand::All* request,
      BodyModel &bodyModel) {
   int direction = 1;
   bool leftTurnPhase = false; // use left foot ==> left foot is not swinging ==> left foot is not phase foot
                               //                ==> leftTurnPhase = false
   if (foot == Body::RIGHT) {
      direction = -1;
      leftTurnPhase = true;
   }

   //do transition
   if (turnDribbleState == TurnDribbleEngine::INIT && bodyModel.isLeftPhase != leftTurnPhase
         && walkEngine->t == 0) {
      turnDribbleState = TurnDribbleEngine::PRE_TURN;
   } else if (turnDribbleState == TurnDribbleEngine::PRE_TURN && bodyModel.isLeftPhase == leftTurnPhase
         && walkEngine->t == 0) {
      turnDribbleState = TurnDribbleEngine::TURN;
   } else if (turnDribbleState == TurnDribbleEngine::TURN && bodyModel.isLeftPhase != leftTurnPhase
         && walkEngine->t == 0) {
      turnDribbleState = TurnDribbleEngine::FORWARD;      
   } else if (turnDribbleState == TurnDribbleEngine::FORWARD && bodyModel.isLeftPhase == leftTurnPhase
         && walkEngine->t == 0) {
      turnDribbleState = TurnDribbleEngine::END;
      turnDribbleTimer.restart();
   }

   // set request
   request->body.left = 0;
   if (turnDribbleState == TurnDribbleEngine::TURN) {
      request->body.forward = 0;
      request->body.turn = direction * request->body.turn;
   } else if (turnDribbleState == TurnDribbleEngine::FORWARD) {
      request->body.forward = 100;
      request->body.turn = 0;
   } else if(turnDribbleState == TurnDribbleEngine::END) {
      request->body.forward = 70;
      request->body.turn = 0;
   } else {
      request->body.forward = 1; // hack so walk doesn't stand
      request->body.turn = 0;
   }
   toWalkRequest(request);
   walkEngine->exactStepsRequested = true;

}


WalkEnginePreProcessor::WalkEnginePreProcessor() {
   walkEngine = new Walk2014Generator();
   lineUpEngine = new LineUpEngine(walkEngine);
   turnDribbleEngine = new TurnDribbleEngine(walkEngine);
   isKicking = false;
}

WalkEnginePreProcessor::~WalkEnginePreProcessor() {
   delete walkEngine;
   delete lineUpEngine;
   delete turnDribbleEngine;
}

bool isLineUpRequired(Body::ActionType actionType) {
   // @ijnek: Turned off motion line up because it interferes with behaviour lineup
   return false;
//    return (actionType == Body::KICK || actionType == Body::DRIBBLE || actionType == Body::TURN_DRIBBLE);
}

JointValues WalkEnginePreProcessor::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors,
                                          BodyModel &bodyModel,
                                          float ballX,
                                          float ballY,
                                          MotionDebugInfo &motionDebugInfo) {
   Body::ActionType active = request->body.actionType;

   // don't try to turn dribble again within timer
   if (active == Body::TURN_DRIBBLE && turnDribbleEngine->turnDribbleTimer.elapsed_ms() < TURN_DRIBBLE_MIN_TIME) {
      toWalkRequest(request);
      active = Body::WALK;
      request->body.forward = 1;
      request->body.left = 0;
      request->body.turn = 0;
   }

   // persist until current state ends
   if (!isKicking) {
      if (!turnDribbleEngine->hasEnded()) {
         turnDribbleEngine->preProcess(request, bodyModel);
         active = Body::TURN_DRIBBLE;
      } else {
         turnDribbleEngine->reset();

         // line up is on demand, can be interrupted
         if (isLineUpRequired(request->body.actionType)) {
            // start line up
            if (!lineUpEngine->hasStarted) {
               lineUpEngine->start(request->body.foot);
            }

            if (!lineUpEngine->hasEnded(request, ballX, ballY)) {
               // do line up
               lineUpEngine->preProcess(request, ballX, ballY);
               active = Body::LINE_UP;
            } else {
               lineUpEngine->reset();
               if (request->body.actionType == Body::KICK) {
                  // walkEngine will set request back to walk after it's done kicking
                  // don't preProcess anything in the mean time
                  isKicking = true;
               } else if (request->body.actionType == Body::TURN_DRIBBLE) {
                  // start turn dribble
                  turnDribbleEngine->start(request->body.foot);
                  turnDribbleEngine->preProcess(request, bodyModel);
                  active = Body::TURN_DRIBBLE;
               }
            }
         } else {
            lineUpEngine->reset();
         }
      }
   }
   if (request->body.actionType == Body::TURN_DRIBBLE) {
                  // start turn dribble
                  turnDribbleEngine->start(request->body.foot);
                  turnDribbleEngine->preProcess(request, bodyModel);
                  active = Body::TURN_DRIBBLE;
   }

   if (request->body.actionType == Body::KICK) {
      request->body.turn = 0;    // 0 the turn used for line up heading adjustments
      request->body.speed = 0;   // reverted overloaded param for turn threshold
   }

   JointValues joints = walkEngine->makeJoints(request, odometry, sensors, bodyModel, ballX, ballY, motionDebugInfo);

   // walkEngine sets kick to walk2014 after kick finishes
   if (walkEngine->active.actionType == Body::KICK && request->body.actionType == Body::WALK) {
      isKicking = false;
   } else {
         request->body.actionType = active;
   }

   return joints;
}

bool WalkEnginePreProcessor::isActive() {
   return walkEngine->isActive() || !turnDribbleEngine->hasEnded();
}

void WalkEnginePreProcessor::readOptions(const boost::program_options::variables_map& config) {
   walkEngine->readOptions(config);
}

void WalkEnginePreProcessor::reset() {
   walkEngine->reset();
   lineUpEngine->reset();
   turnDribbleEngine->reset();
   isKicking = false;
}

void WalkEnginePreProcessor::stop() {
   walkEngine->stop();
}
