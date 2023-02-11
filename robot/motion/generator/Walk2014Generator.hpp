/**
 * Walk2014Generator.hpp
 * BH 18 Jan 2014
 */

#pragma once

#include <cmath>
#include "motion/generator/Generator.hpp"
#include "motion/generator/BodyModel.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/ActionCommand.hpp"
#include "utils/Timer.hpp"
#include "motion/MotionOdometry.hpp"

class Walk2014Generator : Generator {
   public:
   explicit Walk2014Generator();
   ~Walk2014Generator();
   JointValues makeJoints(ActionCommand::All* request,
                          Odometry* odometry,
                          const SensorValues &sensors,
                          BodyModel &bodyModel,
                          float ballX,
                          float ballY,
                          MotionDebugInfo &motionDebugInfo);
   // once started by makeJoints, will not be stopped, unless called reset
   bool isActive();
   ActionCommand::Body active;

   enum Walk2014Option {
      STAND        = 0, // with knees straight and stiffness set to zero to conserve energy and heat generation in motors
      STANDUP      = 1, // process of moving from WALK crouch to STAND
      CROUCH       = 2, // process of transitioning from STAND to WALK
      WALK         = 3,
      READY        = 4, // stand still ready to walk
      KICK         = 5,
      NONE         = 6,
      NUMBER_OF_WALK_OPTIONS
   };

   enum WalkState {
      WALKING        = 0,
      STARTING       = 1,
      STOPPING       = 2,
      NOT_WALKING    = 3,
      NUMBER_OF_WALK_STATES
   };

   Walk2014Option walk2014Option;
   WalkState walkState;
   void readOptions(const boost::program_options::variables_map& config);
   void reset();
   void stop();
   friend class WalkEnginePreProcessor;

   private:
   bool exactStepsRequested;

   // legacy code
   bool stopping;
   bool stopped;
   // time step, timers,
   float dt;
   float t;
   float globalTime;

   float timer;
   float T;                                                // period of half a walk cycle

   // Nao H25 V4 dimensions - from utils/body.hpp and converted to meters
   float thigh;                                            // thigh length in meters
   float tibia;                                            // tibia length in meters
   float ankle;                                            // height of ankle above ground

   // Walk 2014 parameters in meters and seconds
   float hiph;                                             // variable vertical distance ground to hip in meters
   float hiph0;                                            // some initial hiph
   float foothL;                                           // meters left foot is lifted off the ground
   float foothR;                                           // meters right foot is lifted off the ground
   float nextFootSwitchT;                                  // next time-point at which to change support foot
   float forward;                                          // Omnidirectional walk forward/backward
   float lastForward;                                      // previous forward value accepted
   float forwardL0, forwardL;                              // variable left foot position wrt standing
   float forwardR0, forwardR;                              // variable right foot position wrt standing
   float leftR0, leftR;                                    // sideways step in meters for right foot
   float leftL0, leftL;                                    // sideways step in meters for left  foot
   float left, lastLeft;                                   // Omnidirectional walk left/right
   float turn, lastTurn;                                   // Omnidirectional walk CW / CCW
   float power;                                            // Omnidirectional walk - reserved for kicking
   float bend;
   float speed;
   ActionCommand::Body::Foot foot;                         // is this right?
   bool useShuffle;
   float stiffness;                                        // global stiffness (power to motors)
   float kneeStiffness;                                    // Specific dynamic knee stiffnesss to reduce overheating
   float ankleStiffness;                                   // Specific dynamic ankle stiffnesss to reduce overheating
   float currentVolume;                                    // Volume of the inputs requested
   float turnRL;                                           // turn variable
   float turnRL0;                                          // turnRL at support foot switch
   bool supportFoothasChanged;                             // Indicates that support foot is deemed to have changed
   bool weightHasShifted;
   float comOffset;                                        // move in meters of CoM in x-direction when walking to spread weight more evenly over foot
   float comOffset0;
   float targetComOffset;
   bool shouldEmergencyStep;                               // whether an emergency step should be taken
   float currentTurnChange;
   bool ballXYDebug;

   float timerSinceLastBlock;                              // Timer to ensure stability after a blocking step

   // Gyro filters
   float filteredGyroX;
   float filteredGyroY;

   // Angle filters
   float filteredAngleY;

   // Balance Controller Variables
   float sagittalBalanceAdjustment;
   float coronalBalanceAdjustment;
   float sagittalHipBalanceAdjustment;

   // PD gyro controller
   float KpGyro;           // Proportional gain
   float KdGyro;           // Derivative gain
   float preErrorGyro;      // Previous tick gyro error

   // PID Angle controller
   float KpAngle;          // Proportional gain
   float KiAngle;          // Integral gain
   float KdAngle;          // Derivative gain
   float angleError;       // Current angle error
   float angleErrorSum;    // Error sum
   float preErrorAngle;    // Previous tick angle error

   // Kicks
   float kickT;
   float rock;
   float kneePitchL, kneePitchR, lastKneePitch;
   float anklePitchL, anklePitchR;
   float lastKickForward;
   float lastSide;
   float lastAnklePitch;
   float lastShoulderRollAmp;
   float lastFooth;
   float lastRock;
   float lastKickTime;
   float shoulderPitchL;                                   // to swing left  arm while walking / kicking
   float shoulderPitchR;                                   // to swing right arm while walking / kicking
   float shoulderRollL;
   float shoulderRollR;
   float dynamicSide;
   float turnAngle;
   float lastKickTurn;
   float kneePitchEnd;
   float anklePitchStart;
   float anklePitchEnd;
   float swingDelayFactor;
   bool holdAnkle;
   bool hipBalance;

   int tempCounter;

   // Kick parameter constants
   float stableCounter; // number of frames robot was stable for
   int kickStableNumFrames; // number of frames robot must be stable for
   float kickStableAngleX; // AngleX to consider stable before kicking (deg)
   float kickExtraStableAngleX; // AngleX to consider extra stable before kicking (deg)
   float kickExtraStableGyroscopeX; // GyroscopeX to consider extra stable before kicking (deg/s)
   float kickGyroscopeXOntoSupportFootThresh; // Maximum GyroscopeX value when rocking towards the support foot, to consider stable before kicking (deg/s)
   float kickGyroscopeXAwayFromSupportFootThresh; // Maximum GyroscopeX value when rocking away from the support foot, to consider stable before kicking (deg/s)
   float shiftPeriod; // time to shift weight on to one leg
   float shiftEndPeriod; // time to shift weight back from one leg
   float backPhase; // time to move kick foot back
   float kickPhase; // time to swing kick foot
   float throughPhase; // time to hold kick foot
   float endPhase; // time to return kick foot to zero position
   float shoulderRollAmpMultiplier; // arm roll to leave room for kicking
   float kickLean; // the base amount of lean when kicking - defined in options.cpp
   bool kickLeanOffsetLVaried; // whether to vary left kick lean offset with ball position.
   bool kickLeanOffsetRVaried; // whether to vary right kick lean offset with ball position.
   float kickLeanOffsetL; // kick lean offset when kicking with left foot for different robots
   float kickLeanOffsetR; // kick lean offset when kicking with right foot for different robots
   // Kick lean offset when kicking a ball close to the robot's centre line for different robots.
   float kickLeanOffsetLInner; // with left foot
   float kickLeanOffsetRInner; // with right foot
   // Kick lean offset when kicking a ball moderately distant from the robot's centre line for different robots.
   float kickLeanOffsetLMid; // with left foot
   float kickLeanOffsetRMid; // with right foot
   // Kick lean offset when kicking a ball far from the robot's centre line for different robots.
   float kickLeanOffsetLOuter; // with left foot
   float kickLeanOffsetROuter; // with right foot

   bool runningKickCalibration;

   //for odometry updates
   float prevTurn;
   float prevForwardL;
   float prevForwardR;
   float prevLeftL;
   float prevLeftR;

   void initialise();

   // Use for iterative inverse kinematics for turning (see documentation BH 2010)
   struct Hpr {
      float Hp;
      float Hr;
      // Hpr(): Hp(0.0f), Hr(0.0f) { }
   };

   /**
    * Calculates the lean angle given:
    * the commanded left step in meters,
    * time thorough walkStep phase, and total walkStep Phase time
    */
   float leftAngle();

   MotionOdometry motionOdometry;
   Odometry updateOdometry(bool isLeftSwingFoot);

   /**
    * returns smooth values in the range 0 to 1 given time progresses from 0 to period
    */
   float parabolicReturn(float); // step function with deadTimeFraction/2 delay
   float parabolicReturnMod(float); // step function with deadTimeFraction/2 delay
   float parabolicStep(float time, float period, float deadTimeFraction);                  // same as above, except a step up
   float linearStep(float time, float period);                                             // linear increase from 0 to 1
   float interpolateSmooth(float start, float end, float tCurrent, float tEnd);            // sinusoidal interpolation
   float squareSmooth(float start, float end, float tCurrent, float tEnd);            //

   //float moveSin(float start, float finish, float period);
   // sinusoidal step - not used yet

   /**
    * Sets kick settings when starting the kick
    */
   void prepKick(bool isLeft, BodyModel &bodyModel);

   bool canAbortKick();
   bool canAbortKickBeforeLean();

   /**
    * Specifies the joint parameters that form the kick
    */
   void makeForwardKickJoints(float kickLean, float kickStepH, float &footh, float &forward, float &side,
    float &kneePitch, float &shoulderRoll, float &anklePitch, float &ballX, float &ballY, ActionCommand::All* request,
    const SensorValues &sensors, float &gyroXOntoSupportFoot, MotionDebugInfo &motionDebugInfo);

   /**
    * Adds kick parameters to final joint values
    */
   void addKickJoints(JointValues &j);

    /**
     * Calculates the required stiffness for the requested walk based on the volume
     */
   float calculateKneeStiffness(float volume);
   float calculateAnkleStiffness(float volume);

   // Foot to Body coord transform used to calculate IK foot position and ankle tilts to keep foot in ground plane when turning
   XYZ_Coord mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap,
                  float Ar, float xf, float yf, float zf);
   Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
                 float Ar, float xf, float yf, float zf, XYZ_Coord e);

      float clamp(const int min, float value, const int max) const;
};
