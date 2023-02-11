/**
 * ActionCommand.hpp
 * Description: Commands which are accepted by the Locomotion Module
 * This is the new interface between the Behaviour and Locomotion
 * Body are for the walks and special actions which use the body
 * Head are for the head yaw and pitch
 * LED are for the ear, face, chest and foot LEDs
 * Tidied up from 2009 code
 */

#pragma once

#include <stdint.h>

#include <iostream>

#include "utils/body.hpp"

/* Remember to update your constants in python/wrappers/ActionCommand */

namespace ActionCommand {

/**
 * Command for controlling the body
 * Note: Some ActionType Commands WILL disable the head
 */
   struct Body {
      // Predefined actions. These take precedence over walk parameters
      enum ActionType {
         NONE = 0,
         STAND,
         WALK,
         TURN_DRIBBLE,
         GETUP_FRONT,
         GETUP_BACK,
         TIP_OVER,
         KICK,
         INITIAL,
         LIMP,
         REF_PICKUP,
         GOALIE_SIT,
         GOALIE_DIVE_RIGHT,
         GOALIE_DIVE_LEFT,
         GOALIE_CENTRE,
         GOALIE_UNCENTRE,
         GOALIE_INITIAL,
         GOALIE_AFTERSIT_INITIAL,
         DEFENDER_CENTRE,
         GOALIE_FAST_SIT,
         MOTION_CALIBRATE,
         STAND_STRAIGHT,
         LINE_UP,
         TEST_ARMS,
         RAISE_ARM,
         UKEMI_FRONT,
         UKEMI_BACK,
         GOALIE_STAND,
         SIT,
         SIGNAL_KICK_IN_RIGHT,
         SIGNAL_KICK_IN_LEFT,
         SIGNAL_GOAL_KICK_RIGHT,
         SIGNAL_GOAL_KICK_LEFT,
         SIGNAL_CORNER_KICK_RIGHT,
         SIGNAL_CORNER_KICK_LEFT, 
         SIGNAL_GOAL_RIGHT,
         SIGNAL_GOAL_LEFT,
         SIGNAL_PUSHING_FREE_KICK_RIGHT,
         SIGNAL_PUSHING_FREE_KICK_LEFT, 
         SIGNAL_FULL_TIME,                
         NUM_ACTION_TYPES,
      };
      ActionType actionType;

      // Walk/Kick Parameters
      int forward; // How far forward (negative for backwards)  (mm)
      int left;  // How far to the left (negative for rightwards) (mm)
      float turn; // How much anti-clockwise turn (negative for clockwise) (rad)
      float power; // How much kick power (0.0-1.0)
      float bend;
      float speed;

      enum Foot {
         LEFT = 0,
         RIGHT
      };
      Foot foot;

      bool useShuffle;

      // Set this to true if we want to put our arms back
      bool leftArmBehind;
      bool rightArmBehind;

      bool blocking; // whether we are trying to block a moving ball
      bool extraStableKick; // whether we want to ensure the kick is stable, even if the execution is slow

      /**
       * Constructor for walks and kicks
       * @param at Action Type
       * @param f  How far forward (mm)
       * @param l  How far to the left (mm)
       * @param t  How much counter-clockwise turn (rad)
       * @param p  How much power
       * @param bend  Angle to bend knees (rad)
       * @param s  Speed of walk
       * @param k  Direction to kick (rad)
       * @param ft  Which foot to use
       * @param useShuffle use shuffle (low step height) or not
       * @param leftArmBehind make left arm go behind back
       * @param rightArmBehind make right arm go behind back
       * @param blocking to tell walk gen we need a big side step
       * @param extraStableKick  to tell walk gen we prefer kick stability over execution time
       * @see http://runswift.cse.unsw.edu.au/confluence/display/rc2010/Movement%2C+walk%2C+kicks
       */
      Body(ActionType at, int f = 0, int l = 0, float t = 0.0, float p = 1.0,
           float bend = 15.0, float s = 1.0, Foot ft = LEFT,
           bool useShuffle=false, bool leftArmBehind=false, bool rightArmBehind=false,
           bool blocking=false, bool extraStableKick=false)
         : actionType(at),
           forward(f),
           left(l),
           turn(t),
           power(p),
           bend(bend),
           speed(s),
           foot(ft),
           useShuffle(useShuffle),
           leftArmBehind(leftArmBehind),
           rightArmBehind(rightArmBehind),
           blocking(blocking),
           extraStableKick(extraStableKick) {}

      /* Boost python makes using default arguements difficult.
       * Define an arguementless constructor to wrap
       */
      Body()
         : actionType(NONE),
           forward(0),
           left(0),
           turn(0),
           power(0),
           bend(0),
           speed(0),
           foot(LEFT),
           useShuffle(false),
           leftArmBehind(false),
           rightArmBehind(false),
           blocking(false),
           extraStableKick(false) {}

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & actionType;
         ar & forward;
         ar & left;
         ar & turn;
         ar & power;
      }
   };

   const uint8_t priorities[Body::NUM_ACTION_TYPES] = {
      0, // NONE
      0, // STAND
      0, // WALK
      0, // TURN_DRIBBLE
      3, // GETUP_FRONT
      3, // GETUP_BACK
      3, // TIP_OVER
      0, // KICK
      2, // INITIAL
      1, // LIMP
      1, // REF_PICKUP
      2, // GOALIE_SIT
      3, // GOALIE_DIVE_LEFT
      3, // GOALIE_DIVE_RIGHT
      3, // GOALIE_CENTRE
      2, // GOALIE_UNCENTRE
      0, // GOALIE_INITIAL
      0, // GOALIE_AFTERSIT_INITIAL
      2, // DEFENDER_CENTRE
      2, // GOALIE FAST SIT
      0, // MOTION_CALIBRATE
      0, // STAND_STRAIGHT
      0, // LINE_UP
      0, // TEST_ARMS
      1, // RAISE_ARM
      3, // UKEMI_FRONT
      3, // UKEMI_BACK
      1, // GOALIE_STAND
      1, // SIT
      1, // SIGNAL_KICK_IN_RIGHT
      1, // SIGNAL_KICK_IN_LEFT
      1, // SIGNAL_GOAL_KICK_RIGHT
      1, // SIGNAL_GOAL_KICK_LEFT
      1, // SIGNAL_CORNER_KICK_RIGHT
      1, // SIGNAL_CORNER_KICK_LEFT 
      1, // SIGNAL_GOAL_RIGHT
      1, // SIGNAL_GOAL_LEFT
      1, // SIGNAL_PUSHING_FREE_KICK_RIGHT
      1, // SIGNAL_PUSHING_FREE_KICK_LEFT 
      1, // SIGNAL_FULL_TIME
   };

/**
 * Command for controlling the head
 */
   struct Head {
      float yaw;      // LEFT-RIGHT motion. Positive is LEFT
      float pitch;    // UP-DOWN angle. Positive is DOWN
      bool isRelative; // TRUE to add to current head angles [DEFAULT]
      float yawSpeed; // Speed of the yaw [0.0, 1.0]
      float pitchSpeed; // Speed of the pitch [0.0, 1.0]

      /**
       * Constructor
       * @param y Yaw amount (Left is positive) (rad)
       * @param p Pitch amount (Down is positive) (rad)
       * @param r Enable relative adjustment (default). False for absolute
       * @param ys Yaw speed [0.0, 1.0]
       * @param ps Pitch speed [0.0, 1.0]
       */
      Head(float y, float p = 0.0, bool r = true,
           float ys = 1.0, float ps = 1.0) : yaw(y),
                                             pitch(p),
                                             isRelative(r),
                                             yawSpeed(ys),
                                             pitchSpeed(ps) {}

      Head()
         : yaw(0.0),
           pitch(0.0),
           isRelative(true),
           yawSpeed(1.0),
           pitchSpeed(1.0) {}

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & yaw;
         ar & pitch;
         ar & isRelative;
         ar & yawSpeed;
         ar & pitchSpeed;
      }
   };

   struct rgb {
      bool red;
      bool green;
      bool blue;

      rgb(bool r = false, bool g = false, bool b = false) : red(r),
                                                            green(g),
                                                            blue(b) {}

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & red;
         ar & green;
         ar & blue;
      }
   };

   struct LED {

      // NOTE: leftEar is not used and is handled entirely in libagent/LoLA*
      uint16_t leftEar; // Number of left ear segments lit [10-bit field]
      uint16_t rightEar; // Number of right ear segments lit [10-bit field]
      rgb leftEye;     // Colour of left eye (default: white)
      rgb rightEye;    // Colour of right eye (default: white)
      rgb chestButton; // Colour of chest button (default: white)
      rgb leftFoot;    // Colour of left foot (default: off)
      rgb rightFoot;   // Colour of right foot (default: off)

      LED(rgb leye, rgb reye = rgb(true, true, true), rgb cb = rgb(true, true, true),
          rgb lf = rgb(), rgb rf = rgb()) : leftEar(0x3FF),
                                            rightEar(0x3FF),
                                            leftEye(leye),
                                            rightEye(reye),
                                            chestButton(cb),
                                            leftFoot(lf),
                                            rightFoot(rf) {}

      LED()
            : leftEar(0x3FF),
              rightEar(0x3FF),
              leftEye(rgb(true, true, true)),
              rightEye(rgb(true, true, true)),
              chestButton(rgb(true, true, true)),
              leftFoot(rgb()),
              rightFoot(rgb()) {}

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & leftEar;
         ar & rightEar;
         ar & leftEye;
         ar & rightEye;
         ar & chestButton;
         ar & leftFoot;
         ar & rightFoot;
      }
   };

   enum Stiffen
   {
      NONE = 0,
      STIFFEN
   };


/**
 * Wrapper for the other action commands, makes it easier to pass them around
 */
   struct All {
      Head head;
      Body body;
      LED leds;
      Stiffen stiffen;

      All() : head(), body(Body::NONE), leds(), stiffen(NONE)
      { }

      All(Head h, Body b, LED l, float s, Stiffen stf) {
         head = h;
         body = b;
         leds = l;
         stiffen = stf;
      }

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & head;
         ar & body;
         ar & leds;
      }
   };

//  These classes support stream output for debugging
   static inline bool operator==(const rgb &a, const rgb &b) {
      return (a.red == b.red) && (a.green == b.green) && (a.blue == b.blue);
   }

   static inline bool operator!=(const rgb &a, const rgb &b) {
      return !(a == b);
   }

   static inline std::ostream & operator<<(std::ostream &out, const rgb &a) {
      out << '{' << a.red << ", " << a.green << ", " << a.blue << '}';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const Head &a) {
      out << '[' << a.yaw << ", " << a.pitch << ", " << a.isRelative << ']';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const Body &a) {
      out << '[' << a.actionType << ", " << a.forward << ", " << a.left
      << ", " << a.turn << "," << a.power << ']';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const LED &a) {
      out << '[' << a.leftEar << ", " << a.rightEar << ", " << a.leftEye << ", "
      << a.rightEye << "," << a.chestButton << ","
      << a.leftFoot << "," << a.rightFoot << ']';
      return out;
   }
};  // namespace ActionCommand
