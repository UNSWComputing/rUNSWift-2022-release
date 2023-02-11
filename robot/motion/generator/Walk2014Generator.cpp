/**
 * Walk2014Generator.cpp
 * BH 18th Jan 2014
 * The period of each foot-step is set by T. T generates the forcing function by alternatively lifting each foot
 * Control:
 * The change of support foot is driven by the ZMP switching sign, but must be > T/2. If > 3*T we switch to try revive
 * The front-back sway is controlled by ankle tilts proportional to the GyroY
 * The user specifies forward(m), left(m), turn(radians) to activate the walk.
 * If these values are all zero the robot stands with the motors turned off.
 * The CoM is moved forward when walking to position it more over the center of the foot
 */

#include "motion/generator/Walk2014Generator.hpp"
#include <cmath>
#include <cstdio>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/speech.hpp"
#include <algorithm>
#include "../MotionDefs.hpp"
#include "utils/OptionConstants.hpp"

//#define WRITE_LEG_JOINT_TEMP_TO_FILE 1;

#define KICK_STEP_HEIGHT 0.065  // how far to lift kicking foot
#define EPSILON 0.01       //10 mm
#define TURN_EPSILON 0.05  //2.8 degrees

#define MAX_EXTRA_LEAN 1  // degrees
#define KICK_MIN_Y_DIST 30 //mm
#define KICK_MAX_Y_DIST 200 //mm

#define DEFAULT_ARM_STIFFNESS 0.1 // stiffness of arms joints during walk on default

#define BIG_NUM 1000000000000.0

using boost::program_options::variables_map;
using namespace std;
using namespace Joints;
using namespace Sensors;

const float MM_PER_M = 1000.0;             // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;              // time in seconds to crouch
const float COM_OFFSET_CROUCH = 0.01; // center of mass offset in x direction in meters when crouched, so that the CoM lies at the centre of the heel and toe
const float COM_OFFSET_FORWARDS = 0.022;   // center of mass offset in x direction in meters when walking forwards
const float COM_OFFSET_BACKWARDS = 0.01;   // center of mass offset in x direction in meters when walking backwards
const float FORWARD_CHANGE = 0.06; // max change of 80mm/sec at each leg change
const float LEFT_CHANGE = 0.1; // max change of 100mm/sec at each leg change
const float TURN_CHANGE = 1.0; // max change of 1.0rad/sec at each leg change (only when forward < MAX_FORWARD_TURN_CHANGE_SLOW)
const float TURN_CHANGE_SLOW = 0.5;     // when forward > MAX_FORWARD_RESTRICT_TURN_CHANGE
const float MAX_FORWARD_TURN_CHANGE_SLOW = 0.1; // When forward is greater than this value lower TURN_CHANGE to TURN_CHANGE_STABLE
const float STAND_HIP_HEIGHT = 0.248; // for tall power saving stand, matches INITIAL action command
const float KNEE_PITCH_RANGE = DEG2RAD(60); // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .25;    //.23  - .25            // seconds to walk one step
const float WALK_HIP_HEIGHT = .23; // Walk hip height - seems to work from .2 to .235
// simulated robot falls over at higher walk speeds (TODO try to debug this problem)
const float MAX_FORWARD = .3;                              // meters
const float MAX_LEFT = .2;                                 // meters
const float MAX_TURN = 2.0;                                // radians
const float BASE_LEG_LIFT = 0.012;                         // meters
const float MAX_LEFT_BLOCKING = .4;                        // meters
const float z = 0;

float evaluateWalkVolume(float x, float y, float z);

Walk2014Generator::Walk2014Generator()
    : t(0.0f), weightHasShifted(false)
{
    initialise();
    llog(INFO) << "Walk2014Generator constructed" << std::endl;
}

Walk2014Generator::~Walk2014Generator() {
    llog(INFO) << "Walk2014Generator destroyed" << std::endl;
}

void Walk2014Generator::initialise() {
    llog(INFO) << "Walk2014 initializing" << endl;
    dt = MOTION_DT;
    t = 0.0;                                   // initialise timers (in seconds)
    timer = 0.0;                            // timer to crouch to walking height
    globalTime = 0;                          // use for diagnostic purposes only
    T = BASE_WALK_PERIOD; // seconds - the period of one step
    stopping = false;                         // legacy code for stopping robot?
    stopped = true;                            // legacy code for stopped robot?
    leftL = leftR = lastLeft = left = 0.0; // Side-step for left, right foot, and (last) left command in meters
    leftL0 = leftR0 = 0.0;
    turnRL = turnRL0 = lastTurn = turn = 0.0;       // Initial turn variables for feet
    forwardL = forwardR = 0.0; // forward step per for left and right foot in meters
    forwardR0 = forwardL0 = 0; // initial last positions for left and right feet keep constant during next walk step
    forward = lastForward = 0.0;           // Current and previous forward value
    shoulderPitchL = shoulderPitchR = 0;              // arm swing while walking
    shoulderRollL = shoulderRollR = 0;                   // arm roll during kick
    hiph = hiph0 = STAND_HIP_HEIGHT; // make robot stand initially based on Stand command
    foothL = foothR = 0;         // robots feet are both on the ground initially
    thigh = Limbs::ThighLength / MM_PER_M;             // thigh length in meters
    tibia = Limbs::TibiaLength / MM_PER_M;             // tibia length in meters
    ankle = Limbs::FootHeight / MM_PER_M;        // height of ankle above ground
    nextFootSwitchT = 0.0; // next time-point to switch support foot (in seconds)
    stiffness = kneeStiffness = ankleStiffness = 0.9; // initial motor stiffness
    currentVolume = 0;                      // initial volume
    walk2014Option = NONE;                           // initial walk 2014 option
    walkState = NOT_WALKING;                               // initial walkState
    supportFoothasChanged = false;       // triggers support foot change actions
    comOffset = targetComOffset = comOffset0 = 0; // Center of Mass offset in sagittal plane used to spread weight along feet in x-dir
    prevTurn = prevForwardL = prevForwardR = 0;            // odometry
    prevLeftL = prevLeftR = 0;                             // odometry
    exactStepsRequested = false; // turns off ratcheting as requested by WalkEnginePreProcessor for kick
    shouldEmergencyStep = false; // Whether robot sohuld take an emergency step
    currentTurnChange = TURN_CHANGE;

    // Balance control
    filteredGyroX = 0;
    filteredGyroY = 0;
    filteredAngleY = 0;
    sagittalBalanceAdjustment = 0;
    coronalBalanceAdjustment = 0;
    sagittalHipBalanceAdjustment = 0;

    // Gyro PD controller
    preErrorGyro = 0;             // Previous tick gyro error

    // Angle PID controller
    angleError = 0;               // Current angle error
    angleErrorSum = 0;            // Error sum
    preErrorAngle = 0;            // Previous tick angle error

    // Kick specific
    kickT = 0;
    stableCounter = 0;
    rock = 0;
    kneePitchL = kneePitchR = lastKneePitch = 0;
    anklePitchL = anklePitchR = 0;
    lastKickForward = 0;
    lastSide = 0;
    lastKickTime = T;
    dynamicSide = 0.0f;
    turnAngle = 0;
    lastKickTurn = 0;
    motionOdometry.reset();
    kneePitchEnd = 0;
    anklePitchStart = 0;
    anklePitchEnd = 0;
    swingDelayFactor = 0;
    holdAnkle = false;
    hipBalance = false;
    ballXYDebug = false;

#ifdef CTC_2_1
    // Kick parameter constants
    shiftPeriod = 2.6;
    shiftEndPeriod = 2.5;
    backPhase = 0.70;
    kickPhase = 0.2;
    throughPhase = 0.3;
    endPhase = 0.0;
    shoulderRollAmpMultiplier = 2.5;
#else
    // Kick parameter constants
    shiftPeriod = 2.6;
    shiftEndPeriod = 2.5;
    backPhase = 0.70;
    kickPhase = 0.1;
    throughPhase = 0.3;
    endPhase = 0.0;
    shoulderRollAmpMultiplier = 2.5;
#endif
    // Blocking variables
    timerSinceLastBlock = 0;

}

float ellipsoidClampWalk(float &forward, float &left, float &turn, float speed) {
    const float MIN_SPEED = 0.0f;
    const float MAX_SPEED = 1.0f;
    speed = crop(speed, MIN_SPEED, MAX_SPEED);

    // limit max to 66-100% depending on speed
    float M_FORWARD = MAX_FORWARD * 0.66 + MAX_FORWARD * 0.34 * speed;
    float M_LEFT = MAX_LEFT * 0.66 + MAX_LEFT * 0.34 * speed;
    float M_TURN = MAX_TURN * 0.66 + MAX_TURN * 0.34 * speed;

    // Values in range [-1..1]
    float forwardAmount = forward / M_FORWARD;
    float leftAmount = left / M_LEFT;
    float turnAmount = turn / M_TURN;

    float x = fabs(forwardAmount);
    float y = fabs(leftAmount);
    float z = fabs(turnAmount);

    // see if the point we are given is already inside the allowed walk params volume
    if (evaluateWalkVolume(x, y, z) > 1.0) {
        float scale = 0.5;
        float high = 1.0;
        float low = 0.0;

        // This is basically a binary search to find the point on the surface.
        for (unsigned i = 0; i < 10; i++) {
            x = fabs(forwardAmount) * scale;
            y = fabs(leftAmount) * scale;
            z = fabs(turnAmount) * scale;

            if (evaluateWalkVolume(x, y, z) > 1.0) {
                float newScale = (scale + low) / 2.0;
                high = scale;
                scale = newScale;
            } else {
                float newScale = (scale + high) / 2.0;
                low = scale;
                scale = newScale;
            }
        }

        forwardAmount *= scale;
        leftAmount *= scale;
        turnAmount *= scale;
    }

    forward = M_FORWARD * forwardAmount;
    left = M_LEFT * leftAmount;
    turn = M_TURN * turnAmount;
    float volume = evaluateWalkVolume(x, y, z);
    return volume;
}

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z) {
    return sqrt(x * x + y * y + z * z);
}

// Stiffness required by the knee to keep stability
// Equation was found by finding the lowest stiffness that was stable at a continous requested speed
// Relate each speed to the corresponding volume and curve fit
float Walk2014Generator::calculateKneeStiffness(float volume) {
    float stiffness = -0.42 * (volume * volume) + volume + 0.395;
    stiffness = ceilf(MIN(1,MAX(0.4,stiffness)) * 100) / 100;    // Min and max (2 dp) required stiffness
    return stiffness;
}
// Stiffness required by the ankle to keep stability
// Equation was found by setting the lowest stiffness that was stable at a continous requested speed
// Relate each speed to the corresponding volume and curve fit
float Walk2014Generator::calculateAnkleStiffness(float volume) {
    float stiffness = -0.32 * (volume * volume) + 0.76 * volume + 0.56;
    stiffness = ceilf(MIN(1,MAX(0.55,stiffness)) * 100) / 100;    // Min and max (2 dp) required stiffness
    return stiffness;
}

JointValues Walk2014Generator::makeJoints(ActionCommand::All* request, Odometry* odometry, const SensorValues &sensors,
                                       BodyModel &bodyModel, float ballX, float ballY, MotionDebugInfo &motionDebugInfo)
{
    // 0. The very first time walk is called, the previous stand height could have been anything, so make sure we interpolate from that
    if (walk2014Option == NONE) {
        // Calculate the current hip height by checking how bent the knee is
        hiph = MAX(sensors.joints.angles[LKneePitch], 0) / KNEE_PITCH_RANGE * (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) + STAND_HIP_HEIGHT;
    }

    // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0
    if (t == 0) {
        if (shouldEmergencyStep)
        {
            // Try and move sideways to free ourselves when we're stuck on a goal post or another robot
            forward = 0;
            left = 300;
            turn = 0;
            power = 1;
        }
        else
        {
            active = request->body;
            forward = (float) active.forward / MM_PER_M;    // in meters
            left = (float) active.left / MM_PER_M;       // in meters
            turn = active.turn;                       // in radians
            power = active.power;                     // controls stiffness when standing and kicking when walking*
            bend = active.bend;                       // knee-bend parameter
            speed = active.speed;                     // controls speed of walk, distinguishes Jab kick
            foot = active.foot;                       // kicking foot
            useShuffle = active.useShuffle;
            if (stopping) {                                   // not used at present
            } else {
                stopped = false;                                  // (re)activate
            }

            if (forward == 0 and left == 0 and turn == 0 and power == 0)
                bend = 0;

            // limit the speed of walk when overheating, can be commented out for serious games
//            for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
//                float temp = sensors.joints.temperatures[i];
//                if (temp > 70)
//                    speed = min(0.5f, speed);
//                if (temp > 75)
//                    speed = 0;
//            }
        }

#ifdef WRITE_LEG_JOINT_TEMP_TO_FILE
        // Record the temperatures of the leg joints ever 5 seconds in the file temp.txt
        static int counter = 0;
        if(counter == 500){
            FILE *file = fopen("/home/nao/data/temp.txt", "a");
            for (int i = 8; i < 19; ++i) {                          //Only want leg joints
                float temp = sensors.joints.temperatures[i];
                fprintf(file, "%lf ", temp);
                //std::cout << " " << i << " - " << temp;
            }
            fprintf(file, "\n");
            //std::cout << "\n";
            fclose(file);
            counter = 0;
        }
        counter++;
#endif //WRITE_LEG_JOINT_TEMP_TO_FILE

        // Stabilise for a bit of time after a block by walking on the spot
        if (active.blocking)
        {
            if (timerSinceLastBlock < 0.8)
            {
                // Should stabilise after a block
                active.blocking = false;
                forward = 0;
                left = 0;
                turn = 0;
            } else {
                // perform a block
                timerSinceLastBlock = 0;
            }
        }

        // Scale back values to try to ensure stability.
        if (!active.blocking && !exactStepsRequested) {
            currentVolume = ellipsoidClampWalk(forward, left, turn, speed);
        }

        // Restrict turn change when forwards is greater than MAX_FORWARD_TURN_CHANGE_SLOW
        if (forward > MAX_FORWARD_TURN_CHANGE_SLOW) {
            currentTurnChange = TURN_CHANGE_SLOW;
        } else {
            currentTurnChange = TURN_CHANGE;
        }

        float f = forward * MM_PER_M;
        float l = left * MM_PER_M;
        forward = f / MM_PER_M;
        left = l / MM_PER_M;

        // Modify T when sidestepping
        if (active.blocking)
            T = BASE_WALK_PERIOD;
        else
            T = BASE_WALK_PERIOD + 0.05 * abs(left) / MAX_LEFT;

        // If we're either walking, or ready to walk, and we have a non zero reqeust, then rachet
        if ((walk2014Option == WALK || walk2014Option == READY) && !(forward == 0 && left == 0 && turn == 0))
            {
            // ratchet forward by FORWARD_CHANGE
            if (!exactStepsRequested && !active.blocking) {
                if (abs(forward - lastForward) > FORWARD_CHANGE) {
                    forward = lastForward + (forward - lastForward) / abs(forward - lastForward) * FORWARD_CHANGE;
                }
                if (abs(left - lastLeft) > LEFT_CHANGE) {
                    left = lastLeft + (left - lastLeft) / abs(left - lastLeft) * LEFT_CHANGE;
                }
                if (abs(turn - lastTurn) > currentTurnChange) {
                    turn = lastTurn + (turn - lastTurn) / abs(turn - lastTurn) * currentTurnChange;
                }
            }
        }
        else
        {
            // set forward, left and turn to 0 if we're not walking. ensure feet
            /// are actually together, before crouching, by checking the LHipRoll
            // joitns.
            if (abs(sensors.joints.angles[Joints::LHipRoll]) > DEG2RAD(1.0)||
                abs(sensors.joints.angles[Joints::RHipRoll]) > DEG2RAD(1.0))
            {
                // make sure feet are back together, by walking on the spot
                forward = 0.01;
                left = 0;
                turn = 0;
            }
            else
            {
                forward = 0;
                left = 0;
                turn = 0;
            }
        }

        // If we've detected a big gyroscopeZ, slow down! This was added for the longer
        // carpet in sydney and prevents falling over by not keep walking
        if (abs(turn) < 0.001)
        {
            if (abs(sensors.sensors[Sensors::InertialSensor_GyroscopeZ]) > 0.9)
            {
                forward /= 3;
            }
        }

        lastForward = forward; // back up old value in m/s
        lastLeft = left; // used to detect when the left walk parameter changes sign
        lastTurn = turn;

        // 1.6 Walk Calibration
        // The definition of forward, left and turn is the actual distance/angle traveled in one second
        // One walk-cycle consists of two Phases, a left phase (left swing foot) and a right phase (right swing foot)

        if (!exactStepsRequested) {
            forward *= T; // theoretical calibration, how much to move each foot per step
            left *= T;
            turn *= T;
            // linear calibration to achieve actual performance ie turn in action command achieves turn/sec in radians on the robot
            forward *= 1.0;
            left *= 0.82;
            turn *= 0.78;
        }
        turn *= -1;   // reverses sign
    }

    // 2. Update timer
    t += dt;
    globalTime += dt;
    lastKickTime += dt;
    timerSinceLastBlock += dt;

    // 3. Determine Walk2014 Option
    if (active.actionType == ActionCommand::Body::KICK && request->body.actionType != ActionCommand::Body::KICK) {
        if (canAbortKickBeforeLean())
        {
            kickT = backPhase + kickPhase + throughPhase + endPhase + shiftEndPeriod / 4;
            motionDebugInfo.kickAborted = true;
        }
        else if (canAbortKick())
        {
            // Finish transition out if in the middle of a kick by skipping to the end phase
            kickT = backPhase + kickPhase + throughPhase;
            motionDebugInfo.kickAborted = true;
        }
    } else if (active.actionType == ActionCommand::Body::KICK) {
        // This makes sure that the action type gets set back to walk just after a kick is finished.
        // If we don't leave enough time for this to happen, motion moves back into a kick before behaviour
        // can change its mind.
        if (lastKickTime < 4 * T) {
            request->body.actionType = ActionCommand::Body::WALK;
        } else if (abs(hiph - WALK_HIP_HEIGHT) < .0001) { // make sure we retain the designated walking height
            if (walk2014Option == WALK) {
                // make sure walk is in neutral stance before kicking, L is symmetrical to R
                if (fabs(forwardL) < EPSILON && fabs(leftL) < EPSILON && fabs(turnRL) < TURN_EPSILON && t == dt) {
                    // Assuming already at t = 0 from active getting set to kick
                    // Any new settings the first time walk2014Option==KICK go here

                    prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
                }
            } else if (walk2014Option != KICK) {
                prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
            }
        } else {                                      // hiph not= walkHipHeight
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }

    } // end Kick test
    else if (walk2014Option == WALK and walkState != NOT_WALKING) { // we are in the process of walking
        if (forward == 0 and left == 0 and turn == 0) { // request to stop walking
            walkState = STOPPING;
        }
    } else if (bend == 0) { // if we are not walking and wish to stand and power off
        if (abs(hiph - STAND_HIP_HEIGHT) < .0001) { // and robot has reached stand height
            walk2014Option = STAND;                       // (keep) standing
        } else {                                  // if hiph not= standHipHeight
            if (walk2014Option != STANDUP) {
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = STANDUP;                     // stand up first
        }
    } else if (forward == 0 and left == 0 and turn == 0 and bend == 1) { // not walking, but ready to go again, ie don't stand up
        if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
            walk2014Option = READY;
        } else {
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }
    } else {                             // if some walk parameters are non-zero
        if (abs(hiph - WALK_HIP_HEIGHT) < .0001) { // and we are at the designated walking height
            if (walk2014Option != WALK) {
                // Any new settings the first time walk2014Option==WALK go here (just for testing the walk)
                nextFootSwitchT = T;

                if (active.blocking)
                {
                    walkState = WALKING;
                    // Start walking with the foot in the direction we want to go to, if we're blocking
                    bodyModel.setIsLeftPhase(left > 0);
                } else {
                    walkState = STARTING;
                    // Start walking with the foot that's not in the direction we want to go to.
                    // Since the first step the robot takes  is a "STARTING" step, it can only take a side
                    // step from the second step onwards.
                    bodyModel.setIsLeftPhase(left < 0);
                }
            }
            walk2014Option = WALK;                          // (keep) walking
        } else {                                      // hiph not= walkHipHeight
            if (walk2014Option != CROUCH) { // robot starts crouching to walking height
                hiph0 = hiph;
                timer = 0;
            }
            walk2014Option = CROUCH;                       // continue crouching
        }
    }

    // 4. Execute Walk2014 Option
    if (walk2014Option == STAND) { // Place CoM over ankle and turn set power to motors
        hiph = STAND_HIP_HEIGHT;
        forward = left = turn = 0;
        t = nextFootSwitchT = 0;
        stiffness = power;
        if (stiffness < 0.2)
            stiffness = 0.2;
        comOffset0 = comOffset = 0;
    } else if (walk2014Option == STANDUP) {
        hiph = hiph0 + (STAND_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
        forward = left = turn = 0;
        comOffset0 = comOffset -= 0.02 * comOffset; // reduce offset to zero to allow stiffness to be turned down
        stiffness = 1;
        t = nextFootSwitchT = 0;
        timer += dt;
    } else if (walk2014Option == CROUCH) {
        forward = left = turn = 0;
        stiffness = 1;
        hiph = hiph0 + (WALK_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
        comOffset0 = comOffset = COM_OFFSET_CROUCH * parabolicStep(timer, CROUCH_STAND_PERIOD, 0); // move comOffset to 0.01 meters when walking
        t = nextFootSwitchT = 0;
        timer += dt;                                        // inc. option timer
    } else if (walk2014Option == WALK) {
        if(!exactStepsRequested){
            // Set knee and ankle stiffness to 1, (if overheating happens, consider reintroducing variable stiffness through the commened out code below)
            kneeStiffness = 1;  // kneeStiffness = calculateKneeStiffness(currentVolume);
            ankleStiffness = 1;  // ankleStiffness = calculateAnkleStiffness(currentVolume);
        }
        // If moving forward or backward scale comOffset to a maximum and minimum position
        if (lastForward == 0) {
            targetComOffset = COM_OFFSET_CROUCH;
        } else if (lastForward > 0) {
            targetComOffset = COM_OFFSET_BACKWARDS + ((abs(lastForward) / MAX_FORWARD) * (COM_OFFSET_FORWARDS - COM_OFFSET_BACKWARDS));
        } else {
            targetComOffset = COM_OFFSET_BACKWARDS;
        }
        comOffset = comOffset0 + (targetComOffset - comOffset0) * linearStep(t, T);
        stiffness = 1;
    } else if (walk2014Option == KICK) {
        stiffness = 1;
        forward = left = turn = 0;
    }
    if (walk2014Option == READY) {
        forward = left = turn = 0;
        stiffness = power;
        if (stiffness < 0.4)
            stiffness = 0.4;  // need enough stiffness to keep crouching posture
        t = nextFootSwitchT = 0;
        comOffset0 = comOffset;
    }
    // 5. Determine walk variables throughout the walk step phase
    if (walk2014Option == WALK and nextFootSwitchT > 0) {
        // 5.1 Calculate the height to lift each swing foot
        float maxFootHeight = BASE_LEG_LIFT + abs(forward) * 0.01 + abs(left) * 0.03;
        if (useShuffle){
            maxFootHeight *= 0.7;  // lower step height when shuffling (ie. close to obstacles)
        }
        float varfootHeight = maxFootHeight * parabolicReturnMod(t / nextFootSwitchT); // 0.012 lift of swing foot from ground
        // 5.2 When walking in an arc, the outside foot needs to travel further than the inside one - void
        // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt, for left swing foot
        if (bodyModel.isLeftPhase) {             // if the support foot is right
            if (weightHasShifted) {
                // 5.3.1L forward (the / by 2 is because the CoM moves as well and forwardL is wrt the CoM
                forwardR = forwardR0 + ((forward) / 2 - forwardR0) * linearStep(t, nextFootSwitchT);
                forwardL = forwardL0 + parabolicStep(t, nextFootSwitchT, 0) * (-(forward) / 2 - forwardL0); // swing-foot follow-through
                // 5.3.2L Jab kick with left foot - removed
                // 5.3.3L Determine how much to lean from side to side - removed
                // 5.3.4L left
                if (left > 0) {
                    leftL = leftL0 + (left - leftL0) * parabolicStep(t, nextFootSwitchT, 0.2);
                    leftR = -leftL;
                } else {
                    leftL = leftL0 * (1 - parabolicStep(t, nextFootSwitchT, 0.0));
                    leftR = -leftL;
                }
                // 5.3.5L turn (note, we achieve correct turn by splitting turn foot placement unevely over two steps, but 1.6 + 0.4 = 2 and adds up to two steps worth of turn)
                if (turn < 0)
                    turnRL = turnRL0 + (-1.6 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0);
                else
                    turnRL = turnRL0 + (-0.4 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0); //turn back to restore previous turn angle
            }
            // 5.3.6L determine how high to lift the swing foot off the ground
            foothL = varfootHeight;                      // lift left swing foot
            foothR = 0;                             // do not lift support foot;
        }
        // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
        if (not bodyModel.isLeftPhase) {          // if the support foot is left
            if (weightHasShifted) {
                // 5.3.1R forward
                forwardL = forwardL0 + ((forward) / 2 - forwardL0) * linearStep(t, nextFootSwitchT);
                forwardR = forwardR0 + parabolicStep(t, nextFootSwitchT, 0) * (-(forward) / 2 - forwardR0); // swing-foot follow-through
                // 5.3.2R Jab-Kick with right foot - removed
                // 5.3.3R lean - not used
                // 5.3.4R left
                if (left < 0) {
                    leftR = leftR0 + (left - leftR0) * parabolicStep(t, nextFootSwitchT, 0.2);
                    leftL = -leftR;
                } else {
                    leftR = leftR0 * (1 - parabolicStep(t, nextFootSwitchT, 0.0));
                    leftL = -leftR;
                }
                // 5.3.5R turn
                if (turn < 0)
                    turnRL = turnRL0 + (0.4 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0); //turn back to restore previous turn angle
                else
                    turnRL = turnRL0 + (1.6 * turn - turnRL0) * parabolicStep(t, nextFootSwitchT, 0.0);
                // 5.3.6R Foot height
            }
            foothR = varfootHeight;
            foothL = 0;
        }
        // 5.4 Special conditions when priming the walk
        if (walkState == STARTING) {
            turnRL = 0; // don't turn on start of rocking - may relax this in future
            foothL /= 3.5; // reduce max lift due to short duration - may need to adjust this later
            foothR /= 3.5;                                  // "
            leftR = leftL = 0; // don't try to step left on starting and stopping
            forwardL = forwardR = 0;           // don't move forward or backward
            leftL = leftR = 0;
            lastForward = 0.0;
            lastTurn = 0.0;
        }
        // 5.5 "natural" arm swing while walking/kicking to counterbalance foot swing
        shoulderPitchR = -forwardR * 6; //10;                     // forwardR is in meters, 10 is an arbitrary scale factor to match previous walk
        shoulderPitchL = -forwardL * 6;                        //10;
    } else if (walk2014Option == KICK) { // Kicking

        ballX = clamp(160, ballX, 250);

        float kickHeight = KICK_STEP_HEIGHT;

        if(ballX > 200) {
            kickHeight *= 1.125;
        }

        if (active.foot == ActionCommand::Body::LEFT) {
            // Each robot has a slightly different lean angle that works. This is dealt through individual lean offsets.
            float kickLeanMod = kickLean + ((ballY-KICK_MIN_Y_DIST)/(KICK_MAX_Y_DIST-KICK_MIN_Y_DIST))*MAX_EXTRA_LEAN;

            // Clever Nao kick lean.
            if(motionDebugInfo.overwriteKickLean)
                kickLeanMod += motionDebugInfo.kickLeanOverwrite;

            else
            {
                // Varied kick lean.
                if(kickLeanOffsetLVaried)
                {
                    if(ballY < 40)
                        kickLeanMod += kickLeanOffsetLInner;
                    else if(ballY > 80)
                        kickLeanMod += kickLeanOffsetLOuter;
                    else
                        kickLeanMod += kickLeanOffsetLMid;
                }

                // General kick lean.
                else
                    kickLeanMod += kickLeanOffsetL;
            }

            ballY = MIN(150, MAX(ballY, 30));
            float gyroXOntoSupportFoot = sensors.sensors[InertialSensor_GyroscopeX];
            makeForwardKickJoints(kickLeanMod, kickHeight, foothL, forwardL, leftL, kneePitchL, shoulderRollL,
                                    anklePitchL, ballX, ballY, request, sensors, gyroXOntoSupportFoot, motionDebugInfo);
        } else { // with added adjustments for right side
            float kickLeanMod = -1*(kickLean +
                                           ((ballY+KICK_MIN_Y_DIST)/(-KICK_MAX_Y_DIST+KICK_MIN_Y_DIST))*MAX_EXTRA_LEAN);

            // Clever Nao kick lean.
            if(motionDebugInfo.overwriteKickLean)
                kickLeanMod -= motionDebugInfo.kickLeanOverwrite;

            else
            {
                // Varied kick lean.
                if(kickLeanOffsetRVaried)
                {
                    if(ballY < 25)
                        kickLeanMod -= kickLeanOffsetRInner;
                    else if(ballY > 75)
                        kickLeanMod -= kickLeanOffsetROuter;
                    else
                        kickLeanMod -= kickLeanOffsetRMid;
                }

                // General kick lean.
                else
                    kickLeanMod -= kickLeanOffsetR;
            }

            ballY = MAX(-150, MIN(ballY, -30));
            ballY *= -1;
            float gyroXOntoSupportFoot = -sensors.sensors[InertialSensor_GyroscopeX];
            makeForwardKickJoints(kickLeanMod, kickHeight, foothR, forwardR, leftR, kneePitchR, shoulderRollR,
                                    anklePitchR, ballX, ballY, request, sensors, gyroXOntoSupportFoot, motionDebugInfo);
            leftR *= -1;
        }
    } else { // When walk option is not WALK or KICK
        foothL = foothR = 0;
    }

    // 6. Changing Support Foot. Note bodyModel.isLeftPhase means left foot is swing foot.
    //    t>0.75*T tries to avoid bounce, especially when side-stepping
    //    lastZMPL*ZMPL<0.0 indicates that support foot has changed
    //    t>3*T tries to get out of "stuck" situations

    if (t > 0.50 * T and bodyModel.ZMPL * bodyModel.lastZMPL < 0 and walk2014Option == WALK)
    {
        supportFoothasChanged = true;
        shouldEmergencyStep = false; // we don't have to do an emergency step any more once we detect this condition
    }
    if (lastKickTime == 0)
    {
        // If we just finished the kick, change supportFoot so we start walking
        supportFoothasChanged = true;
    }
    if (supportFoothasChanged) {
        weightHasShifted = (bodyModel.isLeftPhase != (bodyModel.ZMPL < 0));
        bodyModel.setIsLeftPhase(bodyModel.ZMPL < 0); // set isLeft phase in body model for kinematics etc
    }
    if (t > 3 * T && walk2014Option != KICK && lastKickTime > 3.0)
    {
        // Trick the walk gneerator into thinking that we've finished the step, so we can
        // get unstuck when we've walked into a post or another robot
        supportFoothasChanged = true;
        weightHasShifted = true;
        bodyModel.setIsLeftPhase(!bodyModel.isLeftPhase);
        shouldEmergencyStep = true;
    }
    if (supportFoothasChanged and walk2014Option == WALK) {
        supportFoothasChanged = false;                      //reset
        // 6.1 Recover previous "left" swing angle -> changed to leftL and leftR
        // 6.2 Decide on timing of next walk step phase
        if (walkState == NOT_WALKING) {                       // Start the walk
            nextFootSwitchT = T;
            if (active.blocking)
                walkState = WALKING; // we go straight to WALKING walkState (needed for blocking moving balls)
            else
                walkState = STARTING;
        } else if (walkState == STARTING) {
            nextFootSwitchT = T;
            walkState = WALKING;
        } else if (walkState == WALKING) {
            nextFootSwitchT = T;
            walkState = WALKING; // continue walking until interrupted by a command to stop (or kick)
        } else if (walkState == STOPPING) {
            nextFootSwitchT = T;
            walkState = NOT_WALKING;
        } else
            llog(FATAL) << "Should never get here: walkState error" << endl;
        // 6.3 reset step phase time
        t = 0;                                         // reset step phase timer
        // 6.4 backup values
        turnRL0 = turnRL;               // store turn value for use in next step
        forwardR0 = forwardR;                             // sagittal right foot
        forwardL0 = forwardL;                              // sagittal left foot
        leftL0 = leftL;
        leftR0 = leftR;
        comOffset0 = comOffset;
        // 6.5 Other stuff on support foot change - none at the moment
    } // end of changing support foot

    // If we're in crouch, we want to be ready to pounce!
    if (walk2014Option == READY)
    {
        t = 0;
    }

    // 7. Sagittal Balance

    // PD Controller tuned using the Ziegler-Nichols method
    filteredGyroY = 0.8 * filteredGyroY + 0.2 * sensors.sensors[InertialSensor_GyroscopeY];
    // Calculate total output
    //    Control output     =    Kp   x     error     +    Kd   x (          deltaError          / dt)
    sagittalBalanceAdjustment =  KpGyro * filteredGyroY + (KdGyro * (filteredGyroY - preErrorGyro) / dt);

    // Save current error to be next previous error
    preErrorGyro = filteredGyroY;

    // PID Controller hand tuned by inspection
    filteredAngleY = 0.8 * filteredAngleY + 0.2 * sensors.sensors[InertialSensor_AngleY];

    // Calculate error only outside angle thresholds
    if (filteredAngleY > 0.06) {
        angleError = 0.06 - filteredAngleY;
    }
    else if (filteredAngleY < -0.09) {
        angleError = -0.09 - filteredAngleY;
    }
    // Otherwise clear the error and error sum
    else {
        angleErrorSum = 0;
        angleError = 0;
    }

    // Add the error to the error sum
    angleErrorSum += angleError * dt;

    // Calculate total output (negated to adjust hip angle correctally)
    //     Control output       =   Kp    x     error  +    Ki   x    errorSum   + (  Kd   *  (          deltaError          / dt)
    sagittalHipBalanceAdjustment = -(KpAngle * angleError + KiAngle * angleErrorSum + (KdAngle * (angleError - preErrorAngle) / dt));

    //Save error to previous error
    preErrorAngle = angleError;

    if (walk2014Option == READY) {
        sagittalBalanceAdjustment = 0;        // to stop swaying while not walking
    }

    // 7.5 Coronal Balance
    filteredGyroX = 0.8 * filteredGyroX + 0.2 * sensors.sensors[InertialSensor_GyroscopeX];
    if (walk2014Option == KICK) {
        coronalBalanceAdjustment = filteredGyroX / 4.5; // adjust ankle roll in proportion to filtered gryoX aggressively during kick
    } else {
        coronalBalanceAdjustment = 0;
    }

    // 8. Odometry update for localisation
    *odometry = *odometry + motionOdometry.updateOdometry(sensors, updateOdometry(bodyModel.isLeftPhase));

    // 9. Work out joint angles from walk variables above

    // 9.05 Sert hip and ankle values
    float HrL = atan(leftL / (hiph - ankle));
    float HrR = atan(leftR / (hiph - ankle));
    float ArL = -HrL;
    float ArR = -HrR;

    // 9.1 Left foot closed form inverse kinematics
    float leghL = hiph - foothL - ankle; // vertical height between ankle and hip in meters
    float legX0L = leghL / cos(HrL); // leg extension (eliminating knee) when forwardL = 0
    float legXL = sqrt(legX0L * legX0L + (forwardL + comOffset) * (forwardL + comOffset)); //leg extension at forwardL
    if (legXL > thigh + tibia)
        std::cout << "(Walk2014Generator) cannot solve Inverse Kinematics for that foot position!" << std::endl;
    float beta1L = acos((thigh * thigh + legXL * legXL - tibia * tibia) / (2.0f * thigh * legXL)); // acute angle at hip in thigh-tibia triangle
    float beta2L = acos((tibia * tibia + legXL * legXL - thigh * thigh) / (2.0f * tibia * legXL)); // acute angle at ankle in thigh-tibia triangle
    float tempL = legX0L / legXL;
    if (tempL > 1.0f)
        tempL = 1.0f; // sin ratio to calculate leg extension pitch. If > 1 due to numerical error round down.
    float deltaL = asin(tempL);                           // leg extension angle
    float dirL = 1.0f;
    if ((forwardL + comOffset) > 0.0f)
        dirL = -1.0f; // signum of position of foot
    float HpL = beta1L + dirL * (M_PI / 2.0f - deltaL); // Hip pitch is sum of leg-extension + hip acute angle above
    float ApL = beta2L + dirL * (deltaL - M_PI / 2.0f); // Ankle pitch is a similar calculation for the ankle joint
    float KpL = HpL + ApL; // to keep torso upright with both feet on the ground, the knee pitch is always the sum of the hip pitch and the ankle pitch.

    // 9.2 right foot closed form inverse kinematics (comments as above but for right leg)
    float leghR = hiph - foothR - ankle;
    float legX0R = leghR / cos(HrR);
    float legXR = sqrt(legX0R * legX0R + (forwardR + comOffset) * (forwardR + comOffset));
    float dirR = 1.0f;
    if ((forwardR + comOffset) > 0.0f)
        dirR = -1.0f;
    if (legXR > thigh + tibia)
        std::cout << "(Walk2014Generator) cannot solve Inverse Kinematics for that foot position!" << std::endl;
    float beta1R = acos((thigh * thigh + legXR * legXR - tibia * tibia) / (2.0f * thigh * legXR));
    float beta2R = acos((tibia * tibia + legXR * legXR - thigh * thigh) / (2.0f * tibia * legXR));
    float tempR = legX0R / legXR;
    if (tempR > 1.0f)
        tempR = 1.0f;
    float deltaR = asin(tempR);
    float HpR = beta1R + dirR * (M_PI / 2.0f - deltaR);
    float ApR = beta2R + dirR * (deltaR - M_PI / 2.0f);
    float KpR = HpR + ApR;

    // 9.3 Sert hip and ankle values -> Moved to 9.05

    // 9.35 Add kick rocking
    if (walk2014Option == KICK) {
        HrL += rock;
        HrR += rock;
        ArL -= rock;
        ArR -= rock;
    }

    // 9.4 Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
    // Turning
    XYZ_Coord tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
    XYZ_Coord sL;
    float Hyp = -turnRL;
    for (int i = 0; i < 3; i++) {
        sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
        XYZ_Coord e((tL.x - sL.x), (tL.y - sL.y), (tL.z - sL.z));
        Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
        HpL -= hpr.Hp;
        HrL += hpr.Hr;
    }
    // ApL and ArL to make sure LEFT foot is parallel to ground
    XYZ_Coord up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
    XYZ_Coord ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
    ApL = ApL + asin(sL.z - up.z);
    ArL = ArL + asin(sL.z - ur.z);

    // 9.5 Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
    // Map to LEFT - we reuse the left foot IK because of symmetry right foot
    float Hr = -HrR;
    float Ar = -ArR;
    // Target foot origin in body coords
    XYZ_Coord t = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
    XYZ_Coord s;
    Hyp = -turnRL;
    for (int i = 0; i < 3; i++) {
        s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
        XYZ_Coord e((t.x - s.x), (t.y - s.y), (t.z - s.z));
        Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
        HpR -= hpr.Hp;
        Hr += hpr.Hr;
    }
    // 9.6 Ap and Ar to make sure foot is parallel to ground
    XYZ_Coord u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
    XYZ_Coord u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
    ApR = ApR + asin(s.z - u1.z);
    Ar = Ar + asin(s.z - u2.z);
    // map back from left foot to right foot
    HrR = -Hr;
    ArR = -Ar;

    // 10. Set joint values and stiffness
    JointValues j = sensors.joints;
    for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
        j.stiffnesses[i] = stiffness;

    // Overwrite knee and ankle pitch stiffness to reduce overheating
    if (walk2014Option == WALK && !exactStepsRequested) {
        j.stiffnesses[Joints::LKneePitch] = kneeStiffness;
        j.stiffnesses[Joints::RKneePitch] = kneeStiffness;
        j.stiffnesses[Joints::LAnklePitch] = ankleStiffness;
        j.stiffnesses[Joints::RAnklePitch] = ankleStiffness;
    }

    // 10.1 Arms

    // Lower stiffness as much as possible, to prevent overheating in arms (important for reliable getups)
    j.stiffnesses[Joints::LShoulderPitch] = 0.3;
    j.stiffnesses[Joints::LShoulderRoll] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::LElbowYaw] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::LElbowRoll] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::LWristYaw] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::RShoulderPitch] = 0.3;
    j.stiffnesses[Joints::RShoulderRoll] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::RElbowYaw] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::RElbowRoll] = DEFAULT_ARM_STIFFNESS;
    j.stiffnesses[Joints::RWristYaw] = DEFAULT_ARM_STIFFNESS;

    j.angles[LShoulderPitch] = DEG2RAD(90) + shoulderPitchL;
    j.angles[LShoulderRoll] = DEG2RAD(7) + shoulderRollL;
    j.angles[LElbowRoll] = DEG2RAD(0); //DEG2RAD(-30)+shoulderPitchL;  //swing bent arms
    j.angles[LElbowYaw] = DEG2RAD(0);
    j.angles[LWristYaw] = DEG2RAD(0);
    j.angles[RShoulderPitch] = DEG2RAD(90) + shoulderPitchR;
    j.angles[RShoulderRoll] = DEG2RAD(-7) - shoulderRollR;
    j.angles[RElbowRoll] = DEG2RAD(0); //DEG2RAD(30)-shoulderPitchR; //swing bent arms
    j.angles[RElbowYaw] = DEG2RAD(0);
    j.angles[RWristYaw] = DEG2RAD(0);

    // If we're performing emergency step, overwrite the armbehind parameter
    if (shouldEmergencyStep) {
        active.leftArmBehind = true;
        active.rightArmBehind = true;
    }

    if (active.leftArmBehind)
    {
        j.stiffnesses[Joints::LShoulderPitch] = 0.3;

        j.angles[Joints::LShoulderPitch] = DEG2RAD(100);
        j.angles[Joints::LShoulderRoll] = DEG2RAD(-13);
        j.angles[Joints::LElbowRoll] = DEG2RAD(-20);
        j.angles[Joints::LElbowYaw] = DEG2RAD(20);
    }
    if (active.rightArmBehind)
    {
        j.stiffnesses[Joints::RShoulderPitch] = 0.3;

        j.angles[Joints::RShoulderPitch] = DEG2RAD(100);
        j.angles[Joints::RShoulderRoll] = DEG2RAD(13);
        j.angles[Joints::RElbowRoll] = DEG2RAD(20);
        j.angles[Joints::RElbowYaw] = DEG2RAD(-20);
    }

    // If we're in emergency step, have higher stiffness for the shoulder joints
    // because the robot's shoulder is probably stuck on something and we need more stiffness
    // to achieve the required position
    if (shouldEmergencyStep) {
        j.stiffnesses[Joints::LShoulderRoll] = 1.0;
        j.stiffnesses[Joints::LShoulderPitch] = 1.0;
        j.stiffnesses[Joints::RShoulderRoll] = 1.0;
        j.stiffnesses[Joints::RShoulderPitch] = 1.0;
    }


    // 10.2 Turn
    j.angles[Joints::LHipYawPitch] = -turnRL;

    // 10.3 Sagittal Joints
    j.angles[Joints::LHipPitch] = -HpL;
    j.angles[Joints::RHipPitch] = -HpR;
    j.angles[Joints::LKneePitch] = KpL;
    j.angles[Joints::RKneePitch] = KpR;

    // Only activate balance control if foot is on the ground
    j.angles[Joints::LAnklePitch] = -ApL;
    j.angles[Joints::RAnklePitch] = -ApR;
    if (walkState != NOT_WALKING and nextFootSwitchT > 0) {
       if (bodyModel.isLeftPhase) {
            j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
            j.angles[Joints::RHipPitch] += sagittalHipBalanceAdjustment;
        } else {
            j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
            j.angles[Joints::LHipPitch] += sagittalHipBalanceAdjustment;
        }
    } else if (walk2014Option == KICK) {
        if (bodyModel.isLeftPhase)
            j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
        else
            j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
    } else {
        j.angles[Joints::RAnklePitch] += sagittalBalanceAdjustment;
        j.angles[Joints::LAnklePitch] += sagittalBalanceAdjustment;
    }

    // 10.4 Coronal Joints
    j.angles[Joints::LHipRoll] = HrL;
    j.angles[Joints::RHipRoll] = HrR;
    j.angles[Joints::LAnkleRoll] = ArL;
    j.angles[Joints::RAnkleRoll] = ArR;

    // Add in joint adjustments for kicks
    if (walk2014Option == KICK) {
        addKickJoints(j);
        // Add in some coronal balancing
        if (bodyModel.isLeftPhase) {
            j.angles[Joints::RAnkleRoll] += coronalBalanceAdjustment;
            if(hipBalance == true) {
                j.angles[Joints::RHipRoll] += coronalBalanceAdjustment / 2;
            }
        }
        else {
            j.angles[Joints::LAnkleRoll] += coronalBalanceAdjustment;
            if(hipBalance == true) {
                j.angles[Joints::LHipRoll] += coronalBalanceAdjustment / 2;
            }
        }
    }

    // Fill out some debug information
    motionDebugInfo.feetPosition = FeetPosition(
      MM_PER_M * -forwardL, MM_PER_M * leftL, RAD2DEG(turnRL),
      MM_PER_M * -forwardR, MM_PER_M * leftR, RAD2DEG(-turnRL));

    return j;
}

float Walk2014Generator::clamp(const int min, float value, const int max) const { return MIN(max, MAX(min, value)); }

void Walk2014Generator::prepKick(bool isLeft, BodyModel &bodyModel) {
    t = 0;
    stableCounter = 0;
    walk2014Option = KICK;
    walkState = NOT_WALKING;
    turnAngle = 0;
    kickT = 0;
    holdAnkle = false;
    hipBalance = false;
    if (isLeft) {
        lastKickForward = forwardL;
        lastKneePitch = kneePitchL;
        lastSide = leftL;
        bodyModel.setIsLeftPhase(true);
        forwardR = 0;
        foothR = 0;
    } else {
        lastKickForward = forwardR;
        lastKneePitch = kneePitchR;
        lastSide = -leftR;
        bodyModel.setIsLeftPhase(false);
        forwardL = 0;
        foothL = 0;
    }
}

// nicer for the motors
bool Walk2014Generator::canAbortKick() {
    float totalShift = shiftPeriod / 4;
    return kickT >= totalShift && kickT < backPhase;
}


// nicer for the motors
bool Walk2014Generator::canAbortKickBeforeLean() {
    return kickT == 0;
}

void Walk2014Generator::makeForwardKickJoints(float kickingLean, float kickStepH, float &footh, float &forwardDist, float &side, float &kneePitch,
        float &shoulderRoll, float &anklePitch, float&ballX, float &ballY, ActionCommand::All* request, const SensorValues &sensors,
        float &gyroXOntoSupportFoot, MotionDebugInfo &motionDebugInfo) {
    bool ballXYDebug = false;
   // Don't execute kick if we're not stable yet. Once we've achieved the balancing criteria for
    if (kickT == 0)
    {
        if (request->body.extraStableKick)
        {
            // Ensure we're extra stable if we've been requested to. This sacrifices execution time for stability

            if (std::fabs(sensors.sensors[InertialSensor_GyroscopeX]) < DEG2RAD(kickExtraStableGyroscopeX) &&
                std::fabs(sensors.sensors[InertialSensor_AngleX]) < DEG2RAD(kickExtraStableAngleX))
            {
                // increment stable counter
                stableCounter++;
            }
            else
            {
                // reset stablecounter
                stableCounter = 0;
            }
        }
        else
        {
            // Ensure we're stable enough to kick. This sacrifies stability for execution time.

            float gyroXAwayFromSupportFoot = -gyroXOntoSupportFoot;
            if (std::fabs(sensors.sensors[InertialSensor_AngleX]) < DEG2RAD(kickStableAngleX) &&
                gyroXOntoSupportFoot < DEG2RAD(kickGyroscopeXOntoSupportFootThresh) &&
                gyroXAwayFromSupportFoot < DEG2RAD(kickGyroscopeXAwayFromSupportFootThresh))
            {
                // increment stable counter
                stableCounter++;
            }
            else
            {
                // reset stablecounter
                stableCounter = 0;
            }
        }

        if (stableCounter < kickStableNumFrames) return;  // Return if we're not stable yet
    }

    kickT += dt;
    float totalPhase = backPhase + kickPhase + throughPhase + endPhase;

    // Used to enable hip stabalisation after lean during kick
    hipBalance = false;

    // Update side position of ball as late as possible.
    if (kickT < backPhase * 0.8) {
        dynamicSide = ballY / 1000.0 - 0.050; // offset to kick point on foot
    }
    // Max safe/useful value for lifting leg (without going past joint limits/balancing)
    float sideAmp = MAX(0.f, MIN(0.10f, dynamicSide));
    float kickAmp = -0.07; // how far forward the kick should reach
    float kickPower = pow(power, 1.7);
    float kickBackAmp = -kickAmp * kickPower;

    float ballXAdjust = ballX / 3.5;

    // Calculate the knee extension to hit balls far away from the robot
    if (kickT < backPhase) {
        kneePitchEnd = -MAX(35, MIN(75, ballXAdjust));
        anklePitchEnd = MAX(0, MIN(50,-kneePitchEnd * 1.3));
        if (fabs(ballY) > 40 && ballX > 190){
            anklePitchEnd += MAX(0, MIN(15, dynamicSide));
        }
        if (ballX < 200) {
            anklePitchStart = (ballX - 180) / 2;
            anklePitchStart = MAX(5, anklePitchStart);
            swingDelayFactor = 0.05;
            holdAnkle = true;
        } else if (ballX < 215) {
            anklePitchStart = 0.33 * ballX - 56;
            swingDelayFactor = 0.2;
        } else {
            anklePitchStart = 15;
            swingDelayFactor = 0.3;
        }
    } else if (ballXYDebug == false) {
        motionDebugInfo.x = ballX;
        motionDebugInfo.y = ballY;
        ballXYDebug = true;
    }

    float shoulderRollAmp = sideAmp * shoulderRollAmpMultiplier; // how much arm should lift to make room for raised kicking leg
    float kneePitchAmp = DEG2RAD(35);
    float kneePitchAmpEnd = DEG2RAD(kneePitchEnd);
    float anklePitchAmp = DEG2RAD(anklePitchEnd);
    float anklePitchRetracted = DEG2RAD(anklePitchStart);
    float anklePitchAmpFinal = DEG2RAD(35);

    if (lastKickTurn > 0) {
        shiftPeriod = 3.5;
    }

    // Shift weight over and swing foot back
    if (kickT < backPhase) {
        float totalShift = shiftPeriod / 4;
        float halfShift = totalShift / 2;

        // shift weight sideways
        if (kickT < totalShift) {

            rock = DEG2RAD(kickingLean) * parabolicStep(kickT, totalShift, 0);
            // If the robot had turned, bring the feet back together after some time for foot to lift off the ground
            if (lastKickTurn > 0) {
                if (kickT >= halfShift) {
                    turnRL = lastKickTurn * (1 - parabolicStep(kickT - halfShift, halfShift, 0.0));
                } else {
                    turnRL = lastKickTurn;
                }
                // Add in some extra lean while shifting weight along with turn
                rock += DEG2RAD(kickingLean) / 3.5 * parabolicReturn(kickT / totalShift);
            }
        } else {
            turnRL = 0;
            lastKickTurn = 0;
        }

        // only start lifting the kicking at 1/3
        if (t >= totalShift / 3) {
            float t3 = kickT - totalShift / 3;
            float endT = backPhase - totalShift / 3;
            footh = kickStepH * parabolicStep(t3, endT, 0);
        }

        // Once we're halfway through shifting our weight, start moving the foot back.
        if (kickT >= halfShift) {
            float kickT2 = kickT - halfShift;
            float endT = backPhase - halfShift;
            forwardDist = interpolateSmooth(0, kickBackAmp, kickT2, endT);
            side = interpolateSmooth(0, sideAmp, kickT2, endT);
            shoulderRoll = interpolateSmooth(0, shoulderRollAmp, kickT2, endT);
            kneePitch = interpolateSmooth(0, kneePitchAmp * 0.9, kickT2, endT);
            anklePitch = interpolateSmooth(0, anklePitchRetracted, kickT2, endT);
        }
        // Swing foot forward.
    } else if (kickT < (backPhase + kickPhase)) {
        // Use hip balance now
        hipBalance = true;
        if (kickT >= backPhase + swingDelayFactor * kickPhase) {
            forwardDist = interpolateSmooth(kickBackAmp, kickAmp, kickT - backPhase - swingDelayFactor*kickPhase, kickPhase);
            anklePitch = interpolateSmooth(anklePitchRetracted, anklePitchAmp, kickT - backPhase - swingDelayFactor*kickPhase, kickPhase);
        }
        if (holdAnkle == true) {
            // Overwrite if we want to hold ankle
            anklePitch = anklePitchRetracted;
        }
        side = sideAmp;
        shoulderRoll = shoulderRollAmp;
        kneePitch = squareSmooth(kneePitchAmp * 0.9, kneePitchAmpEnd, kickT - backPhase, kickPhase);
        // Hold...
    } else if (kickT < (backPhase + kickPhase + throughPhase)) {
        // Keep hip balance
        hipBalance = true;
        forwardDist = kickAmp;
        side = sideAmp;
        shoulderRoll = shoulderRollAmp;
        kneePitch = kneePitchAmpEnd;
        if (holdAnkle == true) {
            anklePitch = interpolateSmooth(anklePitchRetracted, anklePitchAmpFinal,kickT - backPhase - kickPhase, throughPhase);
        } else {
            anklePitch = interpolateSmooth(anklePitchAmp, anklePitchAmpFinal,kickT - backPhase - kickPhase, throughPhase);
        }
    //     // Return foot. @ijnek: this step was taken out in an attempt to stabilise the kick
    // } else if (kickT < (backPhase + kickPhase + throughPhase + endPhase)) {
    //     forwardDist = interpolateSmooth(lastKickForward, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     side = interpolateSmooth(lastSide, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     shoulderRoll = interpolateSmooth(lastShoulderRollAmp, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     kneePitch = interpolateSmooth(lastKneePitch, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //     anklePitch = interpolateSmooth(lastAnklePitch, 0, kickT - backPhase - kickPhase - throughPhase, endPhase);
    //
        // Shift weight back to both feet.
    } else if (kickT < (totalPhase + shiftEndPeriod / 4)) {
        forwardDist = lastKickForward - lastKickForward * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        side = lastSide - lastSide * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        shoulderRoll = lastShoulderRollAmp - shoulderRollAmp * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        kneePitch = lastKneePitch - lastKneePitch * parabolicStep(kickT-totalPhase, shiftEndPeriod/8, 0);
        anklePitch = lastAnklePitch - lastAnklePitch * parabolicStep(kickT-totalPhase, shiftEndPeriod/6, 0);
        rock = lastRock - lastRock * parabolicStep(kickT-totalPhase, shiftEndPeriod/4, 0);
        footh = lastFooth - lastFooth * parabolicStep(kickT-totalPhase, shiftEndPeriod/6, 0);
    } else {
        kickT = 0;
        rock = 0;
        footh = 0;
        walk2014Option = WALK;
        request->body.actionType = ActionCommand::Body::WALK;
        lastKickTime = 0;
        ballXYDebug = false;
        motionDebugInfo.kickCompleted = true;
    }

    // aborting or ending a kick from these values
    if (kickT < (backPhase + kickPhase + throughPhase)) {
        lastKickForward = forwardDist;
        lastKneePitch = kneePitch;
        lastSide = side;
        lastFooth = footh;
        lastAnklePitch = anklePitch;
        lastRock = rock;
        lastShoulderRollAmp = shoulderRollAmp;
        ballXYDebug = false;
    }


}

void Walk2014Generator::addKickJoints(JointValues &j) {
    j.angles[Joints::LKneePitch] += kneePitchL;
    j.angles[Joints::LAnklePitch] += anklePitchL + shoulderRollL;
    j.angles[Joints::LShoulderPitch] -= kneePitchL;

    j.angles[Joints::RKneePitch] += kneePitchR;
    j.angles[Joints::RAnklePitch] += anklePitchR + shoulderRollR;
    j.angles[Joints::RShoulderPitch] -= kneePitchR;
}

float Walk2014Generator::linearStep(float time, float period) {
    if (time <= 0)
        return 0;
    if (time >= period)
        return 1;
    return time / period;
}

float Walk2014Generator::parabolicReturn(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        y = 8 * f * f;
    }
    if (f >= 0.25f && f < 0.5f) {
        x = 0.5f - f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        x = f - 0.5f;
        y = 8 * x * x;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        x = 1.0f - f;
        y = 8 * x * x;
    }
    return y;
}

float Walk2014Generator::parabolicReturnMod(float f) { //normalised [0,1] up and down
    double x = 0;
    double y = 0;
    if (f < 0.25f) {
        // y: 0 -> 0.75
        y = 8 * f * f * 1.50;
    }
    if (f >= 0.25f && f < 0.5f) {
        // y: 0.75 -> 1.00
        x = 0.5f - f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.5f && f < 0.75f) {
        // y: 1.00 -> 0.75
        x = f - 0.5f;
        y = 8 * x * x;
        y = y / 2;
        y = 1.0f - y;
    }
    if (f >= 0.75f && f <= 1.0f) {
        // y: 0.75 -> 0
        x = 1.0f - f;
        y = 8 * x * x * 1.50;
    }
    return y;
}

float Walk2014Generator::parabolicStep(float time, float period, float deadTimeFraction) { //normalised [0,1] step up
    float deadTime = period * deadTimeFraction / 2;
    if (time < deadTime + dt / 2)
        return 0;
    if (time > period - deadTime - dt / 2)
        return 1;
    float timeFraction = (time - deadTime) / (period - 2 * deadTime);
    if (time < period / 2)
        return 2.0 * timeFraction * timeFraction;
    return 4 * timeFraction - 2 * timeFraction * timeFraction - 1;
}

Odometry Walk2014Generator::updateOdometry(bool isLeftSwingFoot) {
    // Work out incremental forward, left, and turn values for next time step
    float turnOdo = -(turnRL - prevTurn);
    float leftOdo = -(leftR - prevLeftR);
    float forwardOdo = (forwardR - prevForwardR);
    if (!isLeftSwingFoot) {
        turnOdo *= -1;
        leftOdo = -(leftL - prevLeftL);
        forwardOdo = (forwardL - prevForwardL);
    }
    forwardOdo *= MM_PER_M;
    leftOdo *= MM_PER_M;
    //Calibrate odometry to match the actual speed
    forwardOdo *= 1;
    leftOdo *= 1.23;
    turnOdo *= -.80;

    // backup odometry values
    prevTurn = turnRL;
    prevLeftL = leftL;
    prevLeftR = leftR;
    prevForwardL = forwardL;
    prevForwardR = forwardR;
    return Odometry(forwardOdo, leftOdo, turnOdo);
}

bool Walk2014Generator::isActive() {
    return !stopped;
}

void Walk2014Generator::readOptions(const boost::program_options::variables_map &config) {
    // Each robot has a slightly different lean angle that works. This is dealt through individual lean offsets.
    kickLeanOffsetLVaried = config["kick.leanOffsetLVaried"].as<bool>();
    kickLeanOffsetRVaried = config["kick.leanOffsetRVaried"].as<bool>();
    kickLeanOffsetL = config["kick.leanOffsetL"].as<float>();
    kickLeanOffsetR = config["kick.leanOffsetR"].as<float>();
    kickLeanOffsetLInner = config["kick.leanOffsetLInner"].as<float>();
    kickLeanOffsetRInner = config["kick.leanOffsetRInner"].as<float>();
    kickLeanOffsetLMid = config["kick.leanOffsetLMid"].as<float>();
    kickLeanOffsetRMid = config["kick.leanOffsetRMid"].as<float>();
    kickLeanOffsetLOuter = config["kick.leanOffsetLOuter"].as<float>();
    kickLeanOffsetROuter = config["kick.leanOffsetROuter"].as<float>();
    kickLean = config["kick.lean"].as<float>();
    KpGyro = config["motion.KpGyro"].as<float>();      // Proportional gain
    KdGyro = config["motion.KdGyro"].as<float>();      // Derivative gain
    KpAngle = config["motion.KpAngle"].as<float>();    // Proportional gain
    KiAngle = config["motion.KiAngle"].as<float>();    // Integral gain
    KdAngle = config["motion.KdAngle"].as<float>();    // Derivative gain
    kickStableNumFrames = config["kick.stableNumFrames"].as<int>();
    kickStableAngleX = config["kick.stableAngleX"].as<float>();
    kickExtraStableAngleX = config["kick.extraStableAngleX"].as<float>();
    kickExtraStableGyroscopeX = config["kick.extraStableGyroscopeX"].as<float>();
    kickGyroscopeXOntoSupportFootThresh = config["kick.kickGyroscopeXOntoSupportFootThresh"].as<float>();
    kickGyroscopeXAwayFromSupportFootThresh = config["kick.kickGyroscopeXAwayFromSupportFootThresh"].as<float>();
    runningKickCalibration = config[CALIBRATION_KICK].as<bool>();
}

void Walk2014Generator::reset() {
    initialise();
    llog(INFO) << "Walk2014 reset" << endl;
}

void Walk2014Generator::stop() {
    // this does nothing
    stopping = true;
    llog(INFO) << "Walk2014 stop" << endl;
}

float Walk2014Generator::interpolateSmooth(float start, float end, float tCurrent, float tEnd) {
    return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

float Walk2014Generator::squareSmooth(float start, float end, float tCurrent, float tEnd) {
    return ((end - start)/(tEnd * tEnd)) * ((tCurrent * tCurrent)) + start;
}

XYZ_Coord Walk2014Generator::mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf) {
    // MFOOT2BODY Transform coords from foot to body.
    // This code originates from 2010 using symbolic equations in Matlab to perform the coordinate transforms - see team report (BH)
    // In future this approach to IK for the Nao should be reworked in closed form, significantly reducing the size of the code the
    // the computational complexity (BH)
    XYZ_Coord result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    result.x = thigh * (s2 * s3 - c2 * c3 * c4) + tibia * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4))
            - yf
                    * (c6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            + c3 * s4 * s6)
            + zf
                    * (s6 * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
                            - c3 * c6 * s4)
            + xf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    result.y = xf
            * (c1 * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                    + s5
                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            + tibia * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
            + thigh * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
            - yf
                    * (s6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            + c6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))))
            - zf
                    * (c6 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))));
    result.z = yf
            * (s6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                    + c6
                            * (c1
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                    - s5
                                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))))
            - tibia * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
            - thigh * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
            - xf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + zf
                    * (c6 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)
                            - s6
                                    * (c1
                                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                                            - s5
                                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))));
    return result;
}

Walk2014Generator::Hpr Walk2014Generator::hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap, float Ar, float xf, float yf, float zf, XYZ_Coord e) {
    // Code from 2010 to perform interative Inverse Kinematics.
    // Symbolic equations generated in Matlab - see 2010 team report for details and reference
    Hpr result;
    float pi = M_PI;
    float tibia = this->tibia * 1000;
    float thigh = this->thigh * 1000;
    float k = sqrt(2.0);
    float c1 = cos(Ap);
    float c2 = cos(Hr + pi / 4.0);
    float c3 = cos(Hyp - pi / 2.0);
    float c4 = cos(Hp);
    float c5 = cos(Kp);
    float c6 = cos(Ar - pi / 2.0);
    float s1 = sin(Kp);
    float s2 = sin(Hp);
    float s3 = sin(Hyp - 1.0 / 2.0 * pi);
    float s4 = sin(Hr + 1.0 / 4.0 * pi);
    float s5 = sin(Ap);
    float s6 = sin(Ar - 1.0 / 2.0 * pi);
    float j11 = thigh * (c4 * s3 + c2 * c3 * s2) - tibia * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2))
            + xf * (c1 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)) - s5 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)))
            + c6 * yf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)))
            - s6 * zf * (c1 * (s1 * (s2 * s3 - c2 * c3 * c4) - c5 * (c4 * s3 + c2 * c3 * s2)) + s5 * (s1 * (c4 * s3 + c2 * c3 * s2) + c5 * (s2 * s3 - c2 * c3 * c4)));
    float j12 = yf * (c6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) - c2 * c3 * s6)
            - tibia * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)
            - zf * (s6 * (c1 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4) + s5 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4)) + c2 * c3 * c6)
            + xf * (c1 * (c3 * c4 * s1 * s4 + c3 * c5 * s2 * s4) - s5 * (c3 * s1 * s2 * s4 - c3 * c4 * c5 * s4)) + c3 * c4 * s4 * thigh;
    float j21 = xf
            * (c1 * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f) - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f))
                    - s5
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)))
            - tibia * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
            - thigh * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
            + c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)))
            - s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) + (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f) - (c3 * c4 * k) / 2.0f)));
    float j22 = tibia * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
            + xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f)))
            + yf
                    * (s6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + zf
                    * (c6 * ((k * s4) / 2.0f + (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f))))
            + c4 * thigh * ((c2 * k) / 2.0f - (k * s3 * s4) / 2.0f);
    float j31 = tibia * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f) + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
            - xf
                    * (c1
                            * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                    - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f))
                            - s5
                                    * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                            + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)))
            + thigh * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
            - c6 * yf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)))
            + s6 * zf
                    * (c1
                            * (c5 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)
                                    + s1 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f))
                            + s5
                                    * (c5 * (c4 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) - (c3 * k * s2) / 2.0f)
                                            - s1 * (s2 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f) + (c3 * c4 * k) / 2.0f)));
    float j32 = -tibia * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
            - xf
                    * (c1 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                            + s5 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f)))
            - yf
                    * (s6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            - c6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - zf
                    * (c6 * ((k * s4) / 2.0f - (c2 * k * s3) / 2.0f)
                            + s6
                                    * (c1 * (c4 * c5 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) - s1 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))
                                            - s5 * (c4 * s1 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f) + c5 * s2 * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f))))
            - c4 * thigh * ((c2 * k) / 2.0f + (k * s3 * s4) / 2.0f);
    float xbe = e.x;
    float ybe = e.y;
    float zbe = e.z;
    float lambda = 0.4f;
    float la2 = lambda * lambda;
    float la4 = la2 * la2;
    float j322 = j32 * j32;
    float j222 = j22 * j22;
    float j122 = j12 * j12;
    float j212 = j21 * j21;
    float j112 = j11 * j11;
    float j312 = j31 * j31;
    float sigma = 1.0f
            / (la4 + j112 * j222 + j122 * j212 + j112 * j322 + j122 * j312 + j212 * j322 + j222 * j312 + j112 * la2 + j122 * la2 + j212 * la2 + j222 * la2 + j312 * la2 + j322 * la2
                    - 2.0f * j11 * j12 * j21 * j22 - 2.0f * j11 * j12 * j31 * j32 - 2.0f * j21 * j22 * j31 * j32);
    result.Hp = sigma * xbe * (j11 * j222 + j11 * j322 + j11 * la2 - j12 * j21 * j22 - j12 * j31 * j32)
            + sigma * ybe * (j122 * j21 + j21 * j322 + j21 * la2 - j11 * j12 * j22 - j22 * j31 * j32)
            + sigma * zbe * (j122 * j31 + j222 * j31 + j31 * la2 - j11 * j12 * j32 - j21 * j22 * j32);
    result.Hr = sigma * xbe * (j12 * j212 + j12 * j312 + j12 * la2 - j11 * j21 * j22 - j11 * j31 * j32)
            + sigma * ybe * (j112 * j22 + j22 * j312 + j22 * la2 - j11 * j12 * j21 - j21 * j31 * j32)
            + sigma * zbe * (j112 * j32 + j212 * j32 + j32 * la2 - j11 * j12 * j31 - j21 * j22 * j31);
    return result;
}
