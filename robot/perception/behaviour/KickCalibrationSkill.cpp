#include "KickCalibrationSkill.hpp"

#include <iostream>
#include <fstream>

#include "types/BallInfo.hpp"
#include "types/AbsCoord.hpp"

#include "utils/clevernao/CleverNaoDefines.hpp"

#define X_TARGET 200
#define Y_TARGET 60

#define DISTANCE_THRESHOLD 80
#define BALL_LOST_COUNT_THRESHOLD 50

KickCalibrationSkill::KickCalibrationSkill(): ballLostCount(0)//, randomLeanGenerated(false)
{
}

BehaviourRequest KickCalibrationSkill::execute(const AbsCoord &ballRel, const std::vector<BallInfo> &detectedBalls,
    ActionCommand::Body::Foot foot, CleverNaoInfoBehaviour& cleverNaoInfo, CleverNaoInfoMotion& cleverNaoInfoMotion,
                                                                                            const std::string& bodyName)
{
    // The behaviour to be performed.
    BehaviourRequest request;

    // The desired Y position of the ball, adjusted for whether the robot is kicking with the left or right foot.
    float adjustedY = Y_TARGET * (foot == ActionCommand::Body::LEFT ?  1 : -1);

    // Just looking down is sufficient for this.
    request.actions.head.pitch = 1;

    // Default to crouching.
    request.actions.body = ActionCommand::Body::WALK;
    request.actions.body.forward = 1;
    request.actions.body.left = 0;
    request.actions.body.turn = 0;
    request.actions.body.bend = 1;

    // Keep track of whether we've lost the ball so that the robot doesn't kick if it can't see the ball.
    if (detectedBalls.empty())
        ballLostCount++;
    else
        ballLostCount = 0;

    /*
    if(!randomLeanGenerated)
    {
        cleverNaoInfo.kickLeanOverwrite = ((float)rand()/(float)(RAND_MAX/10))-5.0f;
        std::cout << "Kick Lean Offset: " << cleverNaoInfo.kickLeanOverwrite << std::endl;
        cleverNaoInfo.overwriteKickLean = true;
        randomLeanGenerated = true;
    }
    */

    // If the robot can see the ball, it's roughly at the target position and Clever Nao isn't telling us not to, kick.
    if (ballLostCount < BALL_LOST_COUNT_THRESHOLD && (!cleverNaoInfo.cleverNaoBehaviour || cleverNaoInfo.kick))
    {
        float distance = ballRel.convertToRobotRelative(AbsCoord(X_TARGET, adjustedY, 0)).distance();
        if (distance <= DISTANCE_THRESHOLD)
        {
            request.actions.body = ActionCommand::Body::KICK;
            if(cleverNaoInfo.cleverNaoBehaviour)
            {
                // Set up for a Clever Nao kick.
                if(!cleverNaoInfo.recordGyroX)
                {
                    // The current situation.
                    std::vector<float> situation;

                    // Clear clever nao data and start recording.
                    cleverNaoInfo.recordGyroX = true;
                    cleverNaoInfo.requestClearGyroXReadings = true;
                    cleverNaoInfo.recordCoM = true;
                    cleverNaoInfo.requestClearCoMReadings = true;
                    cleverNaoInfo.recordRRBallPos = true;
                    cleverNaoInfo.rrBallPos.clear();

                    // Use the kick lean value for the situation closest to the current one.
                    situation.push_back(ballRel.y());
                    cleverNaoInfo.kickLeanOverwrite =
                                               cleverNaoInfo.theExperiment->getRUNSWiftFormatValues(situation, true)[1];
                }

                // Kick with the foot requested by Clever Nao.
                if(cleverNaoInfo.kickLeft)
                    request.actions.body.foot = ActionCommand::Body::LEFT;
                else
                    request.actions.body.foot = ActionCommand::Body::RIGHT;
            }
            else
                request.actions.body.foot = foot;
        }
    }

    if(cleverNaoInfo.cleverNaoRecording && !cleverNaoInfo.cleverNaoBehaviour)
    {
        // If a kick has been aborted and then completed, reset the kick flags and the gyro readings.
        if(cleverNaoInfoMotion.kickAborted && cleverNaoInfoMotion.kickCompleted)
        {
            cleverNaoInfoMotion.kickAborted = false;
            cleverNaoInfoMotion.kickCompleted = false;
            cleverNaoInfo.recordGyroX = false;
            cleverNaoInfo.requestClearGyroXReadings = true;
            cleverNaoInfo.recordCoM = false;
            cleverNaoInfo.requestClearCoMReadings = true;
            cleverNaoInfo.recordRRBallPos = false;
            cleverNaoInfo.rrBallPos.clear();
        }

        // A full kick has been completed, save the relevant data.
        else if(cleverNaoInfoMotion.kickCompleted)
        {
            // Stop recording.
            cleverNaoInfoMotion.kickCompleted = false;
            cleverNaoInfo.recordGyroX = false;
            cleverNaoInfo.recordCoM = false;
            cleverNaoInfo.recordRRBallPos = false;

            /*
            // Set the flag.
            randomLeanGenerated = false;

            // Tell CleverNaoInfo to save its data.

            // The file currently being output to.
            std::ofstream curFileOut;

            // Vector for temporarily storing the results of function calls.
            std::vector<float> tempVec;

            // The folder in which data is to be saved.
            std::string saveDir;

            // Get the directory in which recorded data is to be saved.
            saveDir = cleverNaoInfo.getNextDataDirectory(bodyName);

            // Save the raw recorded data.
            cleverNaoInfo.saveDataToDirectory(cleverNaoInfoMotion, saveDir);

            // Save the random lean used.

            // rUNSWift format values.
            curFileOut.open((saveDir + "/randomLean.txt").c_str(), std::ofstream::out);
            curFileOut << "1f" << std::endl;   // 1 dimension of floats.
            curFileOut << cleverNaoInfo.kickLeanOverwrite << std::endl;
            curFileOut.close();
            */

            // Clear existing recordings.
            cleverNaoInfo.requestClearGyroXReadings = true;
            cleverNaoInfo.requestClearCoMReadings = true;
            cleverNaoInfo.rrBallPos.clear();
        }

        // If data recording is set, request it.
        if(!cleverNaoInfo.recordGyroX)
        {
            cleverNaoInfo.recordGyroX = true;
            cleverNaoInfo.requestClearGyroXReadings = true;
            cleverNaoInfo.recordCoM = true;
            cleverNaoInfo.requestClearCoMReadings = true;
            cleverNaoInfo.recordRRBallPos = true;
            cleverNaoInfo.rrBallPos.clear();
        }
    }

    // Return the generated behaviour.
    return request;
}
