#include <algorithm>
#include <iostream>
#include <fstream>

#include "utils/clevernao/CleverBehaviourGenerator.hpp"

#include "types/Point.hpp"
#include "types/XYZ_Coord.hpp"

// The time it takes to stand up in us.
#define TIME_TO_STAND 5000000

// The minimum time between experiments in us.
#define MIN_TIME_BETWEEN_EXPERIMENTS 2500000

// The maximum gyro reading during a kick that will result in a reward of 1.0.
#define KICK_LEAN_EXPERIMENT_STABILITY_ESTIMATE_FOR_REWARD_OF_ONE 100.0f

/*
Creates a behaviour corresponding to the next action the nao should take given
the state of the experiment being performed. Returns true if the experiment is
complete, false otherwise.
*/
bool CleverBehaviourGenerator::createBehaviourForExperiment(Blackboard &bb, Experiment*& nextExperiment,
                                                     const CleverNaoExperimentType experimentType, std::string& saveDir)
{
    // Whether the experiment is complete.
    bool experimentComplete = false;

    // The information passed to and from Clever Nao.
    CleverNaoInfoBehaviour& cleverNaoInfo = bb.behaviour.cleverNaoInfo;
    CleverNaoInfoMotion& cleverNaoInfoMotion = bb.motion.cleverNaoInfo;

    // Firstly, check if we need to stand up. The nao actually has no idea if
    // it's standing, so we'll have to use our own flags.
    if(!startedStanding)
    {
        // Tell the nao to wait, which causes it to stand.
        bb.behaviour.cleverNaoInfo.setBehaviourWait();

        // Set our flag and time the stand.
        startedStanding = true;
        startedStandingTime.restart();

        // Start the time since lase experiment clock, so it doesn't prevent
        // the system from begginning.
        timeSinceLastExperimentFinished.restart();
    }

    // If there has been time for the nao to stand up, start the experiment.
    if(startedStandingTime.elapsed_us() > TIME_TO_STAND &&
                                            timeSinceLastExperimentFinished.elapsed_us() > MIN_TIME_BETWEEN_EXPERIMENTS)
    {
        // Set the nao as playing.
        bb.gameController.gameState = STATE_PLAYING;

        // Deal with the relevant experiment type.
        if(experimentType == KICK_LEAN_LEFT || experimentType == KICK_LEAN_RIGHT)
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

            // If a kick has just been completed the experiment is done, report the results.
            if(cleverNaoInfoMotion.kickCompleted)
            {
                // The gyro readings during the experiment.
                std::vector<float>& gyroReadings = cleverNaoInfoMotion.gyroReadingsX;

                // The Centre of Mass readings during the experiment.
                std::vector<XYZ_Coord>& CoM = cleverNaoInfoMotion.CoM;

                // The ball position recordings.
                std::vector<AbsCoord>& rrBallPos = cleverNaoInfo.rrBallPos;

                // CoM based estimate of the sideways velocity of the robot at each step.
                std::vector<float> velocityEstimates;

                // Estimate of the stability of the robot at each step. Higher is less stable.
                std::vector<float> stabilityEstimates;

                // The experiment values to be actually trained with.
                std::vector<float> experimentToTrain;

                // The worst stability estimate during the kick.
                float worstStability = 0;

                // The total stability value over the kick.
                float totalStability = 0;

                // The most extreme gyro reading.
                float maxGyro = 0;

                // The most extreme CoM reading.
                float maxCoM = 0;

                // The average squared offset from a balanced stance.
                float averageOffsetFromBalance = 0;

                // Stop recording.
                cleverNaoInfo.recordGyroX = false;
                cleverNaoInfo.recordCoM = false;
                cleverNaoInfo.recordRRBallPos = false;

                // Tell the nao to wait.
                cleverNaoInfo.setBehaviourWait();

                // Calculate reward proportional to the maximum gyro offset.
                for(unsigned int gyro=0; gyro<gyroReadings.size(); ++gyro)
                    maxGyro = std::max(maxGyro, (float)fabs(gyroReadings[gyro]));

                // Estimate stability based on centre of mass.
                stabilityEstimates.reserve(CoM.size());
                velocityEstimates.reserve(CoM.size());
                for(unsigned int com=0; com<CoM.size()-1; ++com)
                {
                    // The velocity estimate for this moment.
                    float velocityEstimate;

                    maxCoM = std::max(maxCoM, (float)fabs(CoM[com].y));
                    averageOffsetFromBalance += pow((float)fabs(CoM[com].y), 4);

                    // Estimate the velocity, setting it to 0 if the foot appears to have switched.
                    velocityEstimate = CoM[com+1].y - CoM[com].y;
                    if(velocityEstimate > 40 && ((CoM[com+1].y > 0) != (CoM[com].y > 0)))
                        velocityEstimate = 0.0f;

                    // Record the velocity and stability estimates.
                    velocityEstimates.push_back(velocityEstimate);
                    if((CoM[com].y > 0) == (velocityEstimate > 0))
                        stabilityEstimates.push_back(fabs(velocityEstimate));
                    else
                        stabilityEstimates.push_back(0.0f);
                }
                averageOffsetFromBalance /= CoM.size();

                // Determine the worst stability estimate during the kick.
                for(unsigned int estimate=0; estimate<stabilityEstimates.size(); ++estimate)
                {
                    if(stabilityEstimates[estimate] > worstStability)
                        worstStability = stabilityEstimates[estimate];
                    totalStability += stabilityEstimates[estimate];
                }

                // Set the experiment state to use the actual state encountered.
                if(cleverNaoInfo.kickLeft)
                    experimentToTrain.push_back(rrBallPos[0].y());
                else
                    experimentToTrain.push_back(-1.0f * rrBallPos[0].y());
                experimentToTrain.push_back(cleverNaoInfo.kickLeanOverwrite);
                cleverNaoInfo.theExperiment->setExperimentFromRUNSWiftFormatValues(experimentToTrain);

                // Reward is proportional to our estimated robot stability.
                cleverNaoInfo.theExperiment->reward =
                                               KICK_LEAN_EXPERIMENT_STABILITY_ESTIMATE_FOR_REWARD_OF_ONE/totalStability;

#ifdef SHOW_KICK_LEAN_EXPERIMENT_DEBUG
                // Display the experiment reward.
                std::cout << "Max Gyro: " << maxGyro << std::endl;
                std::cout << "Max CoM: " << maxCoM << std::endl;
                std::cout << "Average Squared Offset from Balance: " << averageOffsetFromBalance << std::endl;
                std::cout << "Experiment reward: " << cleverNaoInfo.theExperiment->reward << std::endl << std::endl;
#endif // SHOW_KICK_LEAN_EXPERIMENT_DEBUG

                // Record data if needed.
                if(cleverNaoInfo.cleverNaoRecording)
                {
                    // The file currently being output to.
                    std::ofstream curFileOut;

                    // Vector for temporarily storing the results of function calls.
                    std::vector<float> tempVec;

                    // Get the directory in which recorded data is to be saved.
                    saveDir = cleverNaoInfo.getNextDataDirectory(bb.kinematics.bodyName);

                    // Save the raw recorded data.
                    cleverNaoInfo.saveDataToDirectory(cleverNaoInfoMotion, saveDir);

                    // Save information about the experiment actually performed.

                    // rUNSWift format values.
                    curFileOut.open((saveDir + "/rUNSWiftExperiment.txt").c_str(), std::ofstream::out);
                    curFileOut << "1df" << std::endl;   // 1 dimension of floats.
                    tempVec = cleverNaoInfo.theExperiment->getRUNSWiftFormatValues();
                    for(unsigned int dim=0; dim < tempVec.size(); ++dim)
                        curFileOut << tempVec[dim] << std::endl;
                    curFileOut.close();

                    // CleverNao format values.
                    curFileOut.open((saveDir + "/cleverNaoExperiment.txt").c_str(), std::ofstream::out);
                    curFileOut << "1df" << std::endl;   // 1 dimension of floats.
                    tempVec = cleverNaoInfo.theExperiment->getCleverNaoFormatValues();
                    for(unsigned int dim=0; dim < tempVec.size(); ++dim)
                        curFileOut << tempVec[dim] << std::endl;
                    curFileOut.close();

                    // Reward value.
                    curFileOut.open((saveDir + "/reward.txt").c_str(), std::ofstream::out);
                    curFileOut << "1f" << std::endl;    // One float.
                    curFileOut << cleverNaoInfo.theExperiment->reward << std::endl;
                    curFileOut.close();
                }

                // Clear existing recordings.
                cleverNaoInfoMotion.kickCompleted = false;
                cleverNaoInfo.requestClearGyroXReadings = true;
                cleverNaoInfo.requestClearCoMReadings = true;
                cleverNaoInfo.rrBallPos.clear();

                // Mark that the experiment is complete.
                experimentComplete = true;
                experimentInProgress = false;

                // Restart the timer between experiments.
                timeSinceLastExperimentFinished.restart();
            }

            // Otherwise perform a kick.
            else if(!experimentInProgress)
            {
                // The values for this experiment.
                std::vector<float> experimentValues(KICK_LEAN_EXPERIMENT_EXPERIMENT_DIMS,
                                                                                    KICK_LEAN_EXPERIMENT_DEFAULT_VALUE);

                // A value situation.
                std::vector<float> valueSituation;

                // Kick.
                cleverNaoInfo.setBehaviourKick();

                // If the experiment is of the correct type use its values rather than the default values.
                if(nextExperiment->experimentType == experimentType)
                    experimentValues = nextExperiment->getCleverNaoFormatValues();

                // Put the experiment on the blackboard.
                cleverNaoInfo.theExperiment = new ExperimentKickLean(experimentType, experimentValues);
                delete nextExperiment;
                nextExperiment = cleverNaoInfo.theExperiment;

                // Overwrite the kick lean.
                if(experimentType == KICK_LEAN_LEFT)
                    cleverNaoInfo.kickLeft = true;
                else
                    cleverNaoInfo.kickLeft = false;
                cleverNaoInfo.overwriteKickLean = true;
                cleverNaoInfo.kickLeanOverwrite = nextExperiment->getRUNSWiftFormatValues()[1];

                // Set up the relevant situation set.
                valueSituation.push_back(0.2f);
                nextExperiment->valueSituations.push_back(valueSituation);
                valueSituation.clear();
                valueSituation.push_back(0.6f);
                nextExperiment->valueSituations.push_back(valueSituation);
                valueSituation.clear();
                valueSituation.push_back(1.0f);
                nextExperiment->valueSituations.push_back(valueSituation);

                // Mark that the experiment is incomplete and in progress.
                experimentComplete = false;
                experimentInProgress = true;

                // Start the experiment timer.
                timeSinceLastExperimentStarted.restart();
            }
        }
    }

    // If the robot is between experiments the experiment is not complete.
    else
        experimentComplete = false;

    return(experimentComplete);
}
