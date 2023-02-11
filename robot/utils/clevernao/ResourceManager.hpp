#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include <sys/stat.h>
#include <vector>
#include <float.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "blackboard/Blackboard.hpp"

#define GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT

//#define DEBUG_RESOURCE_MANAGER

#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
#include "utils/clevernao/gaussianprocess/MultipleExperimentSituationSearchGaussianProcess.hpp"
#else
#include "utils/clevernao/gaussianprocess/SituationSearchGaussianProcess.hpp"
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
#include "utils/clevernao/gaussianprocess/GPFunc/GPFunc.hpp"
#include "utils/clevernao/gaussianprocess/GPFunc/FlatFunc.hpp"

// The base uncertainty in the gaussian process.
#define GP_VARIANCE_KICK 0.5f

// The measurement noise expected in training samples for the gaussian process.
#define GP_NOISE_KICK 0.2f

// How rapidly the true function is expected to change. Lower means more
// rapidly.
#define BASE_LENGTH_SCALES 0.5f

// An absolute value added to the time penalty.
#define TIME_PENALTY_CONSTANT_KICK 3.33f

// Multiplier for each dimension of the euclidean between the target training
// point and the current point, when applying the time resource penalty.
#define TIME_PENALTY_MULTS 0.0f

// The point from which difference to offset time penalties are calculated.
#define TIME_PENALTY_OFFSETS 0.0f

// The multipliers for each dimension of the time penalty offsets.
#define TIME_PENALTY_OFFSET_MULTS 0.0f

// The overall system aims to be within APPROXIMATION_RANGE with the certainty
// given by CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS. So the model believes
// there is a cdf(CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS) chance that the
// best it found is within APPROXIMATION_RANGE_KICK of the best possible
// average reward.

// The confidence interval (probability range) for the experiments. This is
// how valuable an experiment in a poorly understood part of the search space
// is, so higher means more experiments.
#define CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS_KICK 3.0f

// An absolute value added to the additative penalty.
#define APPROXIMATION_RANGE_KICK 0.2f

// A minimum number of trials to ensure the system gets a suitable sample.
#define MIN_TRIALS_KICK 10
#define MIN_TRIALS_KICK_TIMES_SITUATIONS (MIN_TRIALS_KICK*3)

// The number of zero improvements the best search should see before giving up
// after the first test.
#define ZEROES_BEFORE_GIVE_UP 5

// How low estimated computation value needs to get before we give up, even if
// no good experiment has been found.
#define COMPUTATION_VALUE_BEFORE_GIVE_UP 0.0001f

inline std::vector<SituationSearchGaussianProcessParams>
    createCleverNaoResourceManagerParams(GPFunc* mean,
                                     std::vector<float>& timePenaltyDistance,
                                     std::vector<float>& timePenaltyOffsets,
                                     std::vector<float>& timePenaltyOffsetMults)
{
    std::vector<SituationSearchGaussianProcessParams> params;

    // Kick.
    params.push_back(SituationSearchGaussianProcessParams(mean,
                     GP_VARIANCE_KICK, GP_NOISE_KICK, BASE_LENGTH_SCALES,
                     CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS_KICK,
                     APPROXIMATION_RANGE_KICK, timePenaltyDistance,
                     timePenaltyOffsets, timePenaltyOffsetMults,
                                                   TIME_PENALTY_CONSTANT_KICK));

    return(params);
}

template<int experimentDims, int situationDims, int heuristicDims,
                                                             int numExperiments>
class ResourceManager
{

public:

    /*
    Creates a new resource manager. It trains a experimentDims dimensional
    gaussian process in order to determine the best mapping between heuristics
    and situations.
    */
    ResourceManager(GPFunc* mean) :
        timePenaltyDistance(experimentDims, TIME_PENALTY_MULTS),
        timePenaltyOffsets(experimentDims, TIME_PENALTY_OFFSETS),
        timePenaltyOffsetMults(experimentDims, TIME_PENALTY_OFFSET_MULTS),
        trialCount(0),
        gaussianProcess(createCleverNaoResourceManagerParams(mean,
            timePenaltyDistance, timePenaltyOffsets, timePenaltyOffsetMults)
#ifndef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
        [0]
#endif // not GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
                                                                            ) {}

    /*
    Adds the provided training point to the gaussian process, with the given
    reward value. The trainingPoint vector is expected to be situationDims
    followed by heuristicDims.
    */
    void addTraining(const std::vector<float> &trainingPoint,
                                  const float result, const int experimentType);

    /*
    Selects a new experiment by carefully considering the value of available
    opportunities. newExperiment is a return variable of the form situationDims
    followed by heuristicDims.
    */
    bool selectExperiment(Blackboard &bb,
        std::vector<float> &newExperiment, const std::vector<float>& situation,
        float& lastOpportunityValue, std::vector<float>& currentPoint, const std::string& saveDir
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
        , int experimentType
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
                                                                              );

    /*
    Selects the best values for a given situation.
    */
    float getBest(Blackboard &bb, std::vector<float> &newExperiment,
              const std::vector<float>& situation, float& lastOpportunityValue);

    /*
    Returns the lengthscales to use when comparing situations.
    */
    Eigen::VectorXf getLengthscales() const;

private:

    // Vectors that exist to memory manage the data array around which the
    // gaussian process eigen array is wrapped.
    std::vector<float> timePenaltyDistance;
    std::vector<float> timePenaltyOffsets;
    std::vector<float> timePenaltyOffsetMults;

    // The number of trials that have been performed so far.
    int trialCount;

    // The Guassian Process model to be trained.
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    MultipleExperimentSituationSearchGaussianProcess<experimentDims,
                                                numExperiments> gaussianProcess;
#else
    SituationSearchGaussianProcess<experimentDims> gaussianProcess;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT

    std::string vec2String(std::vector<float> point, std::string delimiter=", ")
    {
        std::string pointAsString = "(";

        for(unsigned int i=0; i<point.size(); ++i)
        {
            if(i != 0)
                pointAsString += delimiter;
            std::stringstream ss(std::stringstream::in |
                                                        std::stringstream::out);
            ss << point[i];
            pointAsString += ss.str();
        }
        pointAsString += ")";

        return(pointAsString);
    }
};

/*
Adds the provided training point to the gaussian process, with the given reward
value. The trainingPoint vector is expected to be situationDims followed by
heuristicDims.
*/
template<int experimentDims, int situationDims, int heuristicDims,
                                                             int numExperiments>
void ResourceManager<experimentDims, situationDims, heuristicDims,
    numExperiments>::addTraining(const std::vector<float> &trainingPoint,
                                   const float result, const int experimentType)
{
    std::cout << "Adding training point: ";
    for(unsigned int point=0; point<trainingPoint.size(); ++point)
        std::cout << trainingPoint[point] << " ";
    std::cout << "with reward " << result;
    std::cout << std::endl;

    // The data can simply be passed on to the gaussian process.
    gaussianProcess.addTraining(trainingPoint, result, experimentType);

    // Try to tune the covariance hyperparameters.
    //if(gaussianProcess.getTrainingPointCount() > 0)
    //    gaussianProcess.optimiseCovarianceHyperparameters(experimentType);
}

/*
Selects a new experiment by carefully considering the value of available
opportunities. newExperiment is a return variable of the form situationDims
followed by heuristicDims.
*/
template<int experimentDims, int situationDims, int heuristicDims,
                                                             int numExperiments>
bool ResourceManager<experimentDims, situationDims, heuristicDims,
    numExperiments>::selectExperiment(Blackboard &bb,
    std::vector<float> &newExperiment, const std::vector<float>& situation,
    float& lastOpportunityValue, std::vector<float>& currentPoint, const std::string& saveDir
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    , int experimentType
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
                                                                               )
{
    // The value of the most valuable test point found.
    float highestTestValueEstimate = -1*FLT_MAX;

    // The value of the highest quality best point found.
    float highestBestPointValueEstimate = -1*FLT_MAX;

    // Initialise the test points.
    std::vector<float> testPoint(situation.begin(), situation.end());
    std::vector<float> testBestPoint(situation.begin(), situation.end());

    // Storage for the highest point and best point.
    std::vector<float> highestPoint;
    std::vector<float> highestBestPoint;

    // The estimated value of a unit time spent on the current step.
    float valueEstimate = FLT_MAX;

    // Timer for the various algorithm steps.
    clock_t start;

    // The total value accumulated by the current step, not time proportional.
    float totalValueEstimate = 0;

    // The total time taken by the current step.
    float totalTimeTaken = 0;

    // Variables used in output for the tester.
    float predictedMean, predictedVariance, predictedMeanBest, predictedVarianceBest;

#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    // Return for the type of experiment to be performed.
    int experimentTypeTest;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT

    // Set the situation.
    gaussianProcess.setSituation(situation);

    // lastOpportunityValue can't be allowed to be zero, as that would create
    // problems.
    if(lastOpportunityValue == 0)
        lastOpportunityValue = 0.00001f;

    // Add some zeros to the end of the points.
    for(int dim=situationDims; dim<experimentDims; ++dim)
    {
        testPoint.push_back(0);
        testBestPoint.push_back(0);
    }

    // First step is finding the best point. This lazy version checks all
    // exemplar, but time sensitivity is possible.
#ifdef DEBUG_RESOURCE_MANAGER
    start = clock();
    std::cout << "Exemplar count: " << gaussianProcess.getExemplarCount() <<
                                                                      std::endl;
#endif // DEBUG_RESOURCE_MANAGER
    for(unsigned int exemplar=0; exemplar<gaussianProcess.getExemplarCount();
                                                                     ++exemplar)
    {
        // The mean value of the latest best point found.
        float bestPointValueEstimate;

        // Get the next exemplar.
        Eigen::Matrix<float, experimentDims, 1>::Map(&testBestPoint[0],
                        experimentDims) = gaussianProcess.getExemplar(exemplar);

        // Move the exemplar to the current situation.
        for(unsigned int dim=0; dim<situation.size(); ++dim)
            testBestPoint[dim] = situation[dim];

        // Do a best point search.
        bestPointValueEstimate = gaussianProcess.findBest(testBestPoint, true);
        if(bestPointValueEstimate > highestBestPointValueEstimate)
        {
            // Update the best point.
            highestBestPointValueEstimate = bestPointValueEstimate;
            highestBestPoint = testBestPoint;
        }
    }
#ifdef DEBUG_RESOURCE_MANAGER
    totalTimeTaken = ((clock() - start)/((float)CLOCKS_PER_SEC))*1000.0f;
    std::cout << "Time taken finding best point: " << totalTimeTaken <<
                                                                      std::endl;
#endif // DEBUG_RESOURCE_MANAGER

    // Reset the value estimates.
    totalValueEstimate = 0;
    totalTimeTaken = 0;
    valueEstimate = FLT_MAX;

    // Now a better point must be found. This can be done with mathematically
    // reasonable checks as comparisons are more direct.
    while(valueEstimate > highestTestValueEstimate && valueEstimate >
                                               COMPUTATION_VALUE_BEFORE_GIVE_UP)
    {
        // The number of milliseconds taken for this step.
        float msTaken;

        // The mean value of the latest best point found.
        float testValueEstimate;

        // The improvement in the best point found.
        float testValueEstimateImprovement = 0;

        // Time the step.
        start = clock();

        // Do the experiment search.
        testValueEstimate = gaussianProcess.findValuePoint(highestBestPoint,
                  highestBestPointValueEstimate, currentPoint, testPoint
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
                  , experimentTypeTest
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
                                                                              );
        if(testValueEstimate > highestTestValueEstimate)
        {
            // Record how much of an improvement this is over the last test
            // point.
            testValueEstimateImprovement = testValueEstimate -
                                                       highestTestValueEstimate;

            // Update the test point.
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
            experimentType = experimentTypeTest;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
            highestTestValueEstimate = testValueEstimate;
            highestPoint = testPoint;
        }

        // Calculate the time taken.
        msTaken = ((clock() - start)/((float)CLOCKS_PER_SEC))*1000.0f;
        totalTimeTaken += msTaken;

        // Calculate the value gain per unit time.

        // If this is the first cycle of this loop improvement is irrelevant.
        if(valueEstimate == FLT_MAX)
        {
            if(highestTestValueEstimate > 0)
            {
                valueEstimate = highestTestValueEstimate *
                                                      (ZEROES_BEFORE_GIVE_UP+1);
            }
            else
            {
                valueEstimate = COMPUTATION_VALUE_BEFORE_GIVE_UP *
                                                      (ZEROES_BEFORE_GIVE_UP+1);
            }
            totalValueEstimate = valueEstimate*msTaken;
        }

        // Otherwise the estimate of the value of another cycle is proportional
        // to the average of past cycles.
        else
        {
            totalValueEstimate += testValueEstimateImprovement;
            valueEstimate = totalValueEstimate/totalTimeTaken;
        }
    }
#ifdef DEBUG_RESOURCE_MANAGER
    std::cout << "Time taken finding test point: " << totalTimeTaken << std::endl;
#endif // DEBUG_RESOURCE_MANAGER

    // Update the last opportunity value.
    lastOpportunityValue = highestTestValueEstimate;

    // Output the state for the trainer to assess.
    predictedMean = gaussianProcess.predictMean(highestPoint);
    predictedVariance = gaussianProcess.predictVariance(highestPoint);
    predictedMeanBest = gaussianProcess.predictMean(highestBestPoint);
    predictedVarianceBest = gaussianProcess.predictVariance(highestBestPoint);
    std::cout << "Experiment point selected for this situation: " << vec2String(highestPoint) << std::endl;
    std::cout << "Experiment reward estimate: " << predictedMean << " +- " << predictedVariance << std::endl;
    std::cout << "Point experiment value: " << highestTestValueEstimate << " = ((" << predictedMean <<
        " (experiment reward estimate) - " << highestBestPointValueEstimate << " (best point value estimate))" << " + "
        << CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS_KICK << " (confidence interval) * sqrt(" << pow(predictedVariance, 2)
        << " (squared variance) + " << pow(predictedVarianceBest, 2) << " (best point squared variance)) - " <<
        APPROXIMATION_RANGE_KICK << " (approximation range)) / " << TIME_PENALTY_CONSTANT_KICK <<
                                                                           " (value of time, unimportant)" << std::endl;
    std::cout << "Best Point: " << vec2String(highestBestPoint) << std::endl;
    std::cout << std::endl;

    //  Record data about experiment selection if requested.
    if(bb.behaviour.cleverNaoInfo.cleverNaoRecording)
    {
        // Used to check if the file already exists.
        struct stat buffer;

        // The the path to the file to record into.
        std::string filePath;

        // The file currently being output to.
        std::ofstream curFileOut;

        // The Gaussian Process hyperparameters.
        Eigen::Matrix<float, KICK_LEAN_EXPERIMENT_EXPERIMENT_DIMS+2, numExperiments> hyperparameters;

        // Record some of the gaussian process settings.
        filePath = saveDir + "/gaussianProcessSettings.txt";

        // Create the file if an earlier situation run hasn't already.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "dict" << std::endl;   // Dictionary.
            curFileOut << "confidenceInterval: " << CONFIDENCE_INTERVAL_STANDARD_DEVIATIONS_KICK << std::endl;
            curFileOut << "approximationRange: " << APPROXIMATION_RANGE_KICK << std::endl;
            curFileOut << "timePenaltyConstant: " << TIME_PENALTY_CONSTANT_KICK << std::endl;
            curFileOut.close();
        }

        // Record the hyperparameters. Order is lengthscales, base variance, noise.
        filePath = saveDir + "/hyperparameters.txt";

        // Create the file if an earlier situation run hasn't already.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
            hyperparameters = gaussianProcess.getHyperparameters();
            for(int experiment=0; experiment<numExperiments; ++experiment)
            {
                for(int hyperparameter=0; hyperparameter<KICK_LEAN_EXPERIMENT_EXPERIMENT_DIMS+2; ++hyperparameter)
                {
                    if(hyperparameter != 0)
                        curFileOut << "\t";
                    curFileOut << hyperparameters(hyperparameter, experiment);
                }
                curFileOut << std::endl;
            }
            curFileOut.close();
        }

        // Record the experiment selected for this situation.
        filePath = saveDir + "/selectedExperiments.txt";

        // Create the file and add the file type if needed.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
            curFileOut.close();
        }

        // Append this situation's selected experiment.
        curFileOut.open(filePath.c_str(), std::fstream::app|std::fstream::out);
        curFileOut << vec2String(highestPoint, "\t") << std::endl;
        curFileOut.close();

        // Record the best point found for this situation.
        filePath = saveDir + "/selectedBestPoints.txt";

        // Create the file and add the file type if needed.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
            curFileOut.close();
        }

        // Append this situation's best point found.
        curFileOut.open(filePath.c_str(), std::fstream::app|std::fstream::out);
        curFileOut << vec2String(highestBestPoint, "\t") << std::endl;
        curFileOut.close();

        // Record the reward estimate for this situation.
        filePath = saveDir + "/rewardEstimates.txt";

        // Create the file and add the file type if needed.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
            curFileOut.close();
        }

        // Append this situation's reward estimate for the experiment.
        curFileOut.open(filePath.c_str(), std::fstream::app|std::fstream::out);
        curFileOut << predictedMean << "\t" << predictedVariance << std::endl;
        curFileOut.close();

        // Record the best reward estimate for this situation.
        filePath = saveDir + "/rewardEstimatesBest.txt";

        // Create the file and add the file type if needed.
        if(!stat(filePath.c_str(), &buffer) == 0)
        {
            curFileOut.open(filePath.c_str(), std::fstream::out);
            curFileOut << "2df" << std::endl;   // 2 dimensions of floats.
            curFileOut.close();
        }

        // Append this situation's reward estimate for the best point.
        curFileOut.open(filePath.c_str(), std::fstream::app|std::fstream::out);
        curFileOut << predictedMeanBest << "\t" << predictedVarianceBest << std::endl;
        curFileOut.close();
    }

#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << "Base Variances: " << std::endl <<
          gaussianProcess.getHyperparameters().row(experimentDims) << std::endl;
#else
    std::cout << "Base Variance: " << gaussianProcess.gpVariance << std::endl;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << "How uncertain the GP is about the chosen base variance: " <<
                                                     gaussianProcess.getConfidenceInterval(experimentDims) << std::endl;
    std::cout << std::endl;
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << "Estimated noise in rewards: " << std::endl <<
                                                gaussianProcess.getHyperparameters().row(experimentDims+1) << std::endl;
#else
    std::cout << "Estimated noise in rewards: " << gaussianProcess.noise << std::endl;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << "How uncertain the GP is about the chosen reward noise level: " <<
                                                   gaussianProcess.getConfidenceInterval(experimentDims+1) << std::endl;
    std::cout << std::endl;
#ifdef GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << "Full hyperparameter list (relative scales of dimensions, base variance, noise): " << std::endl <<
                                                          gaussianProcess.getHyperparameters().transpose() << std::endl;
#endif // GAUSSIAN_PROCESS_MULTIPLE_EXPERIMENT
    std::cout << std::endl;

#ifdef GAUSSIAN_PROCESS_TEST_INFRASTRUCTURE
    returnBestDELETE = highestBestPoint;
#endif // GAUSSIAN_PROCESS_TEST_INFRASTRUCTURE

    // If an experiment worth performing has been found, add it.
    // NOTE: This is where the value of time could be used - set the threshold to more than 0 and time's value matters.
    ++trialCount;
    if(highestTestValueEstimate > 0 || trialCount < MIN_TRIALS_KICK_TIMES_SITUATIONS)
    {
#ifdef GAUSSIAN_PROCESS_TEST_INFRASTRUCTURE
        currentPoint = highestPoint;
        gaussianProcess.addTraining(highestPoint, testFunc(highestPoint));
#else
        newExperiment = highestPoint;
#endif // GAUSSIAN_PROCESS_TEST_INFRASTRUCTURE

        return(true);
    }

    // Otherwise tell the outer system there is nothing else worth trying.
    else
        return(false);
}

/*
Selects the set of variable values with the highest expected performance.
*/
template<int experimentDims, int situationDims, int heuristicDims,
                                                             int numExperiments>
float ResourceManager<experimentDims, situationDims, heuristicDims,
    numExperiments>::getBest(Blackboard &bb, std::vector<float> &newExperiment,
               const std::vector<float>& situation, float& lastOpportunityValue)
{
    // The value of the highest quality best point found.
    float highestBestPointValueEstimate = -1*FLT_MAX;

    // Initialise the test points.
    std::vector<float> testBestPoint(situation.begin(), situation.end());

    // Storage for the best point.
    std::vector<float> highestBestPoint;

#ifdef DEBUG_RESOURCE_MANAGER
    // Timer for the various algorithm steps.
    clock_t start;

    // The total time taken by the current step.
    float totalTimeTaken = 0;
#endif // DEBUG_RESOURCE_MANAGER

    // Set the situation.
    gaussianProcess.setSituation(situation);

    // lastOpportunityValue can't be allowed to be zero, as that would create
    // problems.
    if(lastOpportunityValue == 0)
        lastOpportunityValue = 0.00001f;

    // Add some zeros to the end of the points.
    for(int dim=situationDims; dim<experimentDims; ++dim)
    {
        testBestPoint.push_back(0);
    }

    // Find the best point via exemplar.
#ifdef DEBUG_RESOURCE_MANAGER
    start = clock();
    std::cout << "Exemplar count: " << gaussianProcess.getExemplarCount() <<
                                                                      std::endl;
#endif // DEBUG_RESOURCE_MANAGER
    for(unsigned int exemplar=0; exemplar<gaussianProcess.getExemplarCount();
                                                                     ++exemplar)
    {
        // The mean value of the latest best point found.
        float bestPointValueEstimate;

        // Get the next exemplar.
        Eigen::Matrix<float, experimentDims, 1>::Map(&testBestPoint[0],
                        experimentDims) = gaussianProcess.getExemplar(exemplar);

        // Move the exemplar to the current situation.
        for(unsigned int dim=0; dim<situation.size(); ++dim)
            testBestPoint[dim] = situation[dim];

        // Do a best point search.
        bestPointValueEstimate = gaussianProcess.findBest(testBestPoint, true);
        if(bestPointValueEstimate > highestBestPointValueEstimate)
        {
            // Update the best point.
            highestBestPointValueEstimate = bestPointValueEstimate;
            highestBestPoint = testBestPoint;
        }
    }
#ifdef DEBUG_RESOURCE_MANAGER
    totalTimeTaken = ((clock() - start)/((float)CLOCKS_PER_SEC))*1000.0f;
    std::cout << "Time taken finding best point: " << totalTimeTaken <<
                                                                      std::endl;
#endif // DEBUG_RESOURCE_MANAGER

    // Return the best experiment.
    newExperiment = highestBestPoint;
    return(highestBestPointValueEstimate);
}

/*
Returns the lengthscales to use when comparing situations.
*/
template<int experimentDims, int situationDims, int heuristicDims,
                                                             int numExperiments>
Eigen::VectorXf ResourceManager<experimentDims, situationDims, heuristicDims,
                                       numExperiments>:: getLengthscales() const
{
    return(gaussianProcess.getLengthscales());
}

#endif /* end of include guard: RESOURCE_MANAGER_H */
