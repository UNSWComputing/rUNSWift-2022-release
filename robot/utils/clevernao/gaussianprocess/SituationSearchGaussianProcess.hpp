#include "GaussianProcess.hpp"

#include <stdlib.h>
#include <numeric>
#include <algorithm>

// Multiplier for the derivative to determine axis movement at each hill climb
// step.
#define SITUATION_SEARCH_HILL_CLIMB_STEP_SIZE 0.01f

// The maximum allowed number of hill climb steps.
#define SITUATION_SEARCH_HILL_CLIMB_MAX_STEPS 100

// How "flat" the steepest axis must be for hill climb to terminate.
#define SITUATION_SEARCH_MIN_CLIMB_STEEPNESS 0.01f

// How different a point needs to be from other exemplars to become an exemplar.
#define SITUATION_SEARCH_EXEMPLAR_COVARIANCE_THRESHOLD 0.5f

// Debug flags.

// Show the sub derivatives calculated in getFullDerivative.
//#define DEBUG_SITUATION_SEARCH_REPORT_SUB_DERIVATIVES

// Show derivatives before and after each major calculation.
//#define DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES

// Enable extra error checking.
//#define DEBUG_SITUATION_SEARCH_EXTRA_ERROR_CHECKS

// Show detailed debug logs.
//#define DEBUG_SITUATION_SEARCH_EXTRA_LOGS

// Translate so the dumb compiler can work out I want the parent classes'
// variables, without making the code horrible with the use of
// "GaussianProcess<numDimensions>::" everywhere.

// Variables.
#define trainingPoints (*GaussianProcess<numDimensions>::trainingPoints)
#define trainingWeights GaussianProcess<numDimensions>::trainingWeights
#define covariance GaussianProcess<numDimensions>::covariance
#define lengthScales GaussianProcess<numDimensions>::lengthScales
#define gpVariance GaussianProcess<numDimensions>::gpVariance
#define trainingResults (*GaussianProcess<numDimensions>::trainingResults)
#define confidenceIntervalStandardDeviations GaussianProcess<numDimensions>::confidenceIntervalStandardDeviations

/*
A SituationSearchGaussianProcess has quite a few parameters. This struct
provides a way to pass them all succinctly.
*/
struct SituationSearchGaussianProcessParams
{
    SituationSearchGaussianProcessParams(GPFunc* mean,
        const float gpVar, const float noise, const float baseLengthScales,
        const float confidenceIntervalStandardDeviationsVal,
        const float approximationConst,
        std::vector<float>& timePenaltyDistanceVals,
        std::vector<float>& timePenaltyOffsetVals,
        std::vector<float>& timePenaltyOffsetMultVals,
        const float timePenaltyConst) :
        mean(mean), gpVar(gpVar), noise(noise),
        baseLengthScales(baseLengthScales),
        confidenceIntervalStandardDeviationsVal(
                                       confidenceIntervalStandardDeviationsVal),
        approximationConst(approximationConst),
        timePenaltyDistanceVals(timePenaltyDistanceVals),
        timePenaltyOffsetVals(timePenaltyOffsetVals),
        timePenaltyOffsetMultVals(timePenaltyOffsetMultVals),
                                           timePenaltyConst(timePenaltyConst) {}

    GPFunc* mean;
    float gpVar;
    float noise;
    float baseLengthScales;
    float confidenceIntervalStandardDeviationsVal;
    float approximationConst;
    std::vector<float> timePenaltyDistanceVals;
    std::vector<float> timePenaltyOffsetVals;
    std::vector<float> timePenaltyOffsetMultVals;
    float timePenaltyConst;
};

/*
An extension of gaussian process that adds functions specific to a system with
a divide between situation and heuristic dimensions. Essentially some dimensions
can be locked, then the later dimensions searched through to find optimal
values.
*/
template<int numDimensions>
class SituationSearchGaussianProcess : public GaussianProcess<numDimensions>
{

public:

    /*
    Multiple experiment gaussian process needs to combine mean and variance
    funtions, so needs access to the private calls used to calculate these.
    */
    template<int dimensions, int numExperiments>
    friend class MultipleExperimentSituationSearchGaussianProcess;

    /*
    Create a new situation search gaussian process, primarily just recording the
    mean and variance functions and initialising as needed. Note that mean
    should be a persistant object, and will be freed by GuassianProcess
    automatically when it is destroyed. gpVariance is the amount of variance
    used in the covariance function, and noise is the amount of expected noise
    in the training sample values. *DistanceVals are the multipliers for each
    dimension in calculating the distance based time and resource penalties.
    */
    SituationSearchGaussianProcess(SituationSearchGaussianProcessParams& params)
                  : GaussianProcess<numDimensions>(params.mean,
                    params.gpVar, params.noise, params.baseLengthScales,
                    params.confidenceIntervalStandardDeviationsVal),
                    approximationConst(params.approximationConst),
                    timePenaltyDistance(params.timePenaltyDistanceVals.data()),
                    timePenaltyOffsets(params.timePenaltyOffsetVals.data()),
                    timePenaltyOffsetMults(
                                       params.timePenaltyOffsetMultVals.data()),
                                   timePenaltyConst(params.timePenaltyConst) {};

    /*
    Create a new situation search gaussian process, primarily just recording the
    mean and variance functions and initialising as needed. Note that mean
    should be a persistant object, and will be freed by GuassianProcess
    automatically when it is destroyed. gpVariance is the amount of variance
    used in the covariance function, and noise is the amount of expected noise
    in the training sample values. *DistanceVals are the multipliers for each
    dimension in calculating the distance based time and resource penalties.
    */
    SituationSearchGaussianProcess(GPFunc* mean,
                        const float gpVar, const float noise,
                        const float baseLengthScales,
                        const float confidenceIntervalStandardDeviationsVal,
                        const float approximationConst,
                        std::vector<float>& timePenaltyDistanceVals,
                        std::vector<float>& timePenaltyOffsetVals,
                        std::vector<float>& timePenaltyOffsetMultVals,
                        const float timePenaltyConst) :
                        GaussianProcess<numDimensions>(mean, gpVar, noise,
                                       baseLengthScales,
                                       confidenceIntervalStandardDeviationsVal),
                        approximationConst(approximationConst),
                        timePenaltyDistance(timePenaltyDistanceVals.data()),
                        timePenaltyOffsets(timePenaltyOffsetVals.data()),
                        timePenaltyOffsetMults(
                                              timePenaltyOffsetMultVals.data()),
                                          timePenaltyConst(timePenaltyConst) {};

    /*
    Clean up the exemplars.
    */
    ~SituationSearchGaussianProcess()
    {
        for(unsigned int exemplar=0; exemplar<exemplarMean.size(); ++exemplar)
            delete exemplarMean[exemplar];
    }

    /*
    Sets a "situation" in which future searches will be performed.
    */
    void setSituation(const std::vector<float>& newSituation);

    /*
    Attempts to find the "best" (highest value) sample values for the current
    situation. Performs hill climbing, initialised from best if
    initialiseFromGivenPoint is true, or one random location otherwise. best is
    a return variable that will contain the full set of values for the best
    (local maxima) sample point found.
    */
    float findBest(std::vector<float> &best,
                                     bool initialiseFromGivenPoint=false) const;

    /*
    Attempts to find the most valuable point against the best point given.
    Performs hill climbing, initialised from one random location. best is the
    best point known for the area, value is a return variable for the best point
    found.
    */
    float findValuePoint(const std::vector<float>& best, const float bestValue,
               std::vector<float>& value, const std::vector<float>& currentVec);

    /*
    Calculates the value of a particular point in the situation, using the full
    value equation. point should be the full point, including situation. best is
    the best point known for this situation. bestVec is best as a vector.
    derivative is a return variable.
    */
    float getFullValue(const std::vector<float>& pointVec,
                       const std::vector<float>& bestVec,
                                          const std::vector<float>& currentVec);

    /*
    Gets the number of exemplar points for searching the gaussian process.
    */
    inline unsigned int getExemplarCount()
    {
        return(exemplarMean.size());
    }

    /*
    Gets one of the exemplar points for searching the gaussian process.
    */
    inline const Eigen::Matrix<float, numDimensions, 1>&
                                                       getExemplar(int exemplar)
    {
        return(exemplarMean[exemplar]->second);
    }

#ifdef ENABLE_GRAPHING
    void drawFullValueFunction(std::vector<float> lockedValues,
        const int dimension, const int numSamples, const std::string title,
        const std::vector<float>& bestPoint, const std::vector<float>& current);
#endif // ENABLE_GRAPHING
#ifndef ENABLE_GRAPHING
private:
#endif // not ENABLE_GRAPHING

    /*
    The set of values that compose the current situation.
    */
    std::vector<float> situation;

    /*
    The locked exponent values that correspond to this situation, one per point.
    */
    std::vector<float> situationExpVals;

    /*
    Additative value to apply to the approximation bounds.
    */
    const float approximationConst;

    /*
    The set of multipliers for the dimensions used to calculate the distance
    based portion of the time penalty.
    */
    Eigen::Matrix<float, numDimensions, 1> timePenaltyDistance;

    /*
    The set of offset from which the offset based portion of the time penalty is
    calculated.
    */
    Eigen::Matrix<float, numDimensions, 1> timePenaltyOffsets;

    /*
    The set of multipliers for the dimensions used to calculate the offset based
    portion of the time penalty.
    */
    Eigen::Matrix<float, numDimensions, 1> timePenaltyOffsetMults;

    /*
    Additative value to apply to the time normalisation penalty.
    */
    const float timePenaltyConst;

    /*
    The set of key "exemplar" points for the gaussian process, in terms of mean
    value.
    */
    std::vector<std::pair<int, Eigen::Matrix<float, numDimensions, 1> >* >
                                                                   exemplarMean;

    /*
    Add a new variable to the training set of the gaussian process. Note that
    axis normalisation is not currently considered.
    */
    void addTraining(const std::vector<float> &values, const float result);

    /*
    Calculates the derivative of a particular point in the situation. point
    should be the full point, including situation. derivative is a return
    variable.
    */
    void getMeanDerivative(const Eigen::Matrix<float, numDimensions, 1>& point,
                      Eigen::Matrix<float, numDimensions, 1>& derivative) const;

    /*
    Calculates the derivative of a particular point in the situation, including
    the full value equation. point should be the full point, including
    situation. best is the best point known for this situation. bestVec is best
    as a vector. derivative is a return variable.
    */
    void getFullDerivative(const Eigen::Matrix<float, numDimensions, 1>& point,
                          const Eigen::Matrix<float, numDimensions, 1>& best,
                          const std::vector<float> bestVec,
                          const Eigen::Matrix<float, numDimensions, 1>& current,
                            Eigen::Matrix<float, numDimensions, 1>& derivative);

    /*
    Calculates the value of the derivative of the variance for a particular
    point.
    */
    void getVarianceDerivative(const
                      Eigen::Matrix<float, numDimensions, 1>& point,
                      Eigen::Matrix<float, numDimensions, 1>& derivative) const;

    /*
    Gets all the results of the exponent calculations for each training point
    against the given point.
    */
    Eigen::VectorXf getExponents(
                     const Eigen::Matrix<float, numDimensions, 1>& point) const;

    /*
    Calculates a single exponential value, for a single dimension. Note that the
    full exponential value is exp(a*-1), where a is the sum of this over all
    dimensions.
    */
    inline float getExponent(const float pointA, const float pointB, const
                                                       float lengthScale) const;

    /*
    Calculates a single derivative multiplier value, for a single dimension. Note
    that the full derivative multiplier is the sum of this over all dimensions.
    */
    inline float getDerivativeMultiplier(const float pointA, const float pointB,
                                                 const float lengthScale) const;

    /*
    Calculates the derivative of the resource normalisation value for a given
    point.
    */
    void getNormalisationDerivative(
                    const Eigen::Matrix<float, numDimensions, 1>& point,
                    const Eigen::Matrix<float, numDimensions, 1>& current,
                      Eigen::Matrix<float, numDimensions, 1>& derivative) const;

    /*
    Calculates the value of the resource normalisation value for a given point.
    */
    float getNormalisation(const Eigen::Matrix<float, numDimensions, 1>& point,
                   const Eigen::Matrix<float, numDimensions, 1>& current) const;
};

/*
Calculates a single exponential value, for a single dimension. Note that the
full exponential value is exp(a*-1), where a is the sum of this over all
dimensions.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::getExponent(const
                float pointA, const float pointB, const float lengthScale) const
{
    return(pow(pointA - pointB, 2) / (2*pow(lengthScale, 2)));
}

/*
Calculates a single derivative multiplier value, for a single dimension. Note
that the full derivative multiplier is the sum of this over all dimensions.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::getDerivativeMultiplier(
          const float pointA, const float pointB, const float lengthScale) const
{
    return((pow(gpVariance, 2) * (pointA - pointB)) / pow(lengthScale, 2));
}

/*
Sets a "situation" in which future searches will be performed.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::setSituation(
                                         const std::vector<float> &newSituation)
{
    // Buffer the number of training points.
    const int trainingPointCount = trainingPoints.rows();

    // Clear space for the new situation values.
    situationExpVals.clear();

    // Record the situation.
    situation = newSituation;

    // Set the new situation values.
    for(int trainingPoint=0; trainingPoint<trainingPointCount;
                                                                ++trainingPoint)
    {
        // Array functions could do this all in one nice line if this were a
        // modern version of eigen.
        situationExpVals.push_back(0);
        for(unsigned int col=0; col<newSituation.size(); ++col)
        {
            // The sample's rows map to trainingPoint's cols.
            situationExpVals[trainingPoint] += getExponent(
                                trainingPoints(trainingPoint, col),
                                          situation[col], lengthScales(col, 0));
        }
    }
}

/*
Attempts to find the "best" (highest value) sample values for the current
situation. Performs hill climbing, initialised from best if
initialiseFromGivenPoint is true, or one random location otherwise. best is a
return variable that will contain the full set of values for the best (local
maxima) sample point found.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::findBest(
                  std::vector<float> &best, bool initialiseFromGivenPoint) const
{
    // Stores the derivative of the point currently being examined.
    Eigen::Matrix<float, numDimensions, 1> derivative;

    // The current point in the search.
    Eigen::Matrix<float, numDimensions, 1> point;

    // The number of hill climb steps that have been run.
    int steps = 0;

    // If requested use the given point for the search.
    if(initialiseFromGivenPoint)
    {
        point = Eigen::Map<Eigen::Matrix<float, numDimensions, 1> >(&best[0],
                                                                 numDimensions);
    }

    // Otherwise populate a point for the search.
    else
    {
        for(unsigned int dim=0; dim<numDimensions; ++dim)
        {
            if(dim<situation.size())
            {
                point(dim, 0) = situation[dim];
                derivative(dim, 0) = 0;
            }
            else
                point(dim, 0) = (float)rand()/(float)RAND_DIVISOR;
        }
    }

    // Now hill climb that point. Implementation is vulnerable to flat non
    // maxima areas.
    do
    {
#ifdef DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES
        std::cout << "Derivative before (basic): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES

        // Calculate the derivative of this point.
        getMeanDerivative(point, derivative);

#ifdef DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES
        std::cout << "Derivative after (basic): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES

#ifdef DEBUG_SITUATION_SEARCH_EXTRA_ERROR_CHECKS
        for(unsigned int dim=0; dim<situation.size(); ++dim)
        {
            if(derivative[dim] != 0)
            {
                std::cout << "MOVEMENT ON LOCKED AXIS (BASIC): " <<
                                                     derivative[0] << std::endl;
            }
        }
#endif // DEBUG_SITUATION_SEARCH_EXTRA_ERROR_CHECKS

        // Make the step, but don't allow dimension values to go out of range.
        for(unsigned int dim=situation.size(); dim<numDimensions; ++dim)
        {
            point(dim, 0) += copysign(SITUATION_SEARCH_HILL_CLIMB_STEP_SIZE,
                                                               derivative[dim]);
            point(dim, 0) = std::min(1.0f, std::max(0.0f, point(dim, 0)));
        }

        // Step complete.
        ++steps;
    }
    while((*std::max_element(derivative.data(), derivative.data() +
                   derivative.size()) > SITUATION_SEARCH_MIN_CLIMB_STEEPNESS ||
                   fabs(*std::min_element(derivative.data(), derivative.data() +
                   derivative.size())) > SITUATION_SEARCH_MIN_CLIMB_STEEPNESS)
                              && steps < SITUATION_SEARCH_HILL_CLIMB_MAX_STEPS);

    // Return the point found and its value.
    best.clear();
    for(int dim=0; dim<numDimensions; ++dim)
        best.push_back(point(dim, 0));
    return(GaussianProcess<numDimensions>::predictMean(best));
}

/*
Attempts to find the most valuable point against the best point given.
Performs hill climbing, initialised from one random location. best is the
best point known for the area, bestValue is the value of that point, value is a
return variable for the best point found.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::findValuePoint(
    const std::vector<float>& bestVec, const float bestValue,
                std::vector<float>& value, const std::vector<float>& currentVec)
{
    // Stores the derivative of the point currently being examined.
    Eigen::Matrix<float, numDimensions, 1> derivative;

    // The current point in the search.
    Eigen::Matrix<float, numDimensions, 1> point;

    // The best point in the search.
    const Eigen::Matrix<float, numDimensions, 1> best(bestVec.data());

    // The current point of the searcher.
    const Eigen::Matrix<float, numDimensions, 1> current(currentVec.data());

    // The number of hill climb steps that have been run.
    int steps = 0;

    // Populate a point for the search.
    for(unsigned int dim=0; dim<numDimensions; ++dim)
    {
        if(dim<situation.size())
        {
            point(dim, 0) = situation[dim];
            derivative(dim, 0) = 0;
        }
        else
            point(dim, 0) = (float)rand()/(float)RAND_DIVISOR;
    }

    // Now hill climb that point. Implementation is vulnerable to flat non
    // maxima areas.
    do
    {
#ifdef DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES
        std::cout << "Derivative before (full): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES

        // Calculate the derivative of this point.
        getFullDerivative(point, best, bestVec, current, derivative);

#ifdef DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES
        std::cout << "Derivative after (full): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_SITUATION_SEARCH_SHOW_DERIVATIVES

#ifdef DEBUG_SITUATION_SEARCH_EXTRA_ERROR_CHECKS
        for(unsigned int dim=0; dim<situation.size(); ++dim)
        {
            if(derivative[dim] != 0)
            {
                std::cout << "MOVEMENT ON LOCKED AXIS (full): " <<
                                                   derivative[dim] << std::endl;
            }
        }
#endif // DEBUG_SITUATION_SEARCH_EXTRA_ERROR_CHECKS

        // Make the step, but don't allow dimension values to go out of range.
        for(unsigned int dim=situation.size(); dim<numDimensions; ++dim)
        {
            point(dim, 0) += copysign(SITUATION_SEARCH_HILL_CLIMB_STEP_SIZE,
                                                               derivative[dim]);
            point(dim, 0) = std::min(1.0f, std::max(0.0f, point(dim, 0)));
        }

        // Step complete.
        ++steps;
    }
    while((*std::max_element(derivative.data(), derivative.data() +
                   derivative.size()) > SITUATION_SEARCH_MIN_CLIMB_STEEPNESS ||
                   fabs(*std::min_element(derivative.data(), derivative.data() +
                   derivative.size())) > SITUATION_SEARCH_MIN_CLIMB_STEEPNESS)
                              && steps < SITUATION_SEARCH_HILL_CLIMB_MAX_STEPS);

    // Return the point found and its value.
    value.clear();
    for(int i=0; i<numDimensions; ++i)
        value.push_back(point(i, 0));
    return(getFullValue(value, bestVec, currentVec));
}

/*
Calculates the derivative of a particular point in the situation, including the
full value equation. point should be the full point, including situation. best
is the best point known for this situation. bestVec is best as a vector.
derivative is a return variable.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::getFullDerivative(
               const Eigen::Matrix<float, numDimensions, 1>& point,
               const Eigen::Matrix<float, numDimensions, 1>& best,
               const std::vector<float> bestVec,
               const Eigen::Matrix<float, numDimensions, 1>& current,
                             Eigen::Matrix<float, numDimensions, 1>& derivative)
{
    // This function implements some pretty complex math that doesn't always
    // have an intuitive meaning. It's not likely to be easy to follow. If you
    // wish to understand it look at the equations at: TODO

    // Map point and best into vectors for calls to predictMean and
    // predictVariance.
    const std::vector<float> pointVec(point.data(), point.data()+point.size());

    // The mean and variance values for best.
    const float meanBest = GaussianProcess<numDimensions>::predictMean(bestVec);
    const float varianceBest =
                       GaussianProcess<numDimensions>::predictVariance(bestVec);

    // The mean and covariance of the point being sampled.
    const float mean = GaussianProcess<numDimensions>::predictMean(pointVec);
    const float variance =
                      GaussianProcess<numDimensions>::predictVariance(pointVec);

    // The derivatives of the mean and covariance of the point being sampled.
    Eigen::Matrix<float, numDimensions, 1> meanDerivative;
    Eigen::Matrix<float, numDimensions, 1> varianceDerivative;

    // The sum of the squared variances.
    const float sumSquaredVariance = pow(variance, 2) + pow(varianceBest, 2);

    // The square root of the sum of the squared variances.
    const float sqrtSumSquaredVariance = sqrt(sumSquaredVariance);

    // The difference between the point mean and best mean.
    const float meanDifference = mean-meanBest;

    // The derivative of the combined variance of point and best.
    Eigen::Matrix<float, numDimensions, 1> combinedVarianceDerivative; // u'(V)

    // The normalisation / divider penalty applied to this point. Expected to
    // be the time, resource and risk costs.
    float normPenaltyVal = 0;

    // The derivative of the normalisation penalty.
    Eigen::Matrix<float, numDimensions, 1> normPenaltyDerivative;

    // Initialise mean and variance derivatives to 0 in relevant dimensions.
    for(unsigned int dim=0; dim<situation.size(); ++dim)
    {
        meanDerivative(dim, 0) = 0;
        varianceDerivative(dim, 0) = 0;
        normPenaltyDerivative(dim, 0) = 0;
    }

    // Get the mean and variance derivatives.
    getMeanDerivative(point, meanDerivative);
    getVarianceDerivative(point, varianceDerivative);

    // Calculate the derivative of the variance of the difference between the
    // point and the best point.
    combinedVarianceDerivative = (variance/sqrtSumSquaredVariance) *
                                                             varianceDerivative;

#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    std::cout << "Combined Variance Derivative: " << std::endl;
    std::cout << combinedVarianceDerivative << std::endl << std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS

    // Calculate the normalisation penalty.
    normPenaltyVal = getNormalisation(point, current);

    // Calculate the normalisation derivative.
    getNormalisationDerivative(point, current, normPenaltyDerivative);

#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    std::cout << "Normalisation Penalty: " << normPenaltyVal << std::endl;
    std::cout << "Normalisation Penalty Derivative: " << std::endl;
    std::cout << normPenaltyDerivative << std::endl << std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS

#ifdef DEBUG_SITUATION_SEARCH_REPORT_SUB_DERIVATIVES
    std::cout << "meanDerivative: " << meanDerivative.transpose() << std::endl;
    std::cout << "varianceDerivative: " << varianceDerivative.transpose() <<
                                                                      std::endl;
    std::cout << "combinedVarianceDerivative: " <<
                            combinedVarianceDerivative.transpose() << std::endl;
    std::cout << "normPenaltyDerivative: " << normPenaltyDerivative.transpose()
                                                                   << std::endl;
#endif // DEBUG_SITUATION_SEARCH_REPORT_SUB_DERIVATIVES

    // Calculate the derivative.
    derivative = ((meanDerivative + confidenceIntervalStandardDeviations *
                  combinedVarianceDerivative) /
                  normPenaltyVal) + ((approximationConst - meanDifference -
                  confidenceIntervalStandardDeviations * sqrtSumSquaredVariance)
                              / pow(normPenaltyVal, 2)) * normPenaltyDerivative;
}

/*
Calculates the derivative of a particular point in the situation. point should
be the full point, including situation. derivative is a return variable.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::getMeanDerivative(const
                       Eigen::Matrix<float, numDimensions, 1>& point,
                       Eigen::Matrix<float, numDimensions, 1>& derivative) const
{
    // Zero situation dimensions.
    for(unsigned int dim=0; dim<situation.size(); ++dim)
        derivative(dim, 0) = 0;

    // Calculate the slope for each dimension.
    for(int dim=situation.size(); dim<numDimensions; ++dim)
    {
        // The set of exponent values corresponding to each training point.
        Eigen::VectorXf exponents = getExponents(point);

        // Each derivative is based on every training point.
        derivative(dim, 0) = 0;
        for(int trainingPoint=0; trainingPoint < trainingPoints.rows();
                                                                ++trainingPoint)
        {
            // Calculate this point's part of the sum for this derivative
            // dimension.
            derivative(dim, 0) += getDerivativeMultiplier(
                                  trainingPoints(trainingPoint, dim),
                                  point(dim, 0), lengthScales(dim, 0)) *
                                  trainingWeights(trainingPoint, 0) *
                                                    exponents(trainingPoint, 0);
        }
    }
}

/*
Calculates the value of the derivative of the variance for a particular point.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::getVarianceDerivative(const
                       Eigen::Matrix<float, numDimensions, 1>& point,
                       Eigen::Matrix<float, numDimensions, 1>& derivative) const
{
    // The number of training points.
    const int trainingPointCount = trainingPoints.rows();

    // The set of exponent values corresponding to each training point.
    Eigen::VectorXf exponents = getExponents(point);

    // The outer set of multipliers for the system.
    Eigen::VectorXf outerMults(trainingPointCount);

    // The difference between the points by the relevant exponent.
    Eigen::VectorXf diffByExp(trainingPointCount);

    // The difference between the points by the outer multipliers.
    Eigen::VectorXf diffByOuter(trainingPointCount);

    // The outer set of multipliers for the system times the inverse covariance.
    Eigen::VectorXf invCovOuterMults;

    // Diff times outer mults for the system times the inverse covariance.
    Eigen::VectorXf invCovDiffByOuter;

    /*
    // Inverse covariance.
    Eigen::MatrixXf invCovariance;
    Eigen::MatrixXf identity = Eigen::MatrixXf::Identity(covariance.rows(),
                                                             covariance.cols());
    invCovariance = covariance.ldlt().solve(identity);
    //std::cout << "Inverse covariance: " << std::endl << invCovariance <<
                                                         std::endl << std::endl;
    */

    // Zero situation dimensions.
    for(unsigned int dim=0; dim<situation.size(); ++dim)
        derivative(dim, 0) = 0;

    // Calculate the derivative for each non situation dimension.
    for(int dim=situation.size(); dim<numDimensions; ++dim)
    {
        // Calculate outerMults.
        outerMults = (pow(gpVariance, 4)/pow(lengthScales(dim, 0), 2)) *
                                                                      exponents;

        // Calculate inverse covariance times outerMults.
        invCovOuterMults = covariance.ldlt().solve(outerMults);

        // Calculate point difference times exponent.
        for(int trainingPoint=0; trainingPoint<trainingPointCount;
                                                                ++trainingPoint)
        {
            diffByExp(trainingPoint, 0) = (trainingPoints(trainingPoint, dim) -
                                     point(dim, 0))*exponents(trainingPoint, 0);
        }

        // Calculate point difference times outer mults.
        for(int trainingPoint=0; trainingPoint<trainingPointCount;
                                                                ++trainingPoint)
        {
            diffByOuter(trainingPoint, 0) = (trainingPoints(trainingPoint, dim)
                                  - point(dim, 0))*outerMults(trainingPoint, 0);
        }

        // Calcuate inverse covariance times diffByExp.
        invCovDiffByOuter = covariance.ldlt().solve(diffByOuter);

        // Calculate the derivative for this point.
        derivative(dim, 0) = -1*(diffByExp.transpose()*invCovOuterMults +
                                  exponents.transpose()*invCovDiffByOuter)(0,0);
    }
}

/*
Gets all the results of the exponent calculations for each training point
against the given point.
*/
template<int numDimensions>
Eigen::VectorXf SituationSearchGaussianProcess<numDimensions>::getExponents(
                      const Eigen::Matrix<float, numDimensions, 1>& point) const
{
    // The number of training points in the system.
    const int trainingPointCount = trainingPoints.rows();

    // The set of training point exponents calculated.
    Eigen::VectorXf exponents(trainingPointCount);

    // Run through calculating the values.
#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    std::cout << "Split Exponents: " << std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    for(int trainingPoint=0; trainingPoint<trainingPointCount; ++trainingPoint)
    {
        // Calculate the exponential multiplier.
        exponents(trainingPoint, 0) = situationExpVals[trainingPoint];
#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
        std::cout << situationExpVals[trainingPoint] << " ";
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS
        for(int col=situation.size(); col<numDimensions; ++col)
        {
            exponents(trainingPoint, 0) += getExponent(trainingPoints(
                      trainingPoint, col), point(col, 0), lengthScales(col, 0));
#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
            std::cout << getExponent(trainingPoints(
                trainingPoint, col), point(col, 0), lengthScales(col, 0)) <<
                                                                            " ";
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS
        }
#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
        std::cout << std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS

        // Calculate this training point's exponent value.
        exponents(trainingPoint, 0) = exp(exponents(trainingPoint, 0)*-1);
    }
#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    std::cout << std::endl;

    std::cout << "Exponents: " << std::endl << exponents << std::endl <<
                                                                      std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS

    // Return the calculated set of exponent values.
    return exponents;
}

/*
Calculates the value of a particular point in the situation, using the full
value equation. point should be the full point, including situation. best is the
best point known for this situation. bestVec is best as a vector. derivative is
a return variable.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::getFullValue(
    const std::vector<float>& pointVec, const std::vector<float>& bestVec,
                                           const std::vector<float>& currentVec)
{
    // Convert point and best to eigen matrix.
    const Eigen::Matrix<float, numDimensions, 1> point(pointVec.data());
    const Eigen::Matrix<float, numDimensions, 1> best(bestVec.data());
    const Eigen::Matrix<float, numDimensions, 1> current(currentVec.data());

    // The mean and variance values for best.
    const float meanBest = GaussianProcess<numDimensions>::predictMean(bestVec);
    const float varianceBest =
                       GaussianProcess<numDimensions>::predictVariance(bestVec);

    // The mean and covariance of the point being sampled.
    const float mean = GaussianProcess<numDimensions>::predictMean(pointVec);
    const float variance =
                      GaussianProcess<numDimensions>::predictVariance(pointVec);

    // The sum of the squared variances.
    const float sumSquaredVariance = pow(variance, 2) + pow(varianceBest, 2);

    // The normalisation / divider penalty applied to this point. Expected to
    // be the time cost.
    float normPenaltyVal = 0;

    // Calculate the normalisation penalty.
    normPenaltyVal = getNormalisation(point, current);

#ifdef DEBUG_SITUATION_SEARCH_EXTRA_LOGS
    std::cout << "Norm penalty: " << normPenaltyVal << std::endl;
    std::cout << "Combined variance: " << sqrt(sumSquaredVariance) << std::endl;
    std::cout << "Confidence interval: " << confidenceIntervalStandardDeviations
                                        * sqrt(sumSquaredVariance) << std::endl;
    std::cout << std::endl;
#endif // DEBUG_SITUATION_SEARCH_EXTRA_LOGS

    // Calculate the point value.
    return(((mean-meanBest) + confidenceIntervalStandardDeviations *
               sqrt(sumSquaredVariance) - approximationConst) / normPenaltyVal);
}

/*
Add a new variable to the training set of the gaussian process. Note that
axis normalisation is not currently considered.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::addTraining(
                           const std::vector<float> &values, const float result)
{
    // Add a training point as in GaussianProcess.
    GaussianProcess<numDimensions>::addTraining(values, result);

    // The number of training points.
    const int trainingPointCount = trainingPoints.rows();

    // A point becomes an exemplar if it is sufficiently different from the
    // existing exemplar, or higher than them.
    bool makeNewExemplar = true;
    for(unsigned int exemplar=0; exemplar<exemplarMean.size(); ++exemplar)
    {
        // Check if this point is similar to the existing exemplar.
        if(covariance(exemplarMean[exemplar]->first, trainingPointCount-1) /
                pow(gpVariance, 2) <
                                 SITUATION_SEARCH_EXEMPLAR_COVARIANCE_THRESHOLD)
            makeNewExemplar = false;

        // If this point is similar to the existing exemplar and has a higher
        // result, replace it.
        if(!makeNewExemplar &&
                     trainingResults(exemplarMean[exemplar]->first, 0) < result)
        {
            // Alter the point.
            exemplarMean[exemplar]->first = trainingPointCount-1;
            exemplarMean[exemplar]->second =
                                       trainingPoints.row(trainingPointCount-1);

            // Bubble sort the point up.
            std::pair<int, Eigen::Matrix<float, numDimensions, 1> >* temp;
            while(exemplar > 0 &&
                    trainingResults(exemplarMean[exemplar]->first, 0) >
                            trainingResults(exemplarMean[exemplar-1]->first, 0))
            {
                temp = exemplarMean[exemplar];
                exemplarMean[exemplar] = exemplarMean[exemplar-1];
                exemplarMean[exemplar-1] = temp;
            }

            // The new exemplar has been added, don't add it again.
            break;
        }
    }

    // If this is a new exemplar with no existing nearby equivalent add it.
    if(makeNewExemplar)
    {
        // Index used when inserting the new exemplar.
        unsigned int exemplar=0;

        // The new exemplar.
        std::pair<int, Eigen::Matrix<float, numDimensions, 1> >*
            newExemplar = new std::pair<int,
            Eigen::Matrix<float, numDimensions, 1> >(
                trainingPointCount-1, trainingPoints.row(trainingPointCount-1));

        // Find the insertion point for the new exemplar.
        while(exemplar<exemplarMean.size() &&
                trainingResults(trainingPointCount-1, 0) <
                              trainingResults(exemplarMean[exemplar]->first, 0))
            ++exemplar;

        // Insert the new point.
        exemplarMean.insert(exemplarMean.begin()+exemplar, newExemplar);
    }
}

/*
Calculates the derivative of the resource normalisation value for a given point.
derivative is a return variable.
*/
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::getNormalisationDerivative(
                    const Eigen::Matrix<float, numDimensions, 1>& point,
                    const Eigen::Matrix<float, numDimensions, 1>& current,
                       Eigen::Matrix<float, numDimensions, 1>& derivative) const
{
    // The square root of the sum of the squared differences between the target
    // point and the current situation.
    float rootSumSquaredDifferencesTime = 0;

    // Calculate the axis weighted euclidean distance between the samples.
    for(int dimension=0; dimension<numDimensions; ++dimension)
    {
        rootSumSquaredDifferencesTime += timePenaltyDistance(dimension, 0) *
                            pow(current(dimension, 0) - point(dimension, 0), 2);
    }

    // The dividing factor cannot be allowed to be 0.
    if(rootSumSquaredDifferencesTime > 0)
        rootSumSquaredDifferencesTime = sqrt(rootSumSquaredDifferencesTime);
    else
        rootSumSquaredDifferencesTime = 1;

    // Calculate the overall derivative.
    for(int dimension=situation.size(); dimension<numDimensions; ++dimension)
    {
        // The time penalty from distance for this dimension.
        float timePenaltyDistanceVal = -1*(timePenaltyDistance(dimension, 0) *
                                (current(dimension, 0) - point(dimension, 0)))
                                                / rootSumSquaredDifferencesTime;

        // The time penalty from offset for this dimension.
        float timePenaltyOffsetVal = timePenaltyOffsetMults(dimension, 0);
        if(point(dimension, 0) < timePenaltyOffsets(dimension, 0))
            timePenaltyOffsetVal *= -1;

        // Calculate the derivative.
        derivative(dimension, 0) = timePenaltyDistanceVal+timePenaltyOffsetVal;
    }
}

/*
Calculates the value of the resource normalisation value for a given point.
*/
template<int numDimensions>
float SituationSearchGaussianProcess<numDimensions>::getNormalisation(
                    const Eigen::Matrix<float, numDimensions, 1>& point,
                    const Eigen::Matrix<float, numDimensions, 1>& current) const
{
    // The square root of the sum of the squared differences between the target
    // point and the current situation.
    float rootSumSquaredDifferencesTime = 0;

    // The weighted sum of the absolute differences from the offsets.
    float weightedDiffs = 0;

    // The final normalisation penalty.
    float normPenaltyVal;

    // Calculate the axis weighted euclidean distance between the samples, and
    // the weighted sum of the absolute differences from the offsets.
    for(int dimension=0; dimension<numDimensions; ++dimension)
    {
        float difference = current(dimension, 0) - point(dimension, 0);
        rootSumSquaredDifferencesTime += timePenaltyDistance(dimension, 0) *
                                                             pow(difference, 2);
        weightedDiffs += timePenaltyOffsetMults(dimension, 0) * abs(difference);
    }
    rootSumSquaredDifferencesTime = sqrt(rootSumSquaredDifferencesTime);

    // Make sure the normalisation penalty is not allowed to be 0.
    normPenaltyVal = rootSumSquaredDifferencesTime + weightedDiffs +
                                                               timePenaltyConst;
    if(normPenaltyVal == 0)
        normPenaltyVal = 0.0000000000001f;

    // Return the final normalisation penalty.
    return(normPenaltyVal);
}

#ifdef ENABLE_GRAPHING
template<int numDimensions>
void SituationSearchGaussianProcess<numDimensions>::drawFullValueFunction(
     std::vector<float> lockedValues, const int dimension, const int numSamples,
     const std::string title, const std::vector<float>& bestPoint,
                                              const std::vector<float>& current)
{
    // Used to plot the graph.
    Gnuplot gnuPlotter;
    Gnuplot gnuPlotter2;

    // The set of values to be graphed.
    std::vector<std::pair<float, float> > values;
    std::vector<std::pair<float, float> > values2;

    // Gather up the values.
    for(float xVal=0; xVal<=1.001; xVal += 1.0f/(numSamples-1))
    {
        lockedValues[dimension] = xVal;
        values.push_back(std::pair<float, float>(xVal,
                               getFullValue(lockedValues, bestPoint, current)));
    }

    // Gather up the derivative values.
    for(float xVal=0; xVal<=1.001; xVal += 1.0f/(numSamples-1))
    {
        lockedValues[dimension] = xVal;
        Eigen::Matrix<float, numDimensions, 1> pointEigen(lockedValues.data());
        Eigen::Matrix<float, numDimensions, 1> bestEigen(bestPoint.data());
        Eigen::Matrix<float, numDimensions, 1> currentEigen(current.data());
        Eigen::Matrix<float, numDimensions, 1> derivative;
        getFullDerivative(pointEigen, bestEigen, bestPoint, currentEigen,
                                                                    derivative);
        values2.push_back(std::pair<float, float>(xVal,
                                                     derivative(dimension, 0)));
        if(lockedValues[1] > 0.4 && lockedValues[1] < 0.40)
        {
            std::cout << "Derivative at " << xVal << ": " <<
                                          derivative(dimension, 0) << std::endl;
            getMeanDerivative(pointEigen, derivative);
            std::cout << "Mean derivative at " << xVal << ": " <<
                                          derivative(dimension, 0) << std::endl;
            getVarianceDerivative(pointEigen, derivative);
            std::cout << "Variance derivative at " << xVal << ": " <<
                                          derivative(dimension, 0) << std::endl;
        }
    }

    std::cout << "Best point: (" << bestPoint[0] << ", " << bestPoint[1] << ")"
                                                                   << std::endl;

    // Set up the graph.
    gnuPlotter << "set term wxt title '" + title + "'\n";
    gnuPlotter << "set xrange [0:1]\n";
    //gnuPlotter << "set yrange [-1.0:1.0]\n";
    gnuPlotter << "unset key\n";
    gnuPlotter << "plot '-' with lines\n";

    // Pass in the data.
    gnuPlotter.send1d(values);

    // Set up the derivative graph.
    gnuPlotter2 << "set term wxt title '" + title + " Derivative'\n";
    gnuPlotter2 << "set xrange [0:1]\n";
    //gnuPlotter2 << "set yrange [-1.0:1.0]\n";
    gnuPlotter2 << "unset key\n";
    gnuPlotter2 << "plot '-' with lines\n";
    gnuPlotter2.send1d(values2);
}
#endif // ENABLE_GRAPHING

// Undefine these so they don't mess with anything else.
#undef trainingPoints
#undef trainingWeights
#undef covariance
#undef lengthScales
#undef gpVariance
#undef trainingResults
