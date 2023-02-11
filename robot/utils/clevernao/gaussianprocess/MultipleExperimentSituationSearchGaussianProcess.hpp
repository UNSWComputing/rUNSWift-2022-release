#include <float.h>

#include "utils/clevernao/gaussianprocess/SituationSearchGaussianProcess.hpp"

//#define DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES
//#define DEBUG_MULTIPLE_EXPERIMENT_EXTRA_ERROR_CHECKS

#define COVARIANCE_HYPERPARAMETERS numDimensions+2

/*
This class handles a situation in which the same set of variables must be
optimised to function well under several experimental conditions. It will choose
both which experiment type to do and which experiment within that type is best.
*/
template<int numDimensions, int numExperiments>
class MultipleExperimentSituationSearchGaussianProcess
{
public:

    /*
    Create the set of SituationSearchGaussianProcess underlying this
    MultipleExperimentSituationSearchGaussianProcess.
    */
    MultipleExperimentSituationSearchGaussianProcess(
                      std::vector<SituationSearchGaussianProcessParams>& params)
    {
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            experimentGaussianProcess[experiment] =
                new SituationSearchGaussianProcess<numDimensions>(
                                                            params[experiment]);
        }
    }

    /*
    Create the set of SituationSearchGaussianProcess underlying this
    MultipleExperimentSituationSearchGaussianProcess.
    */
    MultipleExperimentSituationSearchGaussianProcess(
                       std::vector<SituationSearchGaussianProcessParams> params)
    {
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            experimentGaussianProcess[experiment] =
                new SituationSearchGaussianProcess<numDimensions>(
                                                            params[experiment]);
        }
    }

    /*
    Clean up the SituationSearchGaussianProcess created by this
    MultipleExperimentSituationSearchGaussianProcess.
    */
    ~MultipleExperimentSituationSearchGaussianProcess()
    {
        for(int experiment=0; experiment<numExperiments; ++experiment)
            delete experimentGaussianProcess[experiment];
    }

    /*
    Gets the number of training points currently in the model.
    */
    int getTrainingPointCount()
    {
        return(experimentGaussianProcess[0]->getTrainingPointCount());
    }

    /*
    Returns the full set of hyperparameters, as an hyperparameters by
    experiments eigen matrix.
    */
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, numExperiments>
                                                            getHyperparameters()
    {
        // The full set of hyperparameters.
        Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, numExperiments>
                                                                hyperparameters;

        // A single set of hyperparamters.
        Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> temp;

        // Get the covariance hyperparameters for each experiment.
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            experimentGaussianProcess[experiment]->getHyperparameters(temp);
            hyperparameters.col(experiment) = temp;
        }

        // Return the filled matrix of hyperparameters.
        return(hyperparameters);
    }

    /*
    Returns the confidence interval of each of the experiments for the given
    covariance hyperparameter.
    */
    Eigen::Matrix<float, numExperiments, 1> getConfidenceInterval(
                                                             int hyperparameter)
    {
        // The set of confidence intervals.
        Eigen::Matrix<float, numExperiments, 1> confidenceIntervals;

        // Get the confidence interval for each experiment.
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            confidenceIntervals(experiment, 0) =
                experimentGaussianProcess[experiment]->getConfidenceInterval(
                                                                hyperparameter);
        }

        // Return the filled vector of confidence intervals.
        return(confidenceIntervals);
    }

    /*
    Add a new variable to the training set of the gaussian process.
    */
    void addTraining(const std::vector<float>& values, const float result,
                                                               int experimentID)
    {
        experimentGaussianProcess[experimentID]->addTraining(values, result);
    }

    /*
    Sets the situation to be used by the
    MultipleExperimentSituationSearchGaussianProcess.
    */
    void setSituation(const std::vector<float>& newSituation)
    {
        situation = newSituation;
        for(int experiment=0; experiment<numExperiments; ++experiment)
            experimentGaussianProcess[experiment]->setSituation(newSituation);
    }

    /*
    Finds the best settings given that the objective is to maximise the sum of
    the value over all experiment types. Approximated with random initialisation
    hill climbing.
    */
    float findBest(std::vector<float>& testBestPoint,
                                     bool initialiseFromGivenPoint=false) const;

    /*
    Finds the settings that could provide the greatest potential improvement
    over the current best known settings.
    */
    float findValuePoint(const std::vector<float>& best, float bestPointValue,
        const std::vector<float>& currentVec, std::vector<float>& testPoint,
                                                int& preferredExperiment) const;

    /*
    The expected value of the given point, as the sum of value over all
    experiment types.
    */
    float predictMean(const std::vector<float>& point) const
    {
        float mean = 0;
        for(int experiment=0; experiment<numExperiments; ++experiment)
            mean += experimentGaussianProcess[experiment]->predictMean(point);
        return(mean);
    }

    /*
    The expected variance of the given point, as the sum of value over all
    experiment types.
    */
    float predictVariance(const std::vector<float>& point) const
    {
        float variance = 0;
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            variance +=
                  experimentGaussianProcess[experiment]->predictVariance(point);
        }
        return(variance);
    }

    /*
    The full estimated value of a particular point in the current situation.
    */
    float getFullValue(const std::vector<float>& point,
        const std::vector<float>& best, const std::vector<float>& current) const
    {
        float fullValue = 0;
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            fullValue += experimentGaussianProcess[experiment]->getFullValue(
                                                          point, best, current);
        }
        return(fullValue);
    }

    /*
    Perform a single hill climb to optimise the covariance hyperparameters for a
    specific experiment.
    */
    void optimiseCovarianceHyperparameters(int experiment)
    {
        experimentGaussianProcess[
                               experiment]->optimiseCovarianceHyperparameters();
    }

    /*
    Returns the lengthscales to use when comparing situations.
    */
    Eigen::VectorXf getLengthscales() const
    {
        Eigen::VectorXf lengthscales = experimentGaussianProcess[0]->lengthScales;
        for(int experiment=1; experiment<numExperiments; ++experiment)
        {
            for(int dim=0; dim<numDimensions; ++dim)
            {
                if(lengthscales[dim] > experimentGaussianProcess[experiment]->lengthScales[dim])
                    lengthscales[dim] = experimentGaussianProcess[experiment]->lengthScales[dim];
            }
        }
        return(lengthscales);
    }

    /*
    Gets the total number of exemplars in all experiments.
    */
    inline unsigned int getExemplarCount()
    {
        int exemplarCount = 0;
        for(int experiment=0; experiment<numExperiments; ++experiment)
        {
            exemplarCount += experimentGaussianProcess[
                                                experiment]->getExemplarCount();
        }
        return(exemplarCount);
    }

    /*
    Gets the specific exemplar corresponding to the given ID.
    */
    inline const Eigen::Matrix<float, numDimensions, 1>& getExemplar(
                                                          unsigned int exemplar)
    {
        int experiment = 0;
        while(exemplar >= experimentGaussianProcess[
                                                experiment]->getExemplarCount())
        {
            exemplar -= experimentGaussianProcess[
                                                experiment]->getExemplarCount();
            ++experiment;
        }
        return(experimentGaussianProcess[experiment]->getExemplar(exemplar));
    }

private:

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
                      Eigen::Matrix<float, numDimensions, 1>& derivative) const;

    /*
    The underlying set of gaussian process that model each experimental
    condition.
    */
    SituationSearchGaussianProcess<numDimensions>*
                                      experimentGaussianProcess[numExperiments];

    /*
    The current situation.
    */
    std::vector<float> situation;
};

/*
Calculates the derivative of a particular point in the situation. point should
be the full point, including situation. derivative is a return variable.
*/
template<int numDimensions, int numExperiments>
void MultipleExperimentSituationSearchGaussianProcess<numDimensions,
     numExperiments>::getMeanDerivative(
                 const Eigen::Matrix<float, numDimensions, 1>& point,
                       Eigen::Matrix<float, numDimensions, 1>& derivative) const
{
    // Temporary storage for the derivatives until they are summed.
    Eigen::Matrix<float, numDimensions, 1> tempDerivative;

    // Initialise derivative.
    if(numExperiments > 0)
    {
        experimentGaussianProcess[0]->getMeanDerivative(point, tempDerivative);
        derivative = tempDerivative;
    }

    // Add the remaining experiments.
    for(int experiment=1; experiment<numExperiments; ++experiment)
    {
        experimentGaussianProcess[experiment]->getMeanDerivative(point,
                                                                tempDerivative);
        derivative += tempDerivative;
    }
}

/*
Calculates the derivative of a particular point in the situation. point should
be the full point, including situation. derivative is a return variable.
*/
template<int numDimensions, int numExperiments>
void MultipleExperimentSituationSearchGaussianProcess<numDimensions,
     numExperiments>::getFullDerivative(
                       const Eigen::Matrix<float, numDimensions, 1>& point,
                       const Eigen::Matrix<float, numDimensions, 1>& best,
                       const std::vector<float> bestVec,
                       const Eigen::Matrix<float, numDimensions, 1>& current,
                       Eigen::Matrix<float, numDimensions, 1>& derivative) const
{
    // Temporary storage for the derivatives until they are summed.
    Eigen::Matrix<float, numDimensions, 1> tempDerivative;

    // Initialise derivative.
    if(numExperiments > 0)
    {
        experimentGaussianProcess[0]->getFullDerivative(point, best, bestVec,
                                                       current, tempDerivative);
        derivative = tempDerivative;
    }

    // Add the remaining experiments.
    for(int experiment=1; experiment<numExperiments; ++experiment)
    {
        experimentGaussianProcess[experiment]->getFullDerivative(point, best,
                                              bestVec, current, tempDerivative);
        derivative += tempDerivative;
    }
}

/*
Finds the best settings given that the objective is to maximise the sum of
the value over all experiment types. Approximated with random initialisation
hill climbing.
*/
template<int numDimensions, int numExperiments>
float MultipleExperimentSituationSearchGaussianProcess<numDimensions,
    numExperiments>::findBest(std::vector<float>& best,
                                            bool initialiseFromGivenPoint) const
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
#ifdef DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES
        std::cout << "Derivative before (basic): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES

        // Calculate the derivative of this point.
        getMeanDerivative(point, derivative);

#ifdef DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES
        std::cout << "Derivative after (basic): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES

#ifdef DEBUG_MULTIPLE_EXPERIMENT_EXTRA_ERROR_CHECKS
        for(unsigned int dim=0; dim<situation.size(); ++dim)
        {
            if(derivative[dim] != 0)
            {
                std::cout << "MOVEMENT ON LOCKED AXIS (BASIC): " <<
                                                   derivative[dim] << std::endl;
            }
        }
#endif // DEBUG_MULTIPLE_EXPERIMENT_EXTRA_ERROR_CHECKS

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
    return(predictMean(best));
}

/*
Finds the settings that could provide the greatest potential improvement
over the current best known settings.
*/
template<int numDimensions, int numExperiments>
float MultipleExperimentSituationSearchGaussianProcess<numDimensions,
      numExperiments>::findValuePoint(const std::vector<float>& bestVec,
      float bestPointValue, const std::vector<float>& currentVec,
                      std::vector<float>& value, int& preferredExperiment) const
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

    // The highest value of a single experiment at the point found.
    float preferredExperimentValue = -1.0f*FLT_MAX;

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

//std::cout << "Searching From: " << point.transpose() << std::endl;
    // Now hill climb that point. Implementation is vulnerable to flat non
    // maxima areas.
    do
    {
#ifdef DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES
        std::cout << "Derivative before (full): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES

        // Calculate the derivative of this point.
        getFullDerivative(point, best, bestVec, current, derivative);

#ifdef DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES
        std::cout << "Derivative after (full): " << derivative.transpose() <<
                                                                      std::endl;
#endif // DEBUG_MULTIPLE_EXPERIMENT_SHOW_DERIVATIVES

#ifdef DEBUG_MULTIPLE_EXPERIMENT_EXTRA_ERROR_CHECKS
        for(unsigned int dim=0; dim<situation.size(); ++dim)
        {
            if(derivative[dim] != 0)
            {
                std::cout << "MOVEMENT ON LOCKED AXIS (full): " <<
                                                   derivative[dim] << std::endl;
            }
        }
#endif // DEBUG_MULTIPLE_EXPERIMENT_EXTRA_ERROR_CHECKS

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

    // Return the point found.
    value.clear();
    for(int dim=0; dim<numDimensions; ++dim)
        value.push_back(point(dim, 0));

    // Return the preferred experiment at the point found.
    for(int experiment=0; experiment<numExperiments; ++experiment)
    {
        // Calculate the value of this experiment.
        float experimentValue =
            experimentGaussianProcess[experiment]->getFullValue(value, bestVec,
                                                                    currentVec);

        // If this experiment is the best found, mark it for use.
        if(experimentValue > preferredExperimentValue)
        {
            preferredExperimentValue = experimentValue;
            preferredExperiment = experiment;
        }
    }
//std::cout << "Ended at: " << point.transpose() << " with value " << getFullValue(value, bestVec, currentVec) << std::endl;
    // Return the value of the point found.
    return(getFullValue(value, bestVec, currentVec));
}
