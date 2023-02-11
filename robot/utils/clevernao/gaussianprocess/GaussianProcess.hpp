#ifndef GAUSSIAN_PROCESS_H
#define GAUSSIAN_PROCESS_H

#include <vector>
#include <iostream>
#include <math.h>

#include "Eigen/Eigen"
#include "GPFunc/GPFunc.hpp"
#include "DynamicEigen.hpp"

#define RAND_DIVISOR RAND_MAX

// Multiplier for the derivative to determine axis movement at each hill climb
// step.
#define GAUSSIAN_PROCESS_HILL_CLIMB_STEP_SIZE 0.01f

// The maximum allowed number of hill climb steps.
#define GAUSSIAN_PROCESS_HILL_CLIMB_MAX_STEPS 100

// How "flat" the steepest axis must be for hill climb to terminate.
#define GAUSSIAN_PROCESS_MIN_CLIMB_STEEPNESS 0.01f

// The number of hyperparameters in the covariance function.
#define COVARIANCE_HYPERPARAMETERS numDimensions+2

//#define ENABLE_GRAPHING
#ifdef ENABLE_GRAPHING
#include "gnuplot-iostream/gnuplot-iostream.h"
#endif // ENABLE_GRAPHING

//#define MATRIX_DEBUG
//#define GAUSSIAN_PROCESS_HYPERPARAMETER_DERIVATIVE_DEBUG
//#define GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG_EXTRA
//#define GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
//#define CHECK_NANS

// This grabs all the c math constants for use.
#define _USE_MATH_DEFINES

/*
Creates and maintains a Gaussian Process for machine learning in an n
dimensional space.
*/
template<int numDimensions>
class GaussianProcess
{

public:

    /*
    Create a new gaussian process, primarily just recording the mean and
    variance functions and initialising as needed. Note that mean and variance
    should be persistant objects, and will be freed by GuassianProcess
    automatically when it is destroyed. gpVariance is the amount of variance
    used in the covariance function, and noise is the amount of expected noise
    in the training sample values.
    */
    GaussianProcess(GPFunc* mean, const float gpVariance,
                             const float noise, const float baseLengthScales,
                             const float confidenceIntervalStandardDeviations) :
        trainingPoints(new
        Eigen::Matrix<float, Eigen::Dynamic, numDimensions>(1, numDimensions)),
        trainingMeans(new Eigen::Matrix<float, Eigen::Dynamic, numDimensions>(1,
        numDimensions)), trainingResults(new Eigen::VectorXf(1, 1)),
        trainingPriorMeanDiffs(new Eigen::VectorXf(1, 1)),
        covarianceLDLT(new Eigen::LDLT<Eigen::MatrixXf>(
        Eigen::MatrixXf::Zero(1,1))), initialised(false), priorMean(mean),
        gpVariance(gpVariance), noise(noise),
        lengthScales(Eigen::Matrix<float, numDimensions, 1>::Constant(1) *
                     baseLengthScales),
        confidenceIntervalStandardDeviations(
                                          confidenceIntervalStandardDeviations),
                                           hyperparameterLikelihoodChanged(true)
    {
        for(int hyperparameter=0; hyperparameter<COVARIANCE_HYPERPARAMETERS;
                                                               ++hyperparameter)
        {
            hypDerivativeChanged[hyperparameter] = true;
            invCovHypDerivativeChanged[hyperparameter] = true;
            confidenceIntervalChanged[hyperparameter] = true;
        }
    }

    /*
    Destructor. Free all the memory used by GaussianProcess.
    */
    virtual ~GaussianProcess();

    /*
    Add a new variable to the training set of the gaussian process.
    */
    virtual void addTraining(const std::vector<float> &values,
                                                            const float result);

    /*
    Gets the number of training points currently in the model.
    */
    int getTrainingPointCount()
    {
        return(trainingPoints->rows());
    }

    /*
    Predict the mean value of a particular point in the gaussian process.
    */
    float predictMean(const std::vector<float> &values) const;

    /*
    Predict the variance of a particular point in the gaussian process.
    */
    float predictVariance(const std::vector<float> &values);

    /*
    Calculates the probability that point A has a higher true mean return than
    the given gaussian.
    */
    float higherMean(const std::vector<float> &values, float meanB, float
                                                                     varianceB);

    /*
    Attempt to optimise the covariance hyperparameters. Performs hill climbing,
    initialised from one random location. If the parameters found are better
    than the current set, the covaraince function will be updated. The
    difference in likelihood between the point found and the current best is
    returned.
    */
    float optimiseCovarianceHyperparameters();

    /*
    Returns how likely the current set of hyperparameters are, given the
    training data.
    */
    float getLikelihood();

#ifdef ENABLE_GRAPHING
    void drawGaussianProcess(std::vector<float> lockedValues,
                             const int dimension, const int numSamples,
                                                 const std::string title) const;
    void drawGaussianProcess(std::vector<float> lockedValues,
            const int dimension, const int numSamples, const std::string title,
                                     const std::vector<float> extraPoint) const;
#endif // ENABLE_GRAPHING
#ifndef ENABLE_GRAPHING
protected:
#endif // not ENABLE_GRAPHING

    /*
    The set of training points used by the gaussian process. Columns are points,
    rows are the values of those points.
    */
    Eigen::Matrix<float, Eigen::Dynamic, numDimensions>*  trainingPoints;

    /*
    The mean values for each training point, repeated once per dimension.
    */
    Eigen::Matrix<float, Eigen::Dynamic, numDimensions>* trainingMeans;

    /*
    The set of results corresponding to the training points used by the gaussian
    process. Composed of dynamic column vectors of floats (vector, X (dynamic),
    f (float)).
    */
    Eigen::VectorXf* trainingResults;

    /*
    The set of differences between the observed value and the prior mean for
    each training point.
    */
    Eigen::VectorXf* trainingPriorMeanDiffs;

    /*
    The set of prediction weights for the training points.
    */
    Eigen::VectorXf trainingWeights;

    /*
    The covariance of the training points.
    */
    Eigen::MatrixXf covariance;

    /*
    The LDLT decomposition of covariance.
    */
    Eigen::LDLT<Eigen::MatrixXf>* covarianceLDLT;

    /*
    A DynamicEigen instance for manipulating Eigen matricies.
    */
    DynamicEigen manip;

    /*
    Whether the first training sample has been added and the GaussianProcess is
    properly initialised.
    */
    bool initialised;

    /*
    The function used to calculate the prior mean value of a given point in the
    gaussian process.
    */
    GPFunc* priorMean;

    /*
    The unavoidable noise in the system. This would include things like unknowns
    in the world that effect the result of experiments (I think).
    */
    float gpVariance;

    /*
    The independant noise in the training data. This is essentially measurement
    error.
    */
    float noise;

    /*
    The length scales of each axis in gaussian process.
    */
    Eigen::Matrix<float, numDimensions, 1> lengthScales;

    // The number of standard deviations to include in the confidence interval.
    float confidenceIntervalStandardDeviations;

    /*
    These values are buffered as they are needed in several calculations.
    */

    /*
    The likelihood of the current set of hyperparameters.
    */
    float hyperparameterLikelihood;
    bool hyperparameterLikelihoodChanged;

    /*
    The derivative of each hyperparameter.
    */
    Eigen::MatrixXf hypDerivative[COVARIANCE_HYPERPARAMETERS];
    bool hypDerivativeChanged[COVARIANCE_HYPERPARAMETERS];
    const Eigen::MatrixXf& getHyperparameterDerivative(
                                                      const int hyperparameter);

    /*
    The inverse covariance by the derivative of each hyperparameter.
    */
    Eigen::MatrixXf invCovHypDerivative[COVARIANCE_HYPERPARAMETERS];
    bool invCovHypDerivativeChanged[COVARIANCE_HYPERPARAMETERS];
    const Eigen::MatrixXf& getInverseCovarianceByHyperparameterDerivative(
                                                      const int hyperparameter);

    /*
    The confidence interval of the requested hyperparameter.
    */
    float confidenceInterval[COVARIANCE_HYPERPARAMETERS];
    bool confidenceIntervalChanged[COVARIANCE_HYPERPARAMETERS];
    float getConfidenceInterval(const int hyperparameter);

    /*
    Recalculates the training weights. The weights are w=(sigma^-1*(f-m)), where
    sigma is the covariance of the training points, f is the results of the
    training points and m is the prior means for the training points. Actually
    calculating sigma^-1 can be avoided by solving sigma*w=(f-m) for w instead.

    As the covariance matrix is symmetric this can be done with an LDL^T
    decomposition. Details here: http://stackoverflow.com/questions/11785201/
    mahalanobis-distance-inverting-the-covariance-matrix. Basically only one
    half of the matrix need be calculated, using an LU decomposition. The Eigen
    library on the naos actually already has this for use by the Kalman Filter.
    */
    void updateTrainingWeights();

    /*
    Calculates the covariance across the training samples. Uses the equation
    Q=(1/(N-1))(M-1_nx)^T(M-1_Nx) from https://en.wikipedia.org/wiki/Sample_mean
    _and_covariance#Sample_covariance.

    Here M is trainingPoints, N is numDimensions, 1_N just indicates x should
    repeat once for each dimension (mathematically it's an NX1 matrix of ones),
    and x is a the mean values of the training samples. Together 1_Nx is
    trainingMeans.
    */
    void calculateCovariance();

    /*
    Calculates covariance between a sample and and the training samples.
    */
    Eigen::VectorXf calculateCovariance(const
                          Eigen::Matrix<float, numDimensions, 1> &sample) const;

    /*
    Calculates the culumative distribution function value corresponding to the
    given point - the probability that a N(0, 1) gaussian point will have that
    value or less.
    */
    inline float cdf(const float value) const;

    /*
    Calculates the value of the derivative of the covariance between the given
    points by the given length scale, for a single point pair.
    */
    inline float calculateCovarianceLengthscaleDerivative(const float pointA,
        const float pointB, const bool samePoint, const float cov,
                                                     const int dimension) const;

    /*
    Calculates the value of the derivative of the covariance between the given
    points by the given length scale, for the entire covariance matrix.
    */
    inline Eigen::MatrixXf calculateCovarianceLengthscaleDerivatives(
                                                     const int dimension) const;

    /*
    Calculates the value of the derivative of the covariance between the given
    points by the gp variance, for the entire covariance matrix.
    */
    inline Eigen::MatrixXf calculateCovarianceGPVarianceDerivatives() const;

    /*
    Calculates the value of the derivative of the covariance between the given
    points by the noise, for the entire covariance matrix.
    */
    inline Eigen::MatrixXf calculateCovarianceNoiseDerivatives() const;

    /*
    Calculates the derivative covariance matrix corresponding to each covariance
    hyperparameter.
    */
    inline std::vector<Eigen::MatrixXf>
                           calculateCovarianceHyperparameterDerivatives() const;

    /*
    Calculates the derivatives of the covariance function hyperparameter's
    value likelihoods. restoreCovariance should be true if the time should be
    taken to recalculate the old covariance matrix and training weights when
    done.
    */
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>
        calculateHyperparameterLikelihoodDerivatives(
        const Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point,
                                                 bool restoreCovariance = true);

    /*
    Gets a vector of the current hyperparameters.
    */
    void getHyperparameters(Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>&
                                                                    vals) const;

    /*
    Sets the hyperparameters to the given values.
    */
    void setHyperparameters(const
                    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point);

    /*
    When the gaussian process changes this is called to mark buffered solutions
    for recalculation.
    */
    void setBuffersChanged();

    /*
    Calculates the second derivative of the likelihood function by the given
    covariance hyperparameter.
    */
    float secondDerivativeLikelihood(const int hyperparameter);

    /*
    Calculates the second derivative of the covariance matrix by the given
    hyperparameter.
    */
    inline Eigen::MatrixXf secondCovarianceDerivativeHyperparameter(
                                                const int hyperparameter) const;

    /*
    Calculates the second derivative of the covariance function by the given
    lengthscale.
    */
    inline Eigen::MatrixXf secondCovarianceDerivativeLengthscale(
                                                     const int dimension) const;

    /*
    Calculates the second derivative of the covariance function by the
    gpVariance.
    */
    inline Eigen::MatrixXf secondCovarianceDerivativeGPVariance() const;

    /*
    Calculates the second derivative of the covariance function by the noise.
    */
    inline Eigen::MatrixXf secondCovarianceDerivativeNoise() const;

    /*
    Performs a covariance hyperparameter hill climb from the given point.
    */
    void covarianceHyperparameterHillClimb(
                    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point);
};

/*
Destructor. Free all the memory used by GaussianProcess.
*/
template<int numDimensions>
GaussianProcess<numDimensions>::~GaussianProcess()
{
    delete trainingPoints;
    delete trainingMeans;
    delete trainingResults;
    delete trainingPriorMeanDiffs;
    delete priorMean;
}

/*
Attempt to optimise the covariance hyperparameters. Performs hill climbing,
initialised from one random location. If the parameters found are better than
the current set, the covaraince function will be updated. The difference in
likelihood between the point found and the current best is returned.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::optimiseCovarianceHyperparameters()
{


    // The current point in the search.
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> point;

    // The likelihood of the old parameters.
    float oldLikelihood = getLikelihood();

    // The likelihood of the new parameters.
    float newLikelihood;

    // The old hyperparameters.
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> oldParams;

    // Get the existing hyperparameters.
    getHyperparameters(oldParams);

    // Populate a point for the search.
    for(unsigned int hyperparameter=0;
                    hyperparameter<COVARIANCE_HYPERPARAMETERS; ++hyperparameter)
        point(hyperparameter, 0) = (float)rand()/(float)RAND_DIVISOR;

    // Now hill climb that point. Implementation is vulnerable to flat non
    // maxima areas.
    covarianceHyperparameterHillClimb(point);

    // Calculate the actual likelihood of the best point found.
    newLikelihood = getLikelihood();

#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
    std::cout << "Old likelihood: " << oldLikelihood << std::endl;
    std::cout << "New likelihood: " << newLikelihood << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG

    // If the old points were better, restore the old covariance and training
    // weights.
    if(oldLikelihood > newLikelihood)
    {
        // Move our best point closer to the maximum.
        covarianceHyperparameterHillClimb(oldParams);

        // Double check that these parameters are an improvement.
        newLikelihood = getLikelihood();
        if(oldLikelihood < newLikelihood)
        {
#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
            std::cout << "Old hyperparameters climbed to: " << oldParams <<
                                                         std::endl << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
            setHyperparameters(oldParams);
        }
        else
        {
#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
            std::cout << "No new hyperparameters" << std::endl << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG

            // Bring the training weights back in line with the old parameters.
            updateTrainingWeights();
        }
    }

    // Otherwise the current weights should still be correct, so just set the
    // new parameters.
    else
    {
#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
        std::cout << "New hyperparameters:" << std::endl << point << std::endl
                                                                   << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG
        setHyperparameters(point);
    }

    // Return the amount of improvement.
    return(newLikelihood - oldLikelihood);
}

/*
Performs a covariance hyperparameter hill climb from the given point.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::covarianceHyperparameterHillClimb(
                     Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point)
{
    // Stores the derivative of the point currently being examined.
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> derivative;

    // The number of hill climb steps that have been run.
    int steps = 0;

    do
    {
        // Calculate the derivative of this point.
        derivative = calculateHyperparameterLikelihoodDerivatives(point, false);

        // Make the step.
        //point += GAUSSIAN_PROCESS_HILL_CLIMB_STEP_SIZE*derivative;
        for(unsigned int hyperparameter=0;
                    hyperparameter<COVARIANCE_HYPERPARAMETERS; ++hyperparameter)
        {
            if(fabs(derivative[hyperparameter]) >
                                          GAUSSIAN_PROCESS_HILL_CLIMB_STEP_SIZE)
            {
                point(hyperparameter, 0) += copysign(
                                        GAUSSIAN_PROCESS_HILL_CLIMB_STEP_SIZE,
                                                    derivative[hyperparameter]);
            }
        }

        // Step complete.
        ++steps;

    #ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG_EXTRA
        std::cout << "Point at step " << steps << ": " << std::endl << point
                                                                   << std::endl;
        std::cout << "Derivatives at step " << steps << ": " << std::endl <<
                                                        derivative << std::endl;
        std::cout << "Likelihood after step " << steps << ": " <<
                                                   getLikelihood() << std::endl;
    #endif // GAUSSIAN_PROCESS_HYPERPARAMETER_HILL_CLIMB_DEBUG_EXTRA
    }
    while((*std::max_element(derivative.data(), derivative.data() +
                   derivative.size()) > GAUSSIAN_PROCESS_MIN_CLIMB_STEEPNESS ||
                   fabs(*std::min_element(derivative.data(), derivative.data() +
                   derivative.size())) > GAUSSIAN_PROCESS_MIN_CLIMB_STEEPNESS)
                              && steps < GAUSSIAN_PROCESS_HILL_CLIMB_MAX_STEPS);
}

/*
Add a new variable to the training set of the gaussian process. Note that axis
normalisation is not currently considered.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::addTraining(const std::vector<float>
                                                    &values, const float result)
{
    // The index of the new entry.
    int newEntry = trainingPoints->rows();

    // The mean of the new entry.
    float mean = 0;

    // Buffered variables will need to be updated.
    setBuffersChanged();

    // Make room for the new data.

    // "Special case" for the first entry - needed as Eigen does not allow
    // creation of 0 row matricies with new.
    if(!initialised)
    {
        // We're initialised now.
        initialised = true;

        // This is actually the first entry.
        newEntry = 0;

        // Save the values and calculate the mean.
        for(int value=0; value<numDimensions; ++value)
        {
            mean += values[value];
            (*trainingPoints)(newEntry, value) = values[value];
        }
        mean /= numDimensions;

        // Add the mean value to the mean array.
        for(int value=0; value<numDimensions; ++value)
            (*trainingMeans)(newEntry, value) = mean;

        // Set the training results array.
        (*trainingResults)(newEntry, 0) = result;
        (*trainingPriorMeanDiffs)(newEntry, 0) =
                                             result-priorMean->getValue(values);
    }

    // This isn't the first entry, so use DynamicEigen's resize system.
    else
    {
        // Calculate the mean value.
        for(int value=0; value<numDimensions; ++value)
            mean += values[value];
        mean /= numDimensions;

        // Expand the matricies.
        manip.addRow<float, numDimensions>(trainingPoints, values);
        manip.addRow<float, numDimensions>(trainingMeans, mean);
        manip.addRow<float, 1>(trainingResults, result);
        manip.addRow<float, 1>(trainingPriorMeanDiffs,
                                            result-priorMean->getValue(values));
    }

    // Update the weight values.
    updateTrainingWeights();

#ifdef MATRIX_DEBUG
    std::cout << "Training Points:" << std::endl;
    std::cout << (*trainingPoints) << std::endl << std::endl;

    std::cout << "Training Means:" << std::endl;
    std::cout << (*trainingMeans) << std::endl << std::endl;

    std::cout << "Training Results:" << std::endl;
    std::cout << (*trainingResults) << std::endl << std::endl;

    std::cout << "Training Weights:" << std::endl;
    std::cout << trainingWeights << std::endl << std::endl;
#endif // MATRIX_DEBUG
}

/*
Predict the mean value of a particular point in the gaussian process.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::predictMean(const std::vector<float>
                                                                  &values) const
{
    // The predicted mean.
    float mean;

    // Convert the vector into a matrix.
    const Eigen::Matrix<float, numDimensions, 1> sample(&values.front());

    // Grab the covariances between the sample and the training points.
    Eigen::Matrix<float, Eigen::Dynamic, 1> covariances = calculateCovariance(
                                                                        sample);

    // Calculate the mean adjustment and add it to the prior.
    mean = priorMean->getValue(values) +
                                 (covariances.transpose()*trainingWeights)(0,0);

#ifdef MATRIX_DEBUG
    std::cout << "Predict Mean Result: " << mean << std::endl << std::endl;
#endif // MATRIX_DEBUG

    return(mean);
}

/*
Predict the variance of a particular point in the gaussian process.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::predictVariance(const std::vector<float>
                                                                        &values)
{
    // The predicted variance.
    float variance;

    // Convert the vector into a matrix.
    const Eigen::Matrix<float, numDimensions, 1> sample(&values.front());

    // Grab the covariances between the sample and the training points.
    Eigen::Matrix<float, Eigen::Dynamic, 1> covariances = calculateCovariance(
                                                                        sample);

    // The inverse covariance matrix times the train to sample covariances.
    Eigen::Matrix<float, Eigen::Dynamic, 1> intermediateCov;
    intermediateCov = covarianceLDLT->solve(covariances);

#ifdef MATRIX_DEBUG
    std::cout << "Train to Sample Covariances:" << std::endl;
    std::cout << covariances << std::endl << std::endl;
#endif // MATRIX_DEBUG

    // Calculate the variance adjustment and add it to the prior, making
    // pessimistic assumptions about the noise and variance.
    /*
    variance = pow(gpVariance + getConfidenceInterval(numDimensions), 2) +
               pow(noise + getConfidenceInterval(numDimensions+1), 2) -
                                 (covariances.transpose()*intermediateCov)(0,0);
    */
    variance = pow(gpVariance, 2) + pow(noise, 2) - (covariances.transpose()*intermediateCov)(0,0);

#ifdef MATRIX_DEBUG
    std::cout << "Predict Variance Result: " << variance << std::endl <<
                                                                      std::endl;
#endif // MATRIX_DEBUG

    // Return the variance.
    return(variance);
}

/*
Recalculates the training weights. The weights are w=(sigma^-1*(f-m)), where
sigma is the covariance of the training points, f is the results of the
training points and m is the prior means for the training points. Actually
calculating sigma^-1 can be avoided by solving sigma*w=(f-m) for w instead.

As the covariance matrix is symmetric this can be done with an LDL^T
decomposition. Details here: http://stackoverflow.com/questions/11785201/
mahalanobis-distance-inverting-the-covariance-matrix. Basically only one
half of the matrix need be calculated, using an LU decomposition. The Eigen
library on the naos actually already has this for use by the Kalman Filter.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::updateTrainingWeights()
{
#ifdef MATRIX_DEBUG
    Eigen::MatrixXf invCovariance;
#endif // MATRIX_DEBUG

    // Calculate the covariance.
    calculateCovariance();

    // Create the ldlt decomposition of the covariance matrix.
    delete covarianceLDLT;
    covarianceLDLT = new Eigen::LDLT<Eigen::MatrixXf>(covariance);

    // Add a small value to the diagonal to make the matrix positive definite.
    // SHOULDN'T BE NEEDED WITH NEW COVARIANCE FUNCTION.
    //identity = Eigen::MatrixXf::Identity(covariance.rows(), covariance.cols());
    //covariance += identity * 0.00001;

#ifdef MATRIX_DEBUG
    std::cout << "Covariance matrix:" << std::endl;
    std::cout << covariance << std::endl << std::endl;
#endif // MATRIX_DEBUG

#ifdef MATRIX_DEBUG
    // Now use the LDL decomposition to calculate inverse.
    Eigen::MatrixXf identity = Eigen::MatrixXf::Identity(covariance.rows(),
                                                             covariance.cols());
    invCovariance = covarianceLDLT->solve(identity);

    std::cout << "Inverse covariance matrix:" << std::endl;
    std::cout << invCovariance << std::endl << std::endl;

    std::cout << "Training prior vs mean differences:" << std::endl;
    std::cout << (*trainingPriorMeanDiffs) << std::endl << std::endl;
#endif // MATRIX_DEBUG

    // Do an LDL decomposition to calculate training weights.
    trainingWeights = covarianceLDLT->solve(*trainingPriorMeanDiffs);

#ifdef CHECK_NANS
    // Check the training weights.
    bool nans = false;
    for(int i=0; i<trainingWeights.rows(); ++i)
    {
        if(trainingWeights(i, 0) != trainingWeights(i, 0))
            nans = true;
    }
    if(nans)
        std::cout << "NANS!" << std::endl;
#endif // CHECK_NANS
}

/*
Calculates the covariance across the training samples.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::calculateCovariance()
{
    // Buffer this to neaten code.
    const int numPoints = trainingPoints->rows();

    // Unfortunately this is going to need to be a elementwise looping.
    Eigen::MatrixXf inCalcCovariances(numPoints, numPoints);

    // For every point consider every other point...
    for(int point=0; point<numPoints; ++point)
    {
        inCalcCovariances.row(point) =
                                calculateCovariance(trainingPoints->row(point));
    }

    // Add the expected independant noise.
    for(int point=0; point<numPoints; ++point)
    {
        inCalcCovariances(point, point) += pow(noise, 2);
    }

    // Share the new covariances.
    covariance = inCalcCovariances;
}

/*
Calculates covariance between a sample and and the training samples, using the
equation Covariance(T, V) = (o^2)exp(-sum_{j=1}^v(((T_j-V_j)^2)/(2l_j))). T is
a training sample, v is the number of dimensions, V is the test sample and l is
the vector of dimension length scales.
*/
template<int numDimensions>
Eigen::VectorXf GaussianProcess<numDimensions>::calculateCovariance(const
                           Eigen::Matrix<float, numDimensions, 1> &sample) const
{
    // A vector of all the covariances.
    Eigen::VectorXf pairCovariances(trainingPoints->rows());

    // Subtract the sample from every training instance.
    for(int row=0; row<trainingPoints->rows(); ++row)
    {
        // The power value for this training point.
        float powerVal = 0;

        // Array functions could do this all in one nice line if this were a
        // modern version of eigen.
        for(int col=0; col<trainingPoints->cols(); ++col)
        {
            // Sample's rows map to trainingPoints cols.
            powerVal += (pow((*trainingPoints)(row, col) - sample(col, 0), 2) /
                                              (2*pow(lengthScales(col, 0), 2)));
        }

        // Now calculate the covariance value.
        pairCovariances(row, 0) = pow(gpVariance, 2) * exp(powerVal*-1);
    }

    // Return the calculated vector of covariances.
    return(pairCovariances);
}

/*
Calculates the probability that point A has a higher true mean return than the
given gaussian.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::higherMean(const std::vector<float>
                                          &values, float meanB, float varianceB)
{
    // The mean and variance of the sample point.
    float meanA, varianceA;

    // The probability A's true mean is greater than B's true mean.
    float probA;

    // Get the mean and variance of the sample point.
    meanA = predictMean(values);
    varianceA = predictVariance(values);

    // Calculate the probability A's true mean is greater than B's true mean.
    probA = 1.0f-cdf((-(meanA-meanB))/sqrt(pow(varianceA, 2) +
                                                            pow(varianceB, 2)));

    return(probA);
}

/*
Calculates the culumative distribution function value corresponding to the given
point - the probability that a N(0, 1) gaussian point will have that value or
less.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::cdf(const float value) const
{
   return(0.5f + 0.5f * erf(value * M_SQRT1_2));
}

#ifdef ENABLE_GRAPHING
template<int numDimensions>
void GaussianProcess<numDimensions>::drawGaussianProcess(std::vector<float>
                    lockedValues, const int dimension, const int numSamples,
                                                  const std::string title) const
{
    // Used to plot the graph.
    Gnuplot gnuPlotter;

    // The set of values to be graphed.
    std::vector<std::pair<float, float> > values;

    // The training points, which will be added to the graph.
    std::vector<std::pair<float, float> > tPointResults;

    // Gather up the values.
    for(float xVal=0; xVal<=1.001; xVal += 1.0f/(numSamples-1))
    {
        lockedValues[dimension] = xVal;
        values.push_back(std::pair<float, float>(xVal,
                                                    predictMean(lockedValues)));
    }

    // Gather the training points.
    for(int point=0; point<trainingResults->size(); ++point)
    {
        tPointResults.push_back(std::pair<float, float>(
            (*trainingPoints)(point, dimension), (*trainingResults)(point, 0)));
    }

    // Set up the graph.
    gnuPlotter << "set term wxt title '" + title + "'\n";
    gnuPlotter << "set xrange [0:1]\n";
    //gnuPlotter << "set yrange [-1.0:2.0]\n";
    gnuPlotter << "unset key\n";
    gnuPlotter << "plot '-' with lines, '-' with points\n";

    // Pass in the data.
    gnuPlotter.send1d(values);
    gnuPlotter.send1d(tPointResults);
}

template<int numDimensions>
void GaussianProcess<numDimensions>::drawGaussianProcess(std::vector<float>
             lockedValues, const int dimension, const int numSamples,
             const std::string title, const std::vector<float> extraPoint) const
{
    // Used to plot the graph.
    Gnuplot gnuPlotter;

    // The set of values to be graphed.
    std::vector<std::pair<float, float> > values;

    // The training points, which will be added to the graph.
    std::vector<std::pair<float, float> > tPointResults;

    // The extra point.
    std::pair<float, float> extraPointVal(extraPoint[dimension],
                                                       predictMean(extraPoint));

    // Gather up the values.
    for(float xVal=0; xVal<=1.001; xVal += 1.0f/(numSamples-1))
    {
        lockedValues[dimension] = xVal;
        values.push_back(std::pair<float, float>(xVal,
                                                    predictMean(lockedValues)));
    }

    // Gather the training points.
    for(int point=0; point<trainingResults->size(); ++point)
    {
        tPointResults.push_back(std::pair<float, float>(
            (*trainingPoints)(point, dimension), (*trainingResults)(point, 0)));
    }

    // Set up the graph.
    gnuPlotter << "set term wxt title '" + title + "'\n";
    gnuPlotter << "set xrange [0:1]\n";
    //gnuPlotter << "set yrange [-1.0:2.0]\n";
    gnuPlotter << "unset key\n";
    gnuPlotter << "plot '-' with lines, '-' with points, '-' with points\n";

    // Pass in the data.
    gnuPlotter.send1d(values);
    gnuPlotter.send1d(tPointResults);
    gnuPlotter.send1d(std::vector<std::pair<float, float> >(1, extraPointVal));
}
#endif // ENABLE_GRAPHING

/*
Calculates the value of the derivative of the covariance between the given
points by the given length scale, for a single point pair.
*/
template<int numDimensions>
inline float
    GaussianProcess<numDimensions>::calculateCovarianceLengthscaleDerivative(
    const float pointA, const float pointB, const bool samePoint,
                                     const float cov, const int dimension) const
{
    // The derivative value.
    float derivative;

    // The difference is 0, so the derivative is 0. This is just a shortcut.
    if(samePoint)
        derivative = 0;

    // Otherwise use the full derivative calculation.
    else
    {
        derivative = (pow(pointA-pointB, 2) * cov) /
                                             pow(lengthScales(dimension, 0), 3);
    }

    // Return the calculated derivative.
    return(derivative);
}

/*
Calculates the value of the derivative of the covariance between the given
points by the given length scale, for the entire covariance matrix.
*/
template<int numDimensions>
inline Eigen::MatrixXf
    GaussianProcess<numDimensions>::calculateCovarianceLengthscaleDerivatives(
                                                      const int dimension) const
{
    // Create the new derivative matrix.
    Eigen::MatrixXf derivative(covariance.rows(), covariance.cols());

    // Get the number of training points.
    const int trainingPointCount = trainingPoints->rows();

    // Calculate the derivative values.
    for(int pointA=0; pointA<trainingPointCount; ++pointA)
    {
        for(int pointB=0; pointB<trainingPointCount; ++pointB)
        {
            derivative(pointA, pointB) =
                calculateCovarianceLengthscaleDerivative(
                (*trainingPoints)(pointA, dimension),
                (*trainingPoints)(pointB, dimension), pointA == pointB,
                                         covariance(pointA, pointB), dimension);
        }
    }

    // Return the result.
    return(derivative);
}

/*
Calculates the value of the derivative of the covariance between the given
points by the gp variance, for the entire covariance matrix.
*/
template<int numDimensions>
inline Eigen::MatrixXf
    GaussianProcess<numDimensions>::calculateCovarianceGPVarianceDerivatives()
                                                                           const
{
    // Return the calculated derivative.
    return((2*(covariance-(Eigen::MatrixXf::Identity(covariance.rows(),
                                        covariance.cols())*noise)))/gpVariance);
}

/*
Calculates the value of the derivative of the covariance between the given
points by the noise, for the entire covariance matrix.
*/
template<int numDimensions>
inline Eigen::MatrixXf
     GaussianProcess<numDimensions>::calculateCovarianceNoiseDerivatives() const
{
    // Return the calculated derivative.
    return(Eigen::MatrixXf::Identity(covariance.rows(), covariance.cols()) *
                                                                     (2*noise));
}

/*
Calculates the derivatives of the covariance function hyperparameter's
value likelihoods. restoreCovariance should be true if the time should be taken
to recalculate the old covariance matrix and training weights when done.
*/
template<int numDimensions>
Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> GaussianProcess<
    numDimensions>::calculateHyperparameterLikelihoodDerivatives(
    const Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point,
                                                         bool restoreCovariance)
{
    // The old hyperparameters.
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> oldParams;

    // Get the number of training points.
    const int trainingPointCount = trainingPoints->rows();

    // The derivative of each hyperparameter's likelihood.
    Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1> derivatives;

    // Storage for partial solutions.
    Eigen::VectorXf tempVec(trainingPointCount);
    Eigen::MatrixXf tempMatrix;
    float tempFloat;

    // Replace the hyperparameters.
    getHyperparameters(oldParams);
    setHyperparameters(point);

    // Update the covariance matrix to use the new hyperparameters.
    updateTrainingWeights();

#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_DERIVATIVE_DEBUG_EXTRA
    std::cout << "Covariance:" << std::endl << covariance << std::endl;
    std::cout << "Covaraince determinant: " << covariance.determinant()
                                                                   << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_DERIVATIVE_DEBUG_EXTRA

    // Calculate the derivative of each hyperparameter's likelihood.
    for(int hyperparameter=0; hyperparameter<COVARIANCE_HYPERPARAMETERS;
                                                               ++hyperparameter)
    {
        // Get the inverse covariance by the hyperparameter derivative.
        tempMatrix = getInverseCovarianceByHyperparameterDerivative(
                                                                hyperparameter);

        // Deal with the inverse covariance matrix calculation.
        tempVec = covarianceLDLT->solve(*trainingPriorMeanDiffs);

        // Complete solving the right portion of the equation.
        tempVec = tempMatrix * tempVec;

        // Finish the equation.
        tempFloat = ((*trainingPriorMeanDiffs).transpose()*tempVec)(0,0);

#ifdef GAUSSIAN_PROCESS_HYPERPARAMETER_DERIVATIVE_DEBUG_EXTRA
        std::cout << "Positive half: " << 0.5f*tempFloat << std::endl;
        std::cout << "Negative half: " << -0.5*tempMatrix.trace() << std::endl;
#endif // GAUSSIAN_PROCESS_HYPERPARAMETER_DERIVATIVE_DEBUG_EXTRA

        // Calculate the final answer.
        derivatives(hyperparameter, 0) = 0.5f*(tempFloat-tempMatrix.trace());
    }

    // Restore the old hyperparameters.
    setHyperparameters(oldParams);
    if(restoreCovariance)
        updateTrainingWeights();

    // Return the derivatives found.
    return(derivatives);
}

/*
Sets the hyperparameters to the given values.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::setHyperparameters(
               const Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& point)
{
    // There a new hyperparameters so buffered variables must be recalculated.
    setBuffersChanged();

    // Change the hyperparameters.
    for(int dim=0; dim<numDimensions; ++dim)
        lengthScales(dim, 0) = point(dim, 0);
    gpVariance = point(numDimensions, 0);
    noise = point(numDimensions+1, 0);
}

/*
Gets a vector of the current hyperparameters.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::getHyperparameters(
                Eigen::Matrix<float, COVARIANCE_HYPERPARAMETERS, 1>& vals) const
{
    for(int dim=0; dim<numDimensions; ++dim)
        vals(dim, 0) = lengthScales(dim, 0);
    vals(numDimensions, 0) = gpVariance;
    vals(numDimensions+1, 0) = noise;
}

/*
Calculates how likely the current set of hyperparameters are, given the data.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::getLikelihood()
{
    // Partial solution storage.
    Eigen::VectorXf tempVec;

    // The "D" in the LDLT decomposition.
    Eigen::VectorXf covarianceD;

    // The log determinant of the covariance matrix.
    float covarianceLogDeterminant = 0;

    // The number of training points.
    const int trainingPointCount = trainingPoints->rows();

    // Check if the current likelihood is known.
    if(!hyperparameterLikelihoodChanged)
        return(hyperparameterLikelihood);

    // First calculate the inverse covariance portion's solution.
    tempVec = covarianceLDLT->solve(*trainingPriorMeanDiffs);

    // Calculating the determinant is numerically unstable, so it is neccesary
    // to calculate the log determinant.
    covarianceD = covarianceLDLT->vectorD();
    for(int val=0; val<trainingPointCount; ++val)
        covarianceLogDeterminant += log(covarianceD[val]);

    // Calculate the likelihood of the current hyperparameters.
    hyperparameterLikelihood = -0.5*(covarianceLogDeterminant +
        (trainingPriorMeanDiffs->transpose()*tempVec)(0,0) +
                                                trainingPointCount*log(2*M_PI));

    // The hyperparameter likelihood is now up to date.
    hyperparameterLikelihoodChanged = false;

    // Return the hyperparameter likelihood.
    return(hyperparameterLikelihood);
}

/*
Gets the derivative of each hyperparameter. The answer is buffered in case it is
needed again before the gaussian process changes.
*/
template<int numDimensions>
const Eigen::MatrixXf&
    GaussianProcess<numDimensions>::getHyperparameterDerivative(
                                                       const int hyperparameter)
{
    // Return the current hyperparameter derivative if it is up to date.
    if(!hypDerivativeChanged[hyperparameter])
        return(hypDerivative[hyperparameter]);

    // Calculate the hyperparameter derivative.

    // Lengthscale.
    if(hyperparameter < numDimensions)
    {
        hypDerivative[hyperparameter] =
                      calculateCovarianceLengthscaleDerivatives(hyperparameter);
    }

    // GP variance.
    else if(hyperparameter == numDimensions)
    {
        hypDerivative[hyperparameter] =
                                     calculateCovarianceGPVarianceDerivatives();
    }

    // Noise.
    else
        hypDerivative[hyperparameter] = calculateCovarianceNoiseDerivatives();

    // This hyperparameter derivative is now current.
    hypDerivativeChanged[hyperparameter] = false;

    // Return the updated hyperparameter derivative.
    return(hypDerivative[hyperparameter]);
}

/*
Gets the inverse covariance by the derivative of each hyperparameter. The answer
is buffered in case it is needed again before the gaussian process changes.
*/
template<int numDimensions>
const Eigen::MatrixXf& GaussianProcess<
    numDimensions>::getInverseCovarianceByHyperparameterDerivative(
                                                       const int hyperparameter)
{
    // The hyperparameter derivative.
    Eigen::MatrixXf hyperparameterDerivative;

    // If the buffered value is valid, return it.
    if(!invCovHypDerivativeChanged[hyperparameter])
        return(invCovHypDerivative[hyperparameter]);

    // Get the derivative of this hyperparameter.
    hyperparameterDerivative = getHyperparameterDerivative(hyperparameter);

    // Solve for the hyperparameter derivative.
    invCovHypDerivative[hyperparameter] =
                                covarianceLDLT->solve(hyperparameterDerivative);

    // This calculation is now up to date.
    invCovHypDerivativeChanged[hyperparameter] = false;

    // Return the recalculated hyperparameter derivative.
    return(invCovHypDerivative[hyperparameter]);
}

/*
Gets the confidence interval of the specified hyperparameter. The answer is
buffered in case it is needed again before the gaussian process changes.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::getConfidenceInterval(
                                                       const int hyperparameter)
{
    // The second derivative of the likelihood by this hyperparameter.
    float secondDerLikelihood;

    // Return the buffered value if it is current.
    if(!confidenceIntervalChanged[hyperparameter])
        return(confidenceInterval[hyperparameter]);

    // Calculate the second derivative of the likelihood function by this
    // hyperparameter.
    secondDerLikelihood = secondDerivativeLikelihood(hyperparameter);

    // The relevant equation only really applies when we're at a local maxima.
    // If the equation is curving the other way, there isn't even a solution!
    if(secondDerLikelihood < 0)
    {
        // Calculate the confidence interval.
        confidenceInterval[hyperparameter] =
            confidenceIntervalStandardDeviations / sqrt(-1*secondDerLikelihood);
    }
    // How can a confidence interval be generated when the above doesn't apply?
    // In reality we should generally be roughly at a maxima, except when using
    // the initial user defined set of hyperparameters. Logically if
    // secondDerivativeLikelihood >0 we are VERY uncertain, so just returning a
    // high value should be sufficient.
    else
        confidenceInterval[hyperparameter] = 1000000000;

    // This confidence interval is now up to date.
    confidenceIntervalChanged[hyperparameter] = false;

    // Return the updated confidence interval.
    return(confidenceInterval[hyperparameter]);
}

/*
When the gaussian process changes this is called to mark buffered solutions
for recalculation.
*/
template<int numDimensions>
void GaussianProcess<numDimensions>::setBuffersChanged()
{
    hyperparameterLikelihoodChanged = true;
    for(int hyperparameter=0; hyperparameter<COVARIANCE_HYPERPARAMETERS;
                                                               ++hyperparameter)
    {
        hypDerivativeChanged[hyperparameter] = true;
        invCovHypDerivativeChanged[hyperparameter] = true;
        confidenceIntervalChanged[hyperparameter] = true;
    }
}

/*
Calculates the second derivative of the likelihood function by the given
covariance hyperparameter.
*/
template<int numDimensions>
float GaussianProcess<numDimensions>::secondDerivativeLikelihood(
                                                       const int hyperparameter)
{
    // The number of training samples.
    const int trainingPointCount = trainingPoints->rows();

    // The square of the inverse covariance by the hyperparameter derivative.
    Eigen::MatrixXf invCovHypDer;

    // The second derivative of the covariance by the relevant hyperparameter.
    Eigen::MatrixXf secondDerivative;

    // The inner matrix of the negative trace term.
    Eigen::MatrixXf negativeTraceTermInnerMatrix;

    // Storage for a partial solution.
    Eigen::VectorXf tempVec(trainingPointCount);
    Eigen::VectorXf tempVec2(trainingPointCount);

    // The part of the derivative sensitive to the test values.
    float testSensitiveTerm;

    // The positive trace term.
    float positiveTraceTerm;

    // The negative trace term.
    float negativeTraceTerm;

    // Get the inverse covariance by the hyperparameter derivative.
    invCovHypDer = getInverseCovarianceByHyperparameterDerivative(
                                                                hyperparameter);

    // Calculate the second derivative of the covariance by the hyperparameter.
    secondDerivative = secondCovarianceDerivativeHyperparameter(hyperparameter);

    // Calculate the inner matrix of the negative trace term.
    negativeTraceTermInnerMatrix = covarianceLDLT->solve(secondDerivative);

    // Calculate the right portion of the test sensitive term.
    tempVec = (2*getHyperparameterDerivative(hyperparameter)*invCovHypDer -
                                            secondDerivative) * trainingWeights;

    // Multiply the right portion by the inverse covariance.
    tempVec2 = covarianceLDLT->solve(tempVec);

    // Calculate the test sensitive portion of the equation.
    testSensitiveTerm = ((*trainingPriorMeanDiffs).transpose() * tempVec2)(0,0);

    // Calculate the positive trace term.
    positiveTraceTerm = (invCovHypDer*invCovHypDer).trace();

    // Calculate the negative trace term.
    negativeTraceTerm = negativeTraceTermInnerMatrix.trace();

    // Return the full second derivative.
    return(0.5*(positiveTraceTerm - negativeTraceTerm - testSensitiveTerm));
}

/*
Calculates the second derivative of the covariance matrix by the given
hyperparameter.
*/
template<int numDimensions>
inline Eigen::MatrixXf
    GaussianProcess<numDimensions>::secondCovarianceDerivativeHyperparameter(
                                                 const int hyperparameter) const
{
    // The second derivative of the covariance matrix by the given
    // hyperparameter.
    Eigen::MatrixXf derivative;

    // Calculate the second derivative for the appropriate hyperparameter.

    // Lengthscale.
    if(hyperparameter < numDimensions)
        derivative = secondCovarianceDerivativeLengthscale(hyperparameter);

    // GPVariance.
    else if(hyperparameter == numDimensions)
        derivative = secondCovarianceDerivativeGPVariance();

    // Noise.
    else
        derivative = secondCovarianceDerivativeNoise();

    // Return the relevant derivative.
    return(derivative);
}

/*
Calculates the second derivative of the covariance function by the given
lengthscale.
*/
template<int numDimensions>
inline Eigen::MatrixXf
    GaussianProcess<numDimensions>::secondCovarianceDerivativeLengthscale(
                                                      const int dimension) const
{
    // Create the new derivative matrix.
    Eigen::MatrixXf derivative(covariance.rows(), covariance.cols());

    // Get the number of training points.
    const int trainingPointCount = trainingPoints->rows();

    // Calculate the derivative values.
    for(int pointA=0; pointA<trainingPointCount; ++pointA)
    {
        for(int pointB=0; pointB<trainingPointCount; ++pointB)
        {
            if(pointA == pointB)
                derivative(pointA, pointB) = 0;
            else
            {
                // The difference between the two relevant training point
                // values, squared.
                float diffSquared = pow((*trainingPoints)(pointA, dimension) -
                                       (*trainingPoints)(pointB, dimension), 2);

                // The relevant lengthscale.
                float relLengthscale = pow(lengthScales(dimension, 0), 2);

                derivative(pointA, pointB) = (diffSquared *
                    covariance(pointA, pointB) * (diffSquared-3*relLengthscale))
                                                       / pow(relLengthscale, 3);
            }
        }
    }

    // Return the result.
    return(derivative);
}

/*
Calculates the second derivative of the covariance function by the gpVariance.
*/
template<int numDimensions>
inline Eigen::MatrixXf
    GaussianProcess<numDimensions>::secondCovarianceDerivativeGPVariance() const
{
    // Return the calculated derivative.
    return((2*(covariance-(Eigen::MatrixXf::Identity(covariance.rows(),
                                covariance.cols())*noise)))/pow(gpVariance, 2));
}

/*
Calculates the value of the derivative of the covariance between the given
points by the noise, for the entire covariance matrix.
*/
template<int numDimensions>
inline Eigen::MatrixXf
         GaussianProcess<numDimensions>::secondCovarianceDerivativeNoise() const
{
    // Return the calculated derivative.
    return(Eigen::MatrixXf::Identity(covariance.rows(), covariance.cols()) * 2);
}

/*
Solves an equation of the form trace(inv(covariance)*B), returning the trace.
*/
/*
DISABLED AS I DOUBT IT WOULD BE ANY FASTER.
template<int numDimensions>
inline float GaussianProcess<numDimensions>::solveTraceOnly(
                                           const Eigen::MatrixXf& matrixB) const
{
    // The trace of the product of the inverse covariance and matrixB.
    float trace = 0;

    // The number of training points.
    const int trainingPointCount = trainingPoints.rows();

    // The intermediate solution.
    Eigen::MatrixXf intermediate(trainingPointCount, trainingPointCount);

    // For each column.
    for(int col=0; col<trainingPointCount; ++col)
    {
        // Work down the rows.
        for(int row=0; row<trainingPointCount; ++row)
        {
            // The amount subtracted due to earlier rows.
            float sub=0;
            for(int preRow=0; preRow<row; ++preRow)
            {
                sub -= covarianceLDLT.matrixL(row, preRow) *
                                                      intermediate(preRow, col);
            }

            // Calculate the final total.
            intermediate(row, col) = (matrixB(row, col) + sub) /
                                               covarianceLDLT.matrixL(row, row);
        }
    }

    // Now the final solution can be calculated.
    for(int point=trainingPointCount-1; point!=0; --point)
    {
        trace +=
    }
}
*/

/*
Calculates the derivative covariance matrix corresponding to each covariance
hyperparameter. Removed as there doesn't seem to be a need for this function
any longer.
*//*
template<int numDimensions>
inline std::vector<Eigen::MatrixXf> GaussianProcess<numDimensions
                         >::calculateCovarianceHyperparameterDerivatives() const
{
    // The derivative covariance matricies.
    std::vector<Eigen::MatrixXf> derivativeCovMatrix;
    derivativeCovMatrix.reserve(COVARIANCE_HYPERPARAMETERS);

    // Get the length scale covariance derivative matricies.
    for(int dimension=0; dimension<numDimensions; ++dimension)
    {
        derivativeCovMatrix.push_back(
                          calculateCovarianceLengthscaleDerivatives(dimension));
    }

    // Get the GP variance derivative matrix.
    derivativeCovMatrix.push_back(calculateCovarianceGPVarianceDerivatives());

    // Get the noise variance derivative matrix.
    derivativeCovMatrix.push_back(calculateCovarianceNoiseDerivatives());

    // Return the result.
    return(derivativeCovMatrix);
}*/



#endif /* end of include guard: GAUSSIAN_PROCESS_H */
