#ifndef STATE_ESTIMATION_ADAPTER_HPP
#define STATE_ESTIMATION_ADAPTER_HPP

#include "blackboard/Adapter.hpp"
#include <fstream>

enum EstimatorsEnum
{
    LOCALISER = 0,
    EGOBALLTRACKER,
    MULTIBALLTRACKER,
    TEAMBALLTRACKER,
    ROBOTFILTER,
    ESTIMATOR_TOTAL
};

class Estimator;
class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
struct Odometry;

class StateEstimationAdapter : Adapter
{
  public:
    explicit StateEstimationAdapter(Blackboard *bb);
    ~StateEstimationAdapter();

    void tick();

    /* Construct and Function, used only for state-estimation-simulation) */
    explicit StateEstimationAdapter(EstimatorInfoInit *estimatorInfoInit);
    void tick(
        EstimatorInfoIn *estimatorInfoIn,
        EstimatorInfoMiddle *estimatorInfoMiddle,
        EstimatorInfoOut *estimatorInfoOut);

  private:

    void initEstimators();
    void addEstimator(unsigned index, Estimator *estimator);
    void runEstimators();
    void runEstimator(unsigned index);
    Estimator *getEstimator(unsigned index);

    void createEstimatorInfoIn();
    void writeToBlackboard();

    Estimator *estimators[ESTIMATOR_TOTAL];

    EstimatorInfoInit *estimatorInfoInit;
    EstimatorInfoIn *estimatorInfoIn;
    EstimatorInfoMiddle *estimatorInfoMiddle;
    EstimatorInfoOut *estimatorInfoOut;

    /* Variables used to calculate difference between frames */
    Odometry *prevOdometry;
    int64_t prevTimeStampInMicroSeconds;

    /* Functions used to calculate difference between frames */
    Odometry calculateOdometryDiff(Odometry newOdometry);
    float calculateDtInSeconds(int64_t newTimeStampInMicroSeconds);

    /* Whether to save estimator objects to file */
    const bool saveEstimatorObjectsToFile;

    /* File stream for outputting estimator objects */
    std::ofstream estimatorObjectsOut;
};

#endif // STATE_ESTIMATION_ADAPTER_HPP
