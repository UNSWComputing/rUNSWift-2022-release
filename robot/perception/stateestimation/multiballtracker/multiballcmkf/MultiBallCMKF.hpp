#ifndef MULTI_MODAL_CMKF_HPP
#define MULTI_MODAL_CMKF_HPP

#include <vector>
#include "BCMKF.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class Odometry;
class FieldFeatureInfo;
class MultiBallCMKFTransitioner;
class FieldFeatureLocations;
class MultiBallCMKFParams;
class FieldFeature;
class BallInfo;

/**
 * Ball state estimator which can track multiple balls at the same time
 * Only the ball closest to the robot is pushed onto blackboard
 * 
 * Maintains a dynamic number of BallCMKFs and assigns ball observations based off distance
 * BCMKF weight increases with each new observation and decays over time 
 * 
 * The main difficulty is with rolling balls (they can appear as multiple BallCMKFs in a line)
 * 
 * When using this state estimator you will want to
 * comment out `#define EARLY_EXIT 1` in BallDetector.cpp
 */
class MultiBallCMKF
{
  public:
    MultiBallCMKF(const EstimatorInfoInit& estimatorInfoInit);
    ~MultiBallCMKF();

    /* Function that gets called for every frame */
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

    std::vector<BCMKF> kfs;

  private:
    void predictCollision(BCMKF &kf); // Predict any possible ball collisions with myself
    void predict(BCMKF &kf, float dtInSeconds);
    void predictWithOdometry(BCMKF &kf, const Odometry &odometry);

    void update(
        BCMKF &kf,
        const BallInfo &ballInfo,
        const EstimatorInfoIn &estimatorInfoIn);
    void mergeCMKFs();
    bool similar(BCMKF &kf1, BCMKF &kf2);
    void merge(BCMKF &kf1, BCMKF &kf2);
    void decayCMKFWeights();
    void deleteLowWeightCMKFs();
    void deleteOffFieldCMKFs(const EstimatorInfoOut &estimatorInfoOut);

    MultiBallCMKFTransitioner *transitioner;

    FieldFeatureLocations *fieldFeatureLocations;

    /* Reference to initialisation information */
    const EstimatorInfoInit &estimatorInfoInit;

    MultiBallCMKFParams *params;
    
    /* Whether we have some ball updates we can send out, since last time we sent data out */
    bool haveOutgoingBallUpdate;

    /* Tracks how long it has been since we last kicked */
    float timeSinceLastKick;
};

#endif // MULTI_MODAL_CMKF_HPP
