#ifndef MULTI_MODAL_CMKF_HPP
#define MULTI_MODAL_CMKF_HPP

#include <vector>
#include "CMKF.hpp"

class EstimatorInfoInit;
class EstimatorInfoIn;
class EstimatorInfoMiddle;
class EstimatorInfoOut;
class Odometry;
class FieldFeatureInfo;
class MultiModalCMKFTransitioner;
class FieldFeatureLocations;
class MultiModalCMKFParams;
class FieldFeature;

class MultiModalCMKF
{
  public:
    MultiModalCMKF(const EstimatorInfoInit& estimatorInfoInit);
    ~MultiModalCMKF();

    /* Function that gets called for every frame */
    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

    std::vector<CMKF> kfs;

  private:
    void predict(CMKF &kf, const Odometry &odometry);
    void update(
        CMKF &kf,
        const FieldFeatureInfo &observation,
        FieldFeature &ff);
    void mergeCMKFs();
    bool similar(CMKF &kf1, CMKF &kf2);
    void merge(CMKF &kf1, CMKF &kf2);
    void normaliseCMKFWeights();
    void deleteLowWeightCMKFs();
    void deleteOffFieldCMKFs();
    void determineBestCMKF();
    void updateUsingConstantXLine(CMKF &kf, const FieldFeatureInfo &observation, float x, bool onPositiveSideOfLine);
    void updateUsingConstantYLine(CMKF &kf, const FieldFeatureInfo &observation, float y, bool onPositiveSideOfLine);

    CMKF *bestCMKF;

    MultiModalCMKFTransitioner *transitioner;

    FieldFeatureLocations *fieldFeatureLocations;

    /* Reference to initialisation information */
    const EstimatorInfoInit &estimatorInfoInit;

    MultiModalCMKFParams *params;
};

#endif // MULTI_MODAL_CMKF_HPP
