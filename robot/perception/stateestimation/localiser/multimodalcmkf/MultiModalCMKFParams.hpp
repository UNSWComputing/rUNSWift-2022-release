#ifndef MULTI_MODAL_CMKF_PARAMS_HPP
#define MULTI_MODAL_CMKF_PARAMS_HPP

class MultiModalCMKFParams {

    public:
        MultiModalCMKFParams();
        ~MultiModalCMKFParams();
        void readParams();

        float modeSplitWeightMultiplyFactor;
        float lineModeSplitWeightMultiplyFactor;
        float odometryForwardMultiplyFactor;
        float odometryLeftMultiplyFactor;
        float odometryHeadingMultiplyFactor;
        float angleUncertainty;
        float updateHeadingUncertainty;
        float similarHeadingThresh;
        float similarXThresh;
        float similarYThresh;
        float minCMKFWeight;
};

#endif //MULTI_MODAL_CMKF_PARAMS_HPP
