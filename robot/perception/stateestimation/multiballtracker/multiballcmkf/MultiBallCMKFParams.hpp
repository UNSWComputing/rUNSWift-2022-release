#ifndef MULTI_MODAL_CMKF_PARAMS_HPP
#define MULTI_MODAL_CMKF_PARAMS_HPP

class MultiBallCMKFParams {

    public:
        MultiBallCMKFParams();
        ~MultiBallCMKFParams();
        void readParams();


        float ballStdAccelerationY;
        float ballStdAccelerationX;
        float BallStdObservationDistBaseWalking;
        float BallStdObservationDistIncreaseRateWalking;
        float BallStdObservationHeadWalking;
        float BallStdObservationDistBaseStanding;
        float BallStdObservationDistIncreaseRateStanding;
        float BallStdObservationHeadStanding;

        float similarThreshLinear;
        float similarThreshConstant;
        float ballCloseDecayRate;
        float ballFarDecayRate;
        float ballWeightGrowth;
        float ballWeightInitial;
};

#endif //MULTI_MODAL_CMKF_PARAMS_HPP
