#ifndef BALL_CMKF_PARAMS_HPP
#define BALL_CMKF_PARAMS_HPP

class BallCMKFParams {

    public:
        BallCMKFParams();
        ~BallCMKFParams();
        void readParams();

        float ballStdAccelerationY;
        float ballStdAccelerationX;
        float BallStdObservationDistBaseWalking;
        float BallStdObservationDistIncreaseRateWalking;
        float BallStdObservationHeadWalking;
        float BallStdObservationDistBaseStanding;
        float BallStdObservationDistIncreaseRateStanding;
        float BallStdObservationHeadStanding;
};

#endif // BALL_CMKF_PARAMS_HPP
