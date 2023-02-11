#ifndef CMKF_HPP
#define CMKF_HPP

#include "types/AbsCoord.hpp"
#include "CMKFConstants.hpp"

class BCMKF
{
  public:
    BCMKF(
        BallStateVector state,
        BallCovarianceMatrix covariance,
        float weight);

    ~BCMKF();

    AbsCoord getBallCoordRRC();
    AbsCoord getBallVelCoordRRC();
    float getBallPosUncertainty();
    float getBallHeadingUncertainty();

    BallStateVector state;
    BallCovarianceMatrix covariance;
    float weight;
};

#endif // CMKF_HPP
