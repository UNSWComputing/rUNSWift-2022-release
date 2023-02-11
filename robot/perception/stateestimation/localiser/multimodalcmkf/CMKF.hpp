#ifndef CMKF_HPP
#define CMKF_HPP

#include "types/AbsCoord.hpp"
#include "CMKFConstants.hpp"

class CMKF
{
  public:
    CMKF(
        StateVector state,
        CovarianceMatrix covariance,
        float weight);

    ~CMKF();

    AbsCoord getRobotAbsCoord();
    float getRobotPosUncertainty();
    float getRobotHeadingUncertainty();

    StateVector state;
    CovarianceMatrix covariance;
    float weight;
};

#endif // CMKF_HPP
