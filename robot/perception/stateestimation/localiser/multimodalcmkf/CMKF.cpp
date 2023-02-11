#include "CMKF.hpp"

#define MAX_INNOVATION_DIM 25

CMKF::CMKF(
    StateVector state,
    CovarianceMatrix covariance,
    float weight)
    : state(state)
    , covariance(covariance)
    , weight(weight)
{
}

CMKF::~CMKF()
{
}

AbsCoord CMKF::getRobotAbsCoord()
{
    AbsCoord robotAbsCoord = AbsCoord(state(ME_X_DIM, 0), state(ME_Y_DIM, 0), state(ME_H_DIM, 0));
    for (unsigned i = 0; i < NUM_DIM_ROBOT; i++)
    {
        for (int j = 0; j < NUM_DIM_ROBOT; j++)
        {
            robotAbsCoord.var(i, j) = covariance(i, j);
        }
    }
    robotAbsCoord.weight = weight;
    return robotAbsCoord;
}

// We define uncertainty as the area of the smallest possible rectangle that can be fit around the
// position covariance ellipse.
float CMKF::getRobotPosUncertainty()
{
    Eigen::Matrix<float, 2, 2> positionCovariance = covariance.block<2, 2>(ME_X_DIM, ME_X_DIM);
    Eigen::EigenSolver<Eigen::Matrix<float, 2, 2> > es(positionCovariance);

    float std_a = sqrt(abs(es.eigenvalues()[0])); // std of one of the ellipse axes
    float std_b = sqrt(abs(es.eigenvalues()[1])); // std of the other ellipse axis

    return std_a * std_b;
}

float CMKF::getRobotHeadingUncertainty()
{
    return sqrt(covariance(ME_H_DIM, ME_H_DIM));
}
