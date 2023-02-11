#include "BCMKF.hpp"

#define MAX_INNOVATION_DIM 25

BCMKF::BCMKF(
    BallStateVector state,
    BallCovarianceMatrix covariance,
    float weight)
    : state(state), covariance(covariance), weight(weight)
{
}

BCMKF::~BCMKF()
{
}

AbsCoord BCMKF::getBallCoordRRC()
{
    AbsCoord ballAbsCoord = AbsCoord(state(BALL_X_DIM, 0), state(BALL_Y_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_BALL_POS; i++)
    {
        for (int j = 0; j < NUM_DIM_BALL_POS; j++)
        {
            ballAbsCoord.var(i, j) = covariance(BALL_X_DIM + i, BALL_X_DIM + j);
        }
    }
    ballAbsCoord.weight = 1;
    return ballAbsCoord;
}

AbsCoord BCMKF::getBallVelCoordRRC()
{
    AbsCoord ballVelAbsCoord = AbsCoord(state(BALL_U_DIM, 0), state(BALL_V_DIM, 0), 0);
    for (unsigned i = 0; i < NUM_DIM_BALL_VEL; i++)
    {
        for (int j = 0; j < NUM_DIM_BALL_VEL; j++)
        {
            ballVelAbsCoord.var(i, j) = covariance(BALL_U_DIM + i, BALL_U_DIM + j);
        }
    }
    ballVelAbsCoord.weight = 1;
    return ballVelAbsCoord;
}

// We define uncertainty as the area of the smallest possible rectangle that can be fit around the
// position covariance ellipse.
float BCMKF::getBallPosUncertainty()
{
    Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_POS> positionCovariance = covariance.block<NUM_DIM_BALL_POS, NUM_DIM_BALL_POS>(BALL_X_DIM, BALL_X_DIM);
    Eigen::EigenSolver<Eigen::Matrix<float, NUM_DIM_BALL_POS, NUM_DIM_BALL_POS> > es(positionCovariance);

    float std_a = sqrt(abs(es.eigenvalues()[0])); // std of one of the ellipse axes
    float std_b = sqrt(abs(es.eigenvalues()[1])); // std of the other ellipse axis

    return std_a * std_b;
}
