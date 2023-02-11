#ifndef BALL_CMKF_CONSTANTS_HPP
#define BALL_CMKF_CONSTANTS_HPP

#include <Eigen/Eigen>

#define NUM_DIM_BALL_POS 2
#define NUM_DIM_BALL_VEL 2

#define NUM_DIM_BALL_TOTAL NUM_DIM_BALL_POS + NUM_DIM_BALL_VEL

#define BALL_X_DIM 0
#define BALL_Y_DIM 1
#define BALL_U_DIM 2
#define BALL_V_DIM 3

// Make sure this value is the same as in behaviours (BallMovement.py)
#define BALL_ACCELERATION -390.0f // mm/s^2

#define POSSIBLE_OFF_FIELD_BALL_MARGIN 400 // mm
#define COLLISION_ROBOT_RADIUS 100 // mm
#define COLLISION_COEFF_OF_RESTITUTION 0.4 // no units
#define COLLISION_ACCEPTABLE_HEADING_DIFF DEG2RAD(20) // radians

typedef Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, 1> BallStateVector;
typedef Eigen::Matrix<float, NUM_DIM_BALL_TOTAL, NUM_DIM_BALL_TOTAL> BallCovarianceMatrix;

#endif // BALL_CMKF_CONSTANTS_HPP
