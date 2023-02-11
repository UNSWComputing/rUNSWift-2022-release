#ifndef TRANSITION_POSES_HPP
#define TRANSITION_POSES_HPP

#include "SPLDefs.hpp"

#include <boost/math/constants/constants.hpp>

/*
 * X, Y, THETA of robots upon transitioning game states
 */

// Initial State Poses (Px = Player Number x)

#define INITIAL_POSE_P1_X -3500
#define INITIAL_POSE_P1_Y 3000
#define INITIAL_POSE_P1_THETA -boost::math::float_constants::half_pi

#define INITIAL_POSE_P2_X -2000
#define INITIAL_POSE_P2_Y -3000
#define INITIAL_POSE_P2_THETA boost::math::float_constants::half_pi

#define INITIAL_POSE_P3_X -2700
#define INITIAL_POSE_P3_Y 3000
#define INITIAL_POSE_P3_THETA -boost::math::float_constants::half_pi

#define INITIAL_POSE_P4_X -1000
#define INITIAL_POSE_P4_Y -3000
#define INITIAL_POSE_P4_THETA boost::math::float_constants::half_pi

#define INITIAL_POSE_P5_X -1500
#define INITIAL_POSE_P5_Y 3000
#define INITIAL_POSE_P5_THETA -boost::math::float_constants::half_pi

#define INITIAL_POSE_P6_X -3000
#define INITIAL_POSE_P6_Y -3000
#define INITIAL_POSE_P6_THETA boost::math::float_constants::half_pi

#define INITIAL_POSE_DEFAULT_X -4000
#define INITIAL_POSE_DEFAULT_Y -3000
#define INITIAL_POSE_DEFAULT_THETA boost::math::float_constants::half_pi

// Unpenalised Hypotheses (H1 = Hypothesis 1)

#define UNPENALISED_H1_X -PENALTY_CROSS_ABS_X
#define UNPENALISED_H1_Y FIELD_WIDTH / 2.0
#define UNPENALISED_H1_THETA -M_PI / 2.0

#define UNPENALISED_H2_X -PENALTY_CROSS_ABS_X
#define UNPENALISED_H2_Y -FIELD_WIDTH / 2.0
#define UNPENALISED_H2_THETA M_PI / 2.0

#define UNPENALISED_H3_X -FIELD_LENGTH / 6.0
#define UNPENALISED_H3_Y -FIELD_WIDTH / 2.0
#define UNPENALISED_H3_THETA M_PI / 2.0

#define UNPENALISED_H4_X -FIELD_LENGTH / 6.0
#define UNPENALISED_H4_Y FIELD_WIDTH / 2.0
#define UNPENALISED_H4_THETA -M_PI / 2.0

// Manual Placement Poses (H1 = Hypothesis 1)

#define MANUAL_PLACEMENT_H1_X -3666
#define MANUAL_PLACEMENT_H1_Y -464
#define MANUAL_PLACEMENT_H1_THETA 0

#define MANUAL_PLACEMENT_H2_X -3666
#define MANUAL_PLACEMENT_H2_Y 464
#define MANUAL_PLACEMENT_H2_THETA 0

#define MANUAL_PLACEMENT_H3_X -3666
#define MANUAL_PLACEMENT_H3_Y -1998
#define MANUAL_PLACEMENT_H3_THETA 0

#define MANUAL_PLACEMENT_H4_X -3666
#define MANUAL_PLACEMENT_H4_Y 1998
#define MANUAL_PLACEMENT_H4_THETA 0

#define MANUAL_PLACEMENT_GOALIE_X -4250
#define MANUAL_PLACEMENT_GOALIE_Y 0
#define MANUAL_PLACEMENT_GOALIE_THETA 0

#define MANUAL_PLACEMENT_KICKOFF_PLAYER_X -910
#define MANUAL_PLACEMENT_KICKOFF_PLAYER_Y 0
#define MANUAL_PLACEMENT_KICKOFF_PLAYER_THETA 0

// Penalty Shoot Poses

#define PENALTY_SHOOT_OFFENSE_POSE_X 2200
#define PENALTY_SHOOT_OFFENSE_POSE_Y 0
#define PENALTY_SHOOT_OFFENSE_POSE_THETA 0

#define PENALTY_SHOOT_DEFENSE_POSE_X -FIELD_LENGTH / 2
#define PENALTY_SHOOT_DEFENSE_POSE_Y 0
#define PENALTY_SHOOT_DEFENSE_POSE_THETA 0

#define PENALTY_SHOOT_SELECTED_POSE_X -4000
#define PENALTY_SHOOT_SELECTED_POSE_Y -2000
#define PENALTY_SHOOT_SELECTED_POSE_THETA boost::math::float_constants::half_pi

#endif // TRANSITION_POSES_HPP
