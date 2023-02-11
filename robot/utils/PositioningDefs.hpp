#ifndef POSITIONING_DEFS_HPP
#define POSITIONING_DEFS_HPP

// Positioning role enumerations when sending across information
// to other robots
// Start each positioning with a multiple of 10, so we leave
// space that roles can be added to each positioning if needed in the future

// Robot has no role
#define POSITIONING_NONE 0

// Roles from PositioningAgainstKickingTeam
#define POSITIONING_AGAINST_KICKING_TEAM_SUPPORTER 10
#define POSITIONING_AGAINST_KICKING_TEAM_DEFENDER 11
#define POSITIONING_AGAINST_KICKING_TEAM_UPFIELDER 12

// Roles from PositioningFindBall
#define POSITIONING_FIND_BALL_FINDER 20

// Roles from PositioningAgainstDribbleTeam
#define POSITIONING_AGAINST_DRIBBLE_TEAM_RIGHT_SUPPORTER 30
#define POSITIONING_AGAINST_DRIBBLE_TEAM_SHOOTER 31
#define POSITIONING_AGAINST_DRIBBLE_TEAM_LEFT_SUPPORTER 32
#define POSITIONING_AGAINST_DRIBBLE_TEAM_SWEEPER 33


#endif // POSITIONING_DEFS_HPP
