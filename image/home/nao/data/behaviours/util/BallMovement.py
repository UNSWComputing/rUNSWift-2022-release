from util.Global import ballRelPos, ballRelVel, ballWorldPos, ballWorldVel, egoBallWorldPos, egoBallWorldVel
from util.Vector2D import Vector2D
from math import sqrt
from util.Constants import GOAL_POST_ABS_X

# Ball acceleration on carpet, must be tuned for different carpets, and should
# be the same value as in state-estimation (BallCMKFConstants.hpp).
BALL_ACCELERATION = -390.0  # mm/s^2


# Calculate time for ball to reach robot's coronal plane without friction
def timeToReachCoronalPlaneNoFriction():
    pos = ballRelPos()
    vel = ballRelVel()
    if abs(vel.x) < 1:
        vel.x = 1
    t = (0 - pos.x) / vel.x
    return t


# Calculate time for ball to reach our goal base line without friction
def timeToReachOurGoalBaseLineNoFriction():
    pos = ballWorldPos()
    vel = ballWorldVel()
    if abs(vel.x) < 1:
        vel.x = 1
    t = (-GOAL_POST_ABS_X - pos.x) / vel.x
    return t


# Calculate time for egoball to reach our goal base line without friction
def egoBallTimeToReachOurGoalBaseLineNoFriction():
    pos = egoBallWorldPos()
    vel = egoBallWorldVel()
    if abs(vel.x) < 1:
        vel.x = 1
    t = (-GOAL_POST_ABS_X - pos.x) / vel.x
    return t


# Calculate time for ball to reach robot's coronal plane with friction
# 1. If ball doesn't reach the robot, return a huge number
# 2. If ball is going to reach, calculate time it takes to reach by:
#     s = ut + 0.5at^2    solving it gives:  t = (-u - sqrt(u^2 + 2as)) / a
#     where all variables are in the robot-relative x-direction
def timeToReachCoronalPlaneWithFriction():
    pos = ballRelPos()
    vel = ballRelVel()
    stop_rel_pos = stopRelPos()

    # 1. Check if the ball travels from infront/behind robot or vice versa
    if stop_rel_pos.x * pos.x > 0:
        return 100000.0

    # 2. If ball is going to reach, calculate how much time it will take
    a = Vector2D(BALL_ACCELERATION, 0).rotate(vel.heading()).x
    u = vel.x
    s = -pos.x
    t = (-u - sqrt(u * u + 2 * a * s)) / a

    return t


# Where the ball is going to stop rolling in world coordinates.
def stopWorldPos():
    return stopPos(ballWorldPos(), ballWorldVel())


# Where the ego ball is going to stop rolling in world coordinates.
def egoBallStopWorldPos():
    return stopPos(egoBallWorldPos(), egoBallWorldVel())


# Where the ball is going to stop rolling in relative coordinates.
def stopRelPos():
    return stopPos(ballRelPos(), ballRelVel())


# Calculate where the ball is going to stop rolling.
# 1. Calculate distance until ball stops, by:
#     v^2 = u^2 + 2as     rearraging gives,   s = -u^2 / 2a, since v = 0
# 2. Calculate where the ball will stop.
def stopPos(pos, vel):
    # 1. Distance ball travels before stopping
    dist_until_stop = -vel.length2() / (2.0 * BALL_ACCELERATION)

    # 2. Calculate position of ball when it stops rolling
    stop_pos = pos.plus(Vector2D(dist_until_stop, 0).rotate(vel.heading()))

    return stop_pos


# Calculate where along the coronal plane the ball will intersect the
# robot's coronal plane, assuming constant velocity
def YWhenReachCoronalPlane():
    pos = ballRelPos()
    vel = ballRelVel()
    t = timeToReachCoronalPlaneNoFriction()
    final_y = pos.y + vel.y * t
    return final_y


# Calculate where along the goal base line the ball will intersect the
# baseline, assuming constant velocity. Used for goalie
def YWhenReachOurGoalBaseLine():
    pos = ballWorldPos()
    vel = ballWorldVel()
    t = timeToReachOurGoalBaseLineNoFriction()
    final_y = pos.y + vel.y * t
    return final_y


# Calculate where along the goal base line the ball will intersect the
# baseline, assuming constant velocity. Used for goalie
def egoBallYWhenReachOurGoalBaseLine():
    pos = egoBallWorldPos()
    vel = egoBallWorldVel()
    t = egoBallTimeToReachOurGoalBaseLineNoFriction()
    final_y = pos.y + vel.y * t
    return final_y
