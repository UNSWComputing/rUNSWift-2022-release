from util.Vector2D import Vector2D
from util.Global import myPos, ballDistance, ballLostTime, myHeading
from math import acos, radians
from util.Timer import WallTimer
from util.Sonar import hasNearbySonarObject, LEFT, RIGHT
from util.MathUtil import angleSignedDiff
from robot import Sensors
from util.FieldGeometry import ENEMY_GOAL_CENTER

"""
Description:
Functions to help calculate obstacle avoidance behaviours. The obstacle
can be a robot, a goal post or even the ball (as it is an obstacle while
we approah the ball).
"""

blackboard = None

# Timers for how long a sensor has been clear of obstacles
lfoot_bumper_clear_timer = None
rfoot_bumper_clear_timer = None
lsonar_clear_timer = None
rsonar_clear_timer = None

# Minimum time to continue avoiding an obstacle we detected
BUMPER_CLEAR_MINIMUM_SECONDS = 1.0
SONAR_CLEAR_MINIMUM_SECONDS = 0.2

# assumption of headings of obstacles when detected, relative to my heading
lfoot_bumper_obs_heading = radians(10)
rfoot_bumper_obs_heading = radians(-10)
lsonar_obs_heading = radians(30)
rsonar_obs_heading = radians(-30)


def update_obstacle_avoidance(newBlackboard):
    """
    Updates the ObstacleAvoidance.py global variables.

    Callable via `ObstacleAvoidance.update_obstacle_avoidance(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard

    sensorValues = blackboard.motion.sensors.sensors

    # Update left and right foot bumpers
    global lfoot_bumper_clear_timer, rfoot_bumper_clear_timer
    if lfoot_bumper_clear_timer is None:
        lfoot_bumper_clear_timer = WallTimer()
    if rfoot_bumper_clear_timer is None:
        rfoot_bumper_clear_timer = WallTimer()

    if sensorValues[Sensors.LFoot_Bumper_Left] or sensorValues[Sensors.LFoot_Bumper_Right]:
        lfoot_bumper_clear_timer.restart()
    if sensorValues[Sensors.RFoot_Bumper_Left] or sensorValues[Sensors.RFoot_Bumper_Right]:
        rfoot_bumper_clear_timer.restart()

    # Update left and right sonars
    global lsonar_clear_timer, rsonar_clear_timer
    if lsonar_clear_timer is None:
        lsonar_clear_timer = WallTimer()
    if rsonar_clear_timer is None:
        rsonar_clear_timer = WallTimer()

    if hasNearbySonarObject(LEFT):
        lsonar_clear_timer.restart()
    if hasNearbySonarObject(RIGHT):
        rsonar_clear_timer.restart()


# Calculate tangent point from obstacle avoidance in global coordinates
# NOTE: Check if the returned value is not "None" before using! None is used
#       when a tangent cannot be generated because robot is within the radius
def calculate_tangent_point(
    centre=Vector2D(0, 0),  # Centre of obstacle in global coordinates
    radius=0,  # Radius of avoidance (mm)
    left_side=True,
):  # Whether we are considering the left tangent point, seen from the robot  # noqa

    # vector from robot to centre
    robot_to_centre = centre.minus(myPos())

    # if we're inside the radius, tangent cant be generated! return None
    if robot_to_centre.length() <= radius:
        return None

    # angle between the two lines:
    # 1. Line joining centre to tangent point
    # 2. Line joining centre to robot
    alpha = acos(radius / robot_to_centre.length())

    # Radius-length vector from centre towards robot
    centre_to_robot_radius = Vector2D(radius, 0).rotated(robot_to_centre.multiply(-1).heading())

    # Radius-length vector from centre towards tangent point
    centre_to_t_p = centre_to_robot_radius.rotated(-alpha if left_side else alpha)

    return myPos().plus(robot_to_centre).plus(centre_to_t_p)


# Calculate modified walk vector (forward (mm/s), left (mm/s)), considering any
# foot bumper or sonar obstacles
# We consider obstacles as solid walls facing us at specific headings, and the
# robot must walk along the wall if the wall is in the way of the desired walk
# Input: Vector2D(forward(mm/s), left(mm/s))
# Output: Vector2D(forward(mm/s), left(mm/s)) considering obstacle avoidance
def walk_vec_with_avoidance(walk_vec):
    if (
        ballDistance() < 350
        and ballLostTime() < 2.0
        and ENEMY_GOAL_CENTER.minus(myPos()).length() < 1300
        and myHeading() > radians(-60)
        and myHeading() < radians(60)
    ):
        return walk_vec

    walk_vec_heading = walk_vec.heading()

    # Create walk_vector which faces forwards, which we then
    # rotate, depending on the obstacle
    walk_vector_forwards = Vector2D(walk_vec.length(), 0)

    #
    # FOOT BUMPER AVOIDANCE - to avoid walking into fallen robots, standing
    # robots and goal posts
    # NOTE: Consider foot bumpers before sonar, as it is more important to
    # avoid walking into fallen robots, as you can get penalised for it.
    #

    # Check whether we have an lfoot bumper obstacle that interferes the walk
    lfoot_bumper_interferes_walk = False
    if lfoot_bumper_clear_seconds() < BUMPER_CLEAR_MINIMUM_SECONDS:
        angle_lfoot_bumper_obs_to_walk_vec_heading = angleSignedDiff(walk_vec_heading, lfoot_bumper_obs_heading)
        if radians(-90) < angle_lfoot_bumper_obs_to_walk_vec_heading < radians(90):  # noqa
            lfoot_bumper_interferes_walk = True

    # Check whether we have an rfoot bumper obstacle that interferes the walk
    rfoot_bumper_interferes_walk = False
    if rfoot_bumper_clear_seconds() < BUMPER_CLEAR_MINIMUM_SECONDS:
        angle_rfoot_bumper_obs_to_walk_vec_heading = angleSignedDiff(walk_vec_heading, rfoot_bumper_obs_heading)
        if radians(-90) <= angle_rfoot_bumper_obs_to_walk_vec_heading < radians(90):  # noqa
            rfoot_bumper_interferes_walk = True

    # If both bumpers register obstacles that interfere with the walk,
    # see which bumper has registered an obstacle more recently, and
    # decide if we want to perform a side step, away from that obstacle.
    if lfoot_bumper_interferes_walk and rfoot_bumper_interferes_walk:
        if lfoot_bumper_clear_seconds() > rfoot_bumper_clear_seconds():
            lfoot_bumper_interferes_walk = False
        else:
            rfoot_bumper_interferes_walk = False

    # If walk interferes with the foot bumpers, we walk
    # 90 degrees to the bumper obstacle
    if lfoot_bumper_interferes_walk:
        if 0 <= angle_lfoot_bumper_obs_to_walk_vec_heading < radians(90):
            return walk_vector_forwards.rotated(lfoot_bumper_obs_heading + radians(90))
        elif radians(-90) < angle_lfoot_bumper_obs_to_walk_vec_heading < 0:
            return walk_vector_forwards.rotated(lfoot_bumper_obs_heading + radians(-90))
    if rfoot_bumper_interferes_walk:
        if 0 <= angle_rfoot_bumper_obs_to_walk_vec_heading < radians(90):
            return walk_vector_forwards.rotated(rfoot_bumper_obs_heading + radians(90))
        elif radians(-90) < angle_rfoot_bumper_obs_to_walk_vec_heading < 0:
            return walk_vector_forwards.rotated(rfoot_bumper_obs_heading + radians(-90))

    #
    # SONAR AVOIDANCE - to avoid walking into other robots and goal posts.
    #

    # Check whether we have an lsonar obstacle that interferes the walk
    lsonar_interferes_walk = False
    if sonar_left_obstacle(clear_seconds=SONAR_CLEAR_MINIMUM_SECONDS):
        angle_lsonar_obs_to_walk_vec_heading = angleSignedDiff(walk_vec_heading, lsonar_obs_heading)
        if radians(-90) < angle_lsonar_obs_to_walk_vec_heading < radians(90):  # noqa
            lsonar_interferes_walk = True

    # Check whether we have an rsonar obstacle that interferes the walk
    rsonar_interferes_walk = False
    if sonar_right_obstacle(clear_seconds=SONAR_CLEAR_MINIMUM_SECONDS):
        angle_rsonar_obs_to_walk_vec_heading = angleSignedDiff(walk_vec_heading, rsonar_obs_heading)
        if radians(-90) <= angle_rsonar_obs_to_walk_vec_heading < radians(90):  # noqa
            rsonar_interferes_walk = True

    # If both sonars register obstacles that interfere with the walk,
    # use sonar that has registered an obstacle more recently
    if lsonar_interferes_walk and rsonar_interferes_walk:
        if lsonar_clear_seconds() > rsonar_clear_seconds():
            lsonar_interferes_walk = False
        else:
            rsonar_interferes_walk = False

    # If walk interferes with the either sonar, we walk
    # 90 degrees to the bumper obstacle. Direction depends on
    # the angle between sonar obstacle and walk vec heading
    if lsonar_interferes_walk:
        if 0 <= angle_lsonar_obs_to_walk_vec_heading < radians(90):
            return walk_vector_forwards.rotated(lsonar_obs_heading + radians(90))
        elif radians(-90) < angle_lsonar_obs_to_walk_vec_heading < 0:
            return walk_vector_forwards.rotated(lsonar_obs_heading + radians(-90))
    if rsonar_interferes_walk:
        if 0 <= angle_rsonar_obs_to_walk_vec_heading < radians(90):
            return walk_vector_forwards.rotated(rsonar_obs_heading + radians(90))
        elif radians(-90) < angle_rsonar_obs_to_walk_vec_heading < 0:
            return walk_vector_forwards.rotated(rsonar_obs_heading + radians(-90))

    return walk_vec


def lsonar_clear_seconds():
    return lsonar_clear_timer.elapsedSeconds()


def rsonar_clear_seconds():
    return rsonar_clear_timer.elapsedSeconds()


def lfoot_bumper_clear_seconds():
    return lfoot_bumper_clear_timer.elapsedSeconds()


def rfoot_bumper_clear_seconds():
    return rfoot_bumper_clear_timer.elapsedSeconds()


def sonar_left_obstacle(clear_seconds=2.0):
    return lsonar_clear_seconds() < clear_seconds


def sonar_right_obstacle(clear_seconds=2.0):
    return rsonar_clear_seconds() < clear_seconds
