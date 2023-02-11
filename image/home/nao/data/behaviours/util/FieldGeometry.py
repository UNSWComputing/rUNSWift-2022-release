from util import MathUtil
from util.Vector2D import Vector2D, angleBetween, makeVector2DCopy
from math import radians, atan2, cos, sin
from util.Global import ballWorldPos, myPos, myHeading
from util.Constants import (
    FIELD_LENGTH,
    GOAL_POST_DIAMETER,
    GOAL_POST_ABS_X,
    GOAL_POST_ABS_Y,
    PENALTY_CROSS_ABS_X,
    GOAL_BOX_LENGTH,
    GOAL_BOX_WIDTH,
    PENALTY_AREA_LENGTH,
    PENALTY_AREA_WIDTH,
)
from util.MathUtil import clamp

blackboard = None

# Variables that can be accessed from other classes
_ball_near_our_goal = False
_ball_in_front_of_enemy_goal = False


# Enemy goal vectors.
ENEMY_GOAL_CENTER = Vector2D(FIELD_LENGTH / 2.0, 0)
# +100 offset so angles aren't too sharp near goals
ENEMY_GOAL_BEHIND_CENTER = Vector2D(FIELD_LENGTH / 2.0 + 100, 0)
ENEMY_GOAL_INNER_LEFT = Vector2D(FIELD_LENGTH / 2.0, GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_INNER_RIGHT = Vector2D(FIELD_LENGTH / 2.0, -GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_OUTER_LEFT = Vector2D(FIELD_LENGTH / 2.0, GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER / 2))
ENEMY_GOAL_OUTER_RIGHT = Vector2D(FIELD_LENGTH / 2.0, -GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER / 2))
ENEMY_PENALTY_CENTER = Vector2D(PENALTY_CROSS_ABS_X, 0)
ENEMY_LEFT_POST = Vector2D(GOAL_POST_ABS_X, GOAL_POST_ABS_Y)
ENEMY_RIGHT_POST = Vector2D(GOAL_POST_ABS_X, -GOAL_POST_ABS_Y)

# Own goal vectors.
OWN_GOAL_CENTER = Vector2D(-FIELD_LENGTH / 2.0, 0)
# +100 offset so angles aren't too sharp near goals
OWN_GOAL_BEHIND_CENTER = Vector2D(-FIELD_LENGTH / 2.0 - 100, 0)

OUR_GOAL_CENTRE = Vector2D(-FIELD_LENGTH / 2, 0)
OUR_GOAL_BEHIND_CENTRE = Vector2D(-FIELD_LENGTH / 2 - 100, 0)
OUR_LEFT_POST = Vector2D(-GOAL_POST_ABS_X, GOAL_POST_ABS_Y)
OUR_RIGHT_POST = Vector2D(-GOAL_POST_ABS_X, -GOAL_POST_ABS_Y)


def update_field_geometry(newBlackboard):
    """
    Updates the FieldGeometry.py global variables, i.e. the blackboard.

    Callable via `FieldGeometry.update_field_geometry(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    update_ball_near_our_goal()
    update_ball_in_front_of_enemy_goal()


def calculateTimeToReachBall(robot_pos, robot_heading):
    opponentGoal = ENEMY_GOAL_CENTER
    interceptPoint = ballWorldPos()
    interceptToGoal = opponentGoal.minus(interceptPoint)
    interceptToGoalHeading = MathUtil.normalisedTheta(atan2(interceptToGoal.y, interceptToGoal.x))
    return calculateTimeToReachPose(robot_pos, robot_heading, interceptPoint, interceptToGoalHeading)


TURN_RATE = radians(60.0)  # radians/second
WALK_RATE = 300.0  # mm/second
CIRCLE_STRAFE_RATE = radians(40.0)  # radians/second


def calculateTimeToReachPose(myPos, myHeading, targetPos, targetHeading=None):
    toTarget = targetPos.minus(myPos)
    toTargetHeading = atan2(toTarget.y, toTarget.x)

    # How far we need to turn to point at the targetPos
    toTargetTurn = abs(MathUtil.normalisedTheta(toTargetHeading - myHeading))

    # The straightline distance to walk to the targetPos
    toTargetDistance = toTarget.length()

    # How far we need to turn once we get to the targetPos so that we are
    # facing the targetHeading
    if targetHeading is None:
        toTargetHeadingTurn = 0.0
    else:
        toTargetHeadingTurn = abs(MathUtil.normalisedTheta(toTargetHeading - targetHeading))

    return toTargetTurn / TURN_RATE + toTargetDistance / WALK_RATE + toTargetHeadingTurn / CIRCLE_STRAFE_RATE


def angleToPoint(point, absCoord):
    phi = angleBetween(point, makeVector2DCopy(absCoord))
    return MathUtil.normalisedTheta(phi)


# Whether something is inside our goalbox
def isInOurGoalBox(pos, buffx=0, buffy=0):
    return pos.x < -FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + buffx and abs(pos.y) < GOAL_BOX_WIDTH / 2 + buffy


# Whether something is in the opponent goalbox
def isInOpponentGoalBox(pos, buffx=0, buffy=0):
    return pos.x > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - buffx and abs(pos.y) < GOAL_BOX_WIDTH / 2 + buffy


# Whether something is inside our goalbox
def isInOurPenaltyBox(pos, buffx=0, buffy=0):
    return pos.x < -FIELD_LENGTH / 2 + PENALTY_AREA_LENGTH + buffx and abs(pos.y) < PENALTY_AREA_WIDTH / 2 + buffy


# Whether something is in the opponent goalbox
def isInOpponentPenaltyBox(pos, buffx=0, buffy=0):
    return pos.x > FIELD_LENGTH / 2 - PENALTY_AREA_LENGTH - buffx and abs(pos.y) < PENALTY_AREA_WIDTH / 2 + buffy


def addRrToRobot(robotPos, rx, ry):
    x = robotPos.x + cos(robotPos.theta) * rx - sin(robotPos.theta) * ry
    y = robotPos.y + sin(robotPos.theta) * rx + cos(robotPos.theta) * ry
    return x, y


def globalPointToRobotRelativePoint(globalVector):

    robotPos = myPos()
    robotHeading = myHeading()

    return globalVector.minus(robotPos).rotate(-robotHeading)


# Closes y-value from ball to our goal line between posts
def closest_goal_y():
    return clamp(ballWorldPos().y, -GOAL_POST_ABS_Y, GOAL_POST_ABS_Y)


# Closest point from the ball to our goal line between posts
def closest_our_goal_point():
    return Vector2D(-GOAL_POST_ABS_X, closest_goal_y())


# Closest point from the ball to our goal line between posts
def closest_opponent_goal_point():
    return Vector2D(GOAL_POST_ABS_X, closest_goal_y())


# Update whether ball is near our goal, with a noise margin
def update_ball_near_our_goal():
    dist_ball_to_our_goal = ballWorldPos().distanceTo(closest_our_goal_point())

    global _ball_near_our_goal
    if _ball_near_our_goal:
        if dist_ball_to_our_goal > 1500:
            _ball_near_our_goal = False
    else:
        if dist_ball_to_our_goal < 1100:
            _ball_near_our_goal = True


# Update whether ball is in front of opponent goal, with a noise margin
def update_ball_in_front_of_enemy_goal():
    global _ball_in_front_of_enemy_goal
    if _ball_in_front_of_enemy_goal:
        if ballWorldPos().x < FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - 500 or abs(ballWorldPos().y) > GOAL_POST_ABS_Y + 100:
            _ball_in_front_of_enemy_goal = False
    else:
        if (
            ballWorldPos().x > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - 250
            and abs(ballWorldPos().y) < GOAL_POST_ABS_Y - 100
        ):
            _ball_in_front_of_enemy_goal = True


def ball_near_our_goal():
    return _ball_near_our_goal


def ball_in_front_of_enemy_goal():
    return _ball_in_front_of_enemy_goal
