import robot

FIELD_WIDTH = robot.FIELD_WIDTH
HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0
FIELD_LENGTH = robot.FIELD_LENGTH
HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0
GOAL_BOX_WIDTH = robot.GOAL_BOX_WIDTH
GOAL_BOX_LENGTH = robot.GOAL_BOX_LENGTH
CENTER_CIRCLE_DIAMETER = robot.CENTER_CIRCLE_DIAMETER
GOAL_WIDTH = robot.GOAL_WIDTH
BALL_RADIUS = robot.BALL_RADIUS
GOAL_POST_ABS_X = robot.GOAL_POST_ABS_X
GOAL_POST_ABS_Y = robot.GOAL_POST_ABS_Y
GOAL_POST_DIAMETER = robot.GOAL_POST_DIAMETER
ROBOTS_PER_TEAM = robot.ROBOTS_PER_TEAM
PENALTY_CROSS_ABS_X = robot.PENALTY_CROSS_ABS_X
PENALTY_CROSS_DISTANCE = robot.DIST_GOAL_LINE_TO_PENALTY_CROSS
PENALTY_AREA_LENGTH = robot.PENALTY_AREA_LENGTH
PENALTY_AREA_WIDTH = robot.PENALTY_AREA_WIDTH
GOAL_KICK_ABS_X = robot.GOAL_KICK_ABS_X
GOAL_KICK_ABS_Y = robot.GOAL_KICK_ABS_Y
CORNER_KICK_ABS_X = robot.CORNER_KICK_ABS_X
CORNER_KICK_ABS_Y = robot.CORNER_KICK_ABS_Y

# Head joint limits
MIN_HEAD_YAW = robot.HeadYaw_Min
MAX_HEAD_YAW = robot.HeadYaw_Max
MIN_HEAD_PITCH = robot.HeadPitch_Min
MAX_HEAD_PITCH = robot.HeadPitch_Max

# Foot geometry, used for dribblin/approaching the ball
TOE_CENTRE_X = 105
HIP_OFFSET = 50


# LED Colours.
class LEDColour(object):
    off = robot.rgb(False, False, False)
    red = robot.rgb(True, False, False)
    green = robot.rgb(False, True, False)
    blue = robot.rgb(False, False, True)
    yellow = robot.rgb(True, True, False)
    cyan = robot.rgb(False, True, True)
    magenta = robot.rgb(True, False, True)
    white = robot.rgb(True, True, True)


# Time you have to wait before you can enter the circle in a defensive kickoff.
KICKOFF_MIN_WAIT = 10 * 1000 * 1000
