import robot


blankRGB = robot.rgb()


def head(yaw=0, pitch=0, isYawRelative=False, yawSpeed=1.0, pitchSpeed=1.0):
    return robot.HeadCommand(yaw, pitch, isYawRelative, yawSpeed, pitchSpeed)


def stand(power=0.1):
    return walk(0, 0, 0, power, bend=0)


def crouch(power=0.4):
    return walk(0, 0, 0, power, bend=1)


def walk(
    forward=0,
    left=0,
    turn=0,
    power=0.0,
    bend=1,
    speed=1.0,
    foot=robot.Foot.LEFT,
    useShuffle=False,
    leftArmBehind=False,
    rightArmBehind=False,
    blocking=False,
):
    # Positional arguments only http://stackoverflow.com/a/35962682
    return robot.BodyCommand(
        robot.ActionType.WALK,  # actionType
        int(forward),  # forward
        int(left),  # left
        float(turn),  # turn
        float(power),  # power
        float(bend),  # bend
        float(speed),  # speed
        foot,  # foot
        bool(useShuffle),  # shuffle
        bool(leftArmBehind),  # leftArmBehind
        bool(rightArmBehind),  # rightArmBehind
        bool(blocking),  # blocking
        False,  # extraStableKick
    )


def kick(power=1.0, foot=robot.Foot.LEFT, turn=0, extraStableKick=False):
    return robot.BodyCommand(
        robot.ActionType.KICK,  # actionType
        0,  # forward
        0,  # left
        float(turn),  # turn
        power,  # power
        0.0,  # bend
        0.0,  # speed
        foot,  # foot
        False,  # shuffle
        False,  # leftArmBehind
        False,  # rightArmBehind
        False,  # blocking
        bool(extraStableKick),  # extraStableKick
    )


def turnDribble(foot=robot.Foot.LEFT, turn=0):
    return robot.BodyCommand(
        robot.ActionType.TURN_DRIBBLE,  # actionType
        0,  # forward
        0,  # left
        float(turn),  # turn
        1,  # power
        0.0,  # bend
        0.0,  # speed
        foot,  # foot
        False,  # shuffle
        False,  # leftArmBehind
        False,  # rightArmBehind
        False,  # blocking
        False,  # extraStableKick
    )


def motionCalibrate():
    return _type_only_body_command(robot.ActionType.MOTION_CALIBRATE)


def standStraight():
    return _type_only_body_command(robot.ActionType.STAND_STRAIGHT)


def goalieDiveRight():
    return _type_only_body_command(robot.ActionType.GOALIE_DIVE_RIGHT)


def goalieCentre():
    return _type_only_body_command(robot.ActionType.GOALIE_CENTRE)


def goalieUncentre():
    return _type_only_body_command(robot.ActionType.GOALIE_UNCENTRE)


def goalieDiveLeft():
    return _type_only_body_command(robot.ActionType.GOALIE_DIVE_LEFT)


def goalieStand():
    return _type_only_body_command(robot.ActionType.GOALIE_STAND)


def defenderCentre():
    return _type_only_body_command(robot.ActionType.DEFENDER_CENTRE)


def testArms():
    return _type_only_body_command(robot.ActionType.TEST_ARMS)


def raiseArm():
    return _type_only_body_command(robot.ActionType.RAISE_ARM)


def signalKickInRight():
    return _type_only_body_command(robot.ActionType.SIGNAL_KICK_IN_RIGHT)


def signalKickInLeft():
    return _type_only_body_command(robot.ActionType.SIGNAL_KICK_IN_LEFT)


def signalGoalKickRight():
    return _type_only_body_command(robot.ActionType.SIGNAL_GOAL_KICK_RIGHT)


def signalGoalKickLeft():
    return _type_only_body_command(robot.ActionType.SIGNAL_GOAL_KICK_LEFT)


def signalCornerKickRight():
    return _type_only_body_command(robot.ActionType.SIGNAL_CORNER_KICK_RIGHT)


def signalCornerKickLeft():
    return _type_only_body_command(robot.ActionType.SIGNAL_CORNER_KICK_LEFT)


def signalGoalRight():
    return _type_only_body_command(robot.ActionType.SIGNAL_GOAL_RIGHT)


def signalGoalLeft():
    return _type_only_body_command(robot.ActionType.SIGNAL_GOAL_LEFT)


def signalPushingFreeKickRight():
    return _type_only_body_command(robot.ActionType.SIGNAL_PUSHING_FREE_KICK_RIGHT)


def signalPushingFreeKickLeft():
    return _type_only_body_command(robot.ActionType.SIGNAL_PUSHING_FREE_KICK_LEFT)


def signalFullTime():
    return _type_only_body_command(robot.ActionType.SIGNAL_FULL_TIME)


def sit():
    return _type_only_body_command(robot.ActionType.SIT)


def initial():
    return _type_only_body_command(robot.ActionType.INITIAL)


def _type_only_body_command(action_type):
    return robot.BodyCommand(
        action_type,  # actionType
        0,  # forward
        0,  # left
        0,  # turn
        0,  # power
        0.0,  # bend
        0.0,  # speed
        robot.Foot.LEFT,  # foot
        False,  # shuffle
        False,  # leftArmBehind
        False,  # rightArmBehind
        False,  # blocking
        False,  # extraStableKick
    )


def leds(leye=blankRGB, reye=blankRGB, chest=blankRGB, lfoot=blankRGB, rfoot=blankRGB):
    return robot.LEDCommand(leye, reye, chest, lfoot, rfoot)


def compose(head=head(), body=walk(), leds=leds()):
    return robot.All(head, body, leds, 0.0, robot.StiffenCommand.NONE)
