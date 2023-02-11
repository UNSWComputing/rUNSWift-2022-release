from util.Constants import ROBOTS_PER_TEAM
from util.Vector2D import Vector2D
from util.Constants import CENTER_CIRCLE_DIAMETER
from util.Timer import Timer

blackboard = None
data = None
active_player_numbers = []
player_number_with_last_ball_update = 0
players_timer_since_ball_update = [None for _ in range(ROBOTS_PER_TEAM)]
players_seconds_since_ball_update = [100000.0 for _ in range(ROBOTS_PER_TEAM)]

LEFT_MIDFIELDER_CHARGE_TARGET = Vector2D(2000, 2000)
RIGHT_MIDFIELDER_CHARGE_TARGET = Vector2D(2000, -2000)
LEFT_KICK_OFF_TARGET = LEFT_MIDFIELDER_CHARGE_TARGET.minus(Vector2D(-500, 0))
RIGHT_KICK_OFF_TARGET = RIGHT_MIDFIELDER_CHARGE_TARGET.minus(Vector2D(-500, 0))  # noqa


def update_team_status(new_blackboard):
    """
    Updates the TeamStatus.py global variables.

    Callable via `TeamStatus.update_team_status(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = new_blackboard

    global data
    data = blackboard.receiver.data

    update_active_player_numbers()

    update_players_seconds_since_ball_update()

    update_player_number_with_last_ball_update()


def update_active_player_numbers():
    global active_player_numbers
    active_player_numbers = []
    for player_number in range(1, ROBOTS_PER_TEAM + 1):
        if not player_number_is_incapacitated(player_number):
            active_player_numbers.append(player_number)


# Get a very rough estimate of who sent the last ball update
def update_player_number_with_last_ball_update():
    global player_number_with_last_ball_update
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].sharedStateEstimationBundle.haveBallUpdate:
            player_number_with_last_ball_update = player


# Update how long ago each teammate's ball update was (seconds)
def update_players_seconds_since_ball_update():
    global players_timer_since_ball_update
    global players_seconds_since_ball_update
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].sharedStateEstimationBundle.haveBallUpdate:
            players_timer_since_ball_update[player_index] = Timer()
        if players_timer_since_ball_update[player_index] is not None:
            players_seconds_since_ball_update[player_index] = (
                players_timer_since_ball_update[player_index].elapsed() / 1000000.0
            )  # noqa


def my_player_number():
    return blackboard.gameController.player_number


def player_number_is_incapacitated(player_number):
    player_index = player_number_to_player_index(player_number)
    return blackboard.receiver.incapacitated[player_index]


def player_number_to_player_index(player_number):
    return player_number - 1


def player_index_to_player_number(player_index):
    return player_index + 1


def pose_of_player_number(player_number):
    player_index = player_number_to_player_index(player_number)
    pose = data[player_index].robotPos
    return (Vector2D(pose[0], pose[1]), pose[2])


def player_numbers_playing_ball():
    player_numbers = []
    for player_number in active_player_numbers:
        if player_is_playing_ball(player_number):
            player_numbers.append(player_number)
    return player_numbers


def player_is_playing_ball(player_number):
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.playingBall


def player_numbers_assisting():
    player_numbers = []
    for player_number in active_player_numbers:
        if player_is_assisting(player_number):
            player_numbers.append(player_number)
    return player_numbers


def player_is_assisting(player_number):
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.isAssisting


def assistance_is_needed():
    for player_number in player_numbers_playing_ball():
        if player_number_needs_assistance(player_number):
            return True
    return False


def player_number_seconds_since_last_kicked(player_number):
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.secondsSinceLastKick


def player_number_needs_assistance(player_number):
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.needAssistance


def player_number_that_kicked_ball_last():
    min_seconds_since_last_kick = None
    player_number_that_kicked_ball_last = None
    for player_number in active_player_numbers:
        if (min_seconds_since_last_kick is None or player_number_that_kicked_ball_last is None) and (
            player_number_seconds_since_last_kicked(player_number) >= 0
        ):  # noqa
            min_seconds_since_last_kick = player_number_seconds_since_last_kicked(player_number)  # noqa
            player_number_that_kicked_ball_last = player_number
        else:
            if min_seconds_since_last_kick > player_number_seconds_since_last_kicked(player_number) and (
                player_number_seconds_since_last_kicked(player_number) >= 0
            ):  # noqa
                min_seconds_since_last_kick = player_number_seconds_since_last_kicked(player_number)  # noqa
                player_number_that_kicked_ball_last = player_number
    if player_number_that_kicked_ball_last is None:
        return 0
    return player_number_that_kicked_ball_last


def i_kicked_the_ball_last():
    return player_number_that_kicked_ball_last() is my_player_number()


def i_saw_ball_last():
    return player_number_with_last_ball_update is my_player_number()


def get_player_number_with_last_ball_update():
    return player_number_with_last_ball_update


def get_teammate_seconds_since_last_ball_update(player_number):
    index = player_number_to_player_index(player_number)
    return players_seconds_since_ball_update[index]


def get_teammate_pos(player_number):
    index = player_number_to_player_index(player_number)
    pos = blackboard.receiver.data[index].robotPos
    return Vector2D(pos[0], pos[1])


def get_teammate_heading(player_number):
    index = player_number_to_player_index(player_number)
    pos = blackboard.receiver.data[index].robotPos
    return pos[2]


def get_active_player_numbers():
    return active_player_numbers


def get_kick_off_target():
    # check for team mates in a good position

    # We go through active player numbers in a different order
    # if we want to kick off right. Toggle following flag to prefer left kick \
    # off or not.
    prefer_left_kick_off = False
    if prefer_left_kick_off:
        players = active_player_numbers[:]  # prefer left
    else:
        players = active_player_numbers[::-1]  # prefer right

    for player in players:
        player_pos = get_teammate_pos(player)

        if player_pos.y < -1000 and abs(player_pos.x) < 1000:
            return RIGHT_KICK_OFF_TARGET, player

        if player_pos.y > 1000 and abs(player_pos.x) < 1000:
            return LEFT_KICK_OFF_TARGET, player

    return None, None


def check_teammate_already_kick_off():
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].behaviourSharedData.isKickedOff:
            return True
    return False


def teammate_is_near_centre_circle():
    for player_number in active_player_numbers:
        pos = get_teammate_pos(player_number)
        if pos.length() < CENTER_CIRCLE_DIAMETER / 2 + 200:
            return True
    return False


def player_role(player_number):
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.role


# Whether someone on the team is notifying about a kick
def kick_notified():
    for player in active_player_numbers:
        index = player_number_to_player_index(player)
        if data[index].behaviourSharedData.kickNotification:
            return True
    return False


def teammate_ego_ball(player_number):
    index = player_number_to_player_index(player_number)
    pos = blackboard.receiver.data[index].ballPosAbs
    return Vector2D(pos.x, pos.y)


def player_one_is_field_player():
    return blackboard.behaviour.positioning == "PositioningAgainstDribbleTeam"
