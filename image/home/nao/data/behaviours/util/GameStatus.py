import robot
from util.TeamStatus import my_player_number

blackboard = None
gamestate = None
prev_gamestate = None  # Useful for detecting gamestate transitions


def update_game_status(new_blackboad):
    """
    Updates the GameStatus.py global blackboard variable.

    Callable via `GameStatus.update_game_status(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = new_blackboad

    global prev_gamestate
    global gamestate
    prev_gamestate = gamestate
    gamestate = blackboard.gameController.data.state


# Game State for GameController.
class GameState(object):
    INITIAL = robot.STATE_INITIAL
    READY = robot.STATE_READY
    SET = robot.STATE_SET
    PLAYING = robot.STATE_PLAYING
    FINISHED = robot.STATE_FINISHED


class GamePhase(object):
    GAME_PHASE_NORMAL = robot.GAME_PHASE_NORMAL
    GAME_PHASE_PENALTYSHOOT = robot.GAME_PHASE_PENALTYSHOOT
    # Note: Could not get GameController2015 to actually send OVERTIME
    #       but tested GAME_PHASE_TIMEOUT
    GAME_PHASE_OVERTIME = robot.GAME_PHASE_OVERTIME
    GAME_PHASE_TIMEOUT = robot.GAME_PHASE_TIMEOUT


class SetPlay(object):
    SET_PLAY_NONE = robot.SET_PLAY_NONE
    SET_PLAY_GOAL_KICK = robot.SET_PLAY_GOAL_KICK
    SET_PLAY_PUSHING_FREE_KICK = robot.SET_PLAY_PUSHING_FREE_KICK
    SET_PLAY_CORNER_KICK = robot.SET_PLAY_CORNER_KICK
    SET_PLAY_KICK_IN = robot.SET_PLAY_KICK_IN


# class ChallengePhase(object):
#     AUTO_CALIBRATION_PHASE_1 = robot.AUTO_CALIBRATION_PHASE_1
#     AUTO_CALIBRATION_PHASE_2_1 = robot.AUTO_CALIBRATION_PHASE_2_1
#     AUTO_CALIBRATION_PHASE_2_2 = robot.AUTO_CALIBRATION_PHASE_2_2
#     AUTO_CALIBRATION_PHASE_2_3 = robot.AUTO_CALIBRATION_PHASE_2_3
#     AUTO_CALIBRATION_PHASE_2_4 = robot.AUTO_CALIBRATION_PHASE_2_4
#     AUTO_CALIBRATION_PHASE_2_5 = robot.AUTO_CALIBRATION_PHASE_2_5


class Penalty(object):
    PENALTY_NONE = robot.PENALTY_NONE


def game_state():
    return gamestate


def prev_game_state():
    return prev_gamestate


def game_phase():
    return blackboard.gameController.data.gamePhase


def set_play():
    return blackboard.gameController.data.setPlay


# def challenge_phase():
#     return blackboard.gameController.data.challengePhase


def in_penaltyshoot_phase():
    return game_phase() is GamePhase.GAME_PHASE_PENALTYSHOOT


def penalised():
    return player_number_is_penalised(my_player_number())


def player_number_is_penalised(player_number):
    return blackboard.gameController.our_team.players[player_number - 1].penalty != Penalty.PENALTY_NONE


def our_team_number():
    return blackboard.gameController.our_team.teamNumber


def kicking_team():
    return blackboard.gameController.data.kickingTeam


def we_are_kicking_team():
    return our_team_number() == kicking_team()


def in_goal_kick():
    return set_play() == SetPlay.SET_PLAY_GOAL_KICK


def in_pushing_free_kick():
    return set_play() == SetPlay.SET_PLAY_PUSHING_FREE_KICK


def in_corner_kick():
    return set_play() == SetPlay.SET_PLAY_CORNER_KICK


def in_kick_in():
    return set_play() == SetPlay.SET_PLAY_KICK_IN


def in_initial():
    return game_state() == GameState.INITIAL


def in_ready():
    return game_state() == GameState.READY


def in_set():
    return game_state() == GameState.SET


def in_finished():
    return game_state() == GameState.FINISHED


def secs_remaining():
    return blackboard.gameController.data.secsRemaining


def secondary_time():
    return blackboard.gameController.data.secondaryTime


def secs_till_unpenalised():
    return blackboard.gameController.our_team.players[my_player_number() - 1].secsTillUnpenalised


def whistle_detected():
    return blackboard.gameController.whistleDetected
