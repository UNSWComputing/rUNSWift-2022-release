from util.Hysteresis import Hysteresis
import robot

NEARBY_HIT_DECAY_FRAMES = 10
MIN_FRAMES_TO_SEE_SONAR_OBS = 10
blackboard = None
nearbySonarHysteresis = [
    Hysteresis(-MIN_FRAMES_TO_SEE_SONAR_OBS, NEARBY_HIT_DECAY_FRAMES),
    Hysteresis(-MIN_FRAMES_TO_SEE_SONAR_OBS, NEARBY_HIT_DECAY_FRAMES),
]

LEFT = 0
RIGHT = 1

MAX_SONAR_DIST = 0.5  # m


def update_sonar(newBlackboard):
    """
    Updates the Sonar.py global variables, such as the blackboard.

    Callable via `Sonar.update_sonar(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    updateNearbySonarValues()
    # if hasNearbySonarObject(LEFT):
    #     # print("sl")
    #     # robot.say("sonar left")
    # if hasNearbySonarObject(RIGHT):
    #     # print("sr")
    #     # robot.say("sonar right")


def hasNearbySonarObject(i):
    return nearbySonarHysteresis[i].value > 0


def updateNearbySonarValues():
    for i, sonar in enumerate([robot.Sensors.SonarLeft, robot.Sensors.SonarRight]):
        sonarVal = blackboard.motion.sensors.sensors[sonar]
        if sonarVal < MAX_SONAR_DIST:
            nearbySonarHysteresis[i].up()
        else:
            nearbySonarHysteresis[i].down()
