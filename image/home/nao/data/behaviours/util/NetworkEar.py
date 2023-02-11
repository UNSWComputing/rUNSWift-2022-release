# Set colour of ear according to network activity
#
# For each robot, turn on (or off) an LED (correlating to their player
# number) on the right ear to indicate network activity. There are 10
# LEDs so we have one for each player number with an unused LED between
# each. If we have no active robots (including the current robot), we
# spin the lights on the ear to indicate total inactivity.
#
# The light at the top indicates player 1, and the following players are
# represented in clockwise order around the ear. At the moment we set
# every second LED starting at index 0 (there are 10) so if the number of
# players increases we need to change the functionality below too.
#
# NOTE: Robots will still receive their own broadcast packets even if
# they are not connected to a network. As such, the robot may appear to
# be on a network (the LED for the robot's own player number will be lit)
# when it is not actually connected to any.
#
# Future work: Implement some kind of mechanism that can detect whether
# or not a robot is actually connected to a network, instead of relying
# on the timestamp of the last packet received from itself.
import time


# Number of SECONDS since we last heard from a robot before we consider
# them to be network inactive
SECONDS_BEFORE_ROBOT_INACTIVE = 1

# Number of frames with all robots network inactive before we start spinning
# the lights on the ear
FRAMES_BEFORE_SPINNING_EAR = 50

blackboard = None
frame = 0
ear_colour = 0
rotating_ear_colour = 1


def update_network_ear(new_blackboard):
    """
    Updates the NetworkEar.py global variables, such as `rotating_ear_colour`.

    Callable via `NetworkEar.update_network_ear(blackboard)`.

    :param new_blackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    global frame
    global ear_colour
    global rotating_ear_colour
    blackboard = new_blackboard

    network_ear_colour = 0
    now = time.time()
    for player in range(0, 5):
        player_time = blackboard.receiver.lastReceived[player]
        if player_time > 0:
            time_since_seconds = now - player_time
            if time_since_seconds < SECONDS_BEFORE_ROBOT_INACTIVE:
                # We have 10 LEDs so let's set every second LED
                network_ear_colour |= 1 << (player * 2)

    # If we aren't on the network (we haven't received packets from anyone
    # including ourselves for some time limit), let's light up our right
    # ear in circles so we know (oooo pretty)
    if network_ear_colour == 0:
        frame += 1
        if frame % 2 == 0:
            rotating_ear_colour <<= 1
        if rotating_ear_colour > 1023:
            rotating_ear_colour = 1
        if frame >= FRAMES_BEFORE_SPINNING_EAR:
            ear_colour = rotating_ear_colour
            if frame >= FRAMES_BEFORE_SPINNING_EAR * 2:
                # Let's keep this number small
                frame = FRAMES_BEFORE_SPINNING_EAR

    # Otherwise, if we have active robots, lets set our ear to show
    # network activity
    else:
        frame = 0
        ear_colour = network_ear_colour


def get_ear_colour():
    global ear_colour
    return ear_colour
