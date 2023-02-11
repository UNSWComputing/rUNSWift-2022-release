# Send ActionCommand::Stiffen commands to remotely stiffen the robot on
# start up
#
# Stiffen commands can be enabled by setting 'behaviour.remote_stiffen'
# to TRUE in 'runswift.cfg' OR in <robot_name>.cfg. If
# 'behaviour.remote_stiffen' is set to TRUE, the robot (when running
# rUNSWift) will automatically stand-up and stiffen.
#
# Set 'behaviour.remote_stiffen' in any of the config files as
# follows:
#   e.g.
#       [behaviour]
#       remote_stiffen=TRUE
#
# Where the valid values are TRUE or FALSE.
#
# If 'behaviour.remote_stiffen' is not found in either 'runswift.cfg' or
# the robot's individual config file, the default value is FALSE.
import robot


# Holds an instance of the blackboard on each tick
blackboard = None

# Indicates whether we've already sent a STIFFEN command
issued_stiffen = False

# The command we're going to send - the default is StiffenCommand.NONE
# (no effect)
command = robot.StiffenCommand.NONE


def update_remote_stiffener(new_blackboard):
    """
    Updates the RemoteStiffener.py global variables, such as `issued_stiffen`.

    Callable via `RemoteStiffener.update_remote_stiffener(blackboard)`.

    :param new_blackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    global issued_stiffen
    global command
    blackboard = new_blackboard

    # The default command is StiffenCommand.NONE (no effect)
    command = robot.StiffenCommand.NONE

    # If we have remote stiffening enabled
    # AND we haven't already issued a stiffen command
    if blackboard.behaviour.remoteStiffen is True and issued_stiffen is False:
        # issue a stiffen command!
        command = robot.StiffenCommand.STIFFEN

    # Set issued_stiffen to TRUE, so we don't issue multiple stiffen
    # commands!
    issued_stiffen = True


# The get_stiffen_command() function is called by 'behaviour.py' to get the
# most recent stiffen command
def get_stiffen_command():
    global command
    return command
