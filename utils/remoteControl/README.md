Remote control Nao's

Installation:
    To use gamepads, the python script using inputs https://pypi.org/project/inputs/
    To install:
       pip install inputs

    If pip is not already installed do: sudo apt install python-pip

Usage:
    The remote control option must be enabled for the remote control to hook into motion.
    Edit image/home/nao/data/runswift.cfg and search for remotecontrol, set this to TRUE. 
    If it does not exist add:
        [debug]
        remotecontrol=TRUE
    After editing this file:
    nao_sync -r <nao ip address>

    To start the transmitter, on your local machine:
    python transmitter.py <nao ip address> <port (defaults to 2000)>

    e.g. python transmitter.py han.local

    If a gamepad is plugged in, it will be used as input, otherwise the keyboard will be used.
    The -k flag can be used to force the keyboard to be used

Possible Errors:
    Different game-pads have different ranges for their joysticks. You will need to account for this

Notes for future development:
    Messages are sent across in key-value pairs e.g. forwardsVelocity=10 leftVelocity=5 turnVelocity=20
    If you want to add kicks or other behaviours in later do the following:
        1. add the necessary key-value pair to SpeedController.__string__() in SpeedController.py
        2. in robot/receiver/RemoteControlReceiver.cpp grab the valyes from the commands map in remoteControlHandler()
