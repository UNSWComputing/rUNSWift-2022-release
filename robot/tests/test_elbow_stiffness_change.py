#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
This script demonstrates that the elbows jump when varying the stiffness
"""

import socket
import msgpack
import math


class Robot:
    def __init__(self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.connect("/tmp/robocup")

        self.joints = [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LHand",
            "RHand"
        ]
        self.sonars = [
            "Left",
            "Right",
        ]
        self.touch = [
            "ChestBoard/Button",
            "Head/Touch/Front",
            "Head/Touch/Middle",
            "Head/Touch/Rear",
            "LFoot/Bumper/Left",
            "LFoot/Bumper/Right",
            "LHand/Touch/Back",
            "LHand/Touch/Left",
            "LHand/Touch/Right",
            "RFoot/Bumper/Left",
            "RFoot/Bumper/Right",
            "RHand/Touch/Back",
            "RHand/Touch/Left",
            "RHand/Touch/Right",
        ]
        self.LEar = [
            "0",
            "36",
            "72",
            "108",
            "144",
            "180",
            "216",
            "252",
            "288",
            "324"
        ]
        self.REar = [
            "324",
            "288",
            "252",
            "216",
            "180",
            "144",
            "108",
            "72",
            "36",
            "0"
        ]
        self.actuators = {
            'Position': self.joints,
            'Stiffness': self.joints,
            'Chest': ['Red', 'Green', 'Blue'],
            'Sonar': self.sonars,
            'LEar': self.LEar,
            'REar': self.REar
        }
        self.commands = {
            'Position': [0.0] * 25,
            'Stiffness': [0.0] * 25,
            'Chest': [0.0] * 3,
            'Sonar': [True, True],
            'LEar': [0.0] * 10,
            'REar': [0.0] * 10
        }

    def read(self):
        stream = self.socket.recv(896)
        upacker = msgpack.unpackb(stream)
        return upacker

    def command(self, category, device, value):
        self.commands[category][self.actuators[category].index(device)] = value

    def send(self):
        stream = msgpack.packb(self.commands)
        self.socket.send(stream)

    def close(self):
        self.socket.close()


def main():
    robot = Robot()
    try:
        data = robot.read()
        positions_value = data["Position"]
        positions = {}
        for index, name in enumerate(robot.joints):
            positions[name] = positions_value[index]
        initial_right_elbow_roll = positions["RElbowRoll"]
        i = 0.0
        FINAL_ITERATION = 133.0
        while i < FINAL_ITERATION:
            data = robot.read()
            positions_value = data["Position"]
            positions = {}
            for index, name in enumerate(robot.joints):
                positions[name] = positions_value[index]

            progress = i / FINAL_ITERATION
            initial_weighted = (1.0 - progress) * initial_right_elbow_roll
            final_weighted = progress * math.pi / 2.0
            right_elbow_roll = initial_weighted + final_weighted
            robot.command("Position", "RElbowRoll", right_elbow_roll)
            # varying the stiffness causes a jump in the motion
            robot.command("Stiffness", "RElbowRoll", progress)
            # # any value is ok
            # robot.command("Stiffness", "RElbowRoll", 0.2)
            # robot.command("Stiffness", "RElbowRoll", 0.6)
            # robot.command("Stiffness", "RElbowRoll", 1.0)

            robot.send()
            i += 1.0
    except KeyboardInterrupt:
        print "Exit"
    finally:
        robot.command("Stiffness", "RElbowRoll", 0.0)
        robot.send()
        robot.close()


if __name__ == "__main__":
    main()
