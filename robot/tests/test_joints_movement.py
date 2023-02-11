#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import socket
import msgpack
import math
import os
import time

STIFFNESS = 0.6


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

        # not in aldebaran's order.  we test top-down
        self.joint_limits = [
            ("HeadYaw", [-119.5, 119.5],),
            ("HeadPitch", [-38.5, 29.5],),

            ("LShoulderPitch", [-119.5, 119.5],),
            ("RShoulderPitch", [-119.5, 119.5],),
            ("LShoulderRoll", [-18.0, 76],),
            ("RShoulderRoll", [-76.0, 18.0],),

            ("LElbowYaw", [-119.5, 119.5],),
            ("RElbowYaw", [-119.5, 119.5],),
            ("LElbowRoll", [-88.5, -2],),
            ("RElbowRoll", [2.0, 88.5],),
            ("LWristYaw", [-104.5, 104.5],),
            ("RWristYaw", [-104.5, 104.5],),
            # don't test these as they may be taped
            # "LHand", [0.0, 57],),
            # "RHand", [0.0, 57],),

            ("LHipYawPitch", [-65.62, 42.44],),

            ("LHipPitch", [-88.0, 27.73],),
            ("RHipPitch", [-88.0, 27.73],),
            ("LHipRoll", [-21.74, 45.29],),
            ("RHipRoll", [-45.29, 21.74],),
            ("LKneePitch", [-5.29, 121.04],),
            ("RKneePitch", [-5.90, 121.47],),
            ("LAnklePitch", [-68.15, 52.86],),
            ("RAnklePitch", [-67.97, 53.40],),
            ("LAnkleRoll", [-22.79, 44.06],),
            ("RAnkleRoll", [-44.06, 22.80],),
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
        # print "unpack"
        upacker = msgpack.unpackb(stream)
        # print upacker
        return upacker

    def read_joint(self, joint):
        data = self.read()
        positions_value = data["Position"]
        positions = {}
        for index, name in enumerate(self.joints):
            positions[name] = positions_value[index]
        return positions[joint]

    def command(self, category, device, value):
        # print "command ", category, device, value
        self.commands[category][self.actuators[category].index(device)] = value

    def send(self):
        # print "sending"
        stream = msgpack.packb(self.commands)
        self.socket.send(stream)
        # print "sent"

    def close(self):
        self.socket.close()


def main():
    print "Place me flat on ground"
    robot = Robot()
    try:
        for joint, min_max in robot.joint_limits:
            move_and_check_joint(robot, joint, 0)

        for joint, min_max in robot.joint_limits:
            print "checking ", joint
            os.system("/opt/aldebaran/bin/say " + joint)
            # stiffen, start from 0, interpolate to limit, back to 0, unstiffen
            robot.command("Stiffness", joint, STIFFNESS)
            robot.send()

            move_and_check_joint(robot, joint, min_max[0])
            move_and_check_joint(robot, joint, min_max[1])
            move_and_check_joint(robot, joint, 0)

    except KeyboardInterrupt:
        print "Exit"
    finally:
        for index, name in enumerate(robot.joints):
            robot.command("Stiffness", name, 0.0)
        robot.send()
        robot.close()

# interpolate from current position to target. check actual position at the end. report if > 5 degrees different
def move_and_check_joint(robot, joint, target, durationInSeconds=1.0):
    target = math.radians(target)
    current = time.time()
    start = current
    end = current + durationInSeconds
    initial_position = robot.read_joint(joint)
    while current < end:
        # have to read otherwise lola gets stuck
        robot.read()

        progress = (current - start) / (end - start)
        position = initial_position + progress * (target - initial_position)
        print joint, position
        robot.command("Stiffness", joint, STIFFNESS)
        robot.command("Position", joint, position)
        robot.send()
        current = time.time()

    actual_position = robot.read_joint(joint)
    diff = abs(target - actual_position)
    if diff > math.radians(5):
        print "!!!!Joint problem: ", joint, " target ", math.degrees(target), " actual ", math.degrees(actual_position), " diff ", math.degrees(diff)

if __name__ == "__main__":
    main()
