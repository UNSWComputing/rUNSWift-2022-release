#!/usr/bin/python
import argparse
import os
import subprocess
import sys

TEMPLATE_PARAMETERS = {
    "2.1": {
        "wpa_supplicant_filename": "/etc/wpa_supplicant/wpa_supplicant.conf",
        "runswiftwireless":        "/etc/init.d/runswiftwireless",
        "script":                  "/sbin/runscript",
        "run":                     "",
        "copy":                    "scp %s root@%s.local:/etc/init.d/runswiftwireless",
    },
    "2.8": {
        "wpa_supplicant_filename": "/etc/wpa_supplicant.conf",
        "runswiftwireless":        "/etc/init.d/runswiftwireless.sh",
        "script":                  "/bin/bash",
        "run":                     "$1",
        "copy":                    "rsync -aP --rsync-path='sudo rsync' %s nao@%s.local:/etc/init.d/runswiftwireless.sh",
    }
}

ROBOTS = {
   # Please specify a two-digit IP, with a leading zero if necessary
   "robot1":  {"IP": "00", "OS": "2.1"},
   "robot2":  {"IP": "01", "OS": "2.1"},
    #2018 v6s
   "robot3":  {"IP": "33", "OS": "2.8"},
   "robot4":  {"IP": "34", "OS": "2.8"},
    #2019-05-28 v6s
   "robot5":  {"IP": "39", "OS": "2.8"},
   "robot6":  {"IP": "40", "OS": "2.8"},
}

parser = argparse.ArgumentParser(description='Updates WLAN Setup on robots')
parser.add_argument('robots', metavar='ROBOT', nargs='+',
                    help='the name of the robot to setup')
parser.add_argument('--broadcast', default='10.0.255.255',
                    help="broadcast address, should start the "
                         "same as prefix and end with 2 or 3 .255's")
parser.add_argument('--gateway', default='10.0.0.1',
                    help='gateway address, should start the same '
                         'as prefix and be less than broadcast')
parser.add_argument('--netmask', default='255.255.0.0',
                    help="netmask of subnet, should start "
                         "with 1 or 2 255.'s and end with 2 or 3 .0's")
parser.add_argument('--prefix', default='10.0.18.1',
                    help="prefix of robot's IP, should end with '.1'")
args = parser.parse_args()

script_dir = os.path.dirname(__file__)  # <-- absolute dir the script is in

# Accept a robot name.
if not args.robots:
   sys.exit("No robot given")

for robotName in args.robots:
    if robotName not in ROBOTS:
        sys.exit("Invalid robot name")

    # Generate the filled in runswiftwireless file.
    runswiftWirelessFinalise = "/tmp/runswiftwireless"
    runswiftWirelessTemplate = os.path.join(script_dir, "runswiftwireless_template")

    template = open(runswiftWirelessTemplate, "r").read()
    templateParameters = TEMPLATE_PARAMETERS[ROBOTS[robotName]["OS"]]
    templateParameters["playerIP"] = args.prefix + ROBOTS[robotName]["IP"]
    templateParameters["gateway"] = args.gateway
    templateParameters["netmask"] = args.netmask
    filledIn = template % templateParameters

    output = open(runswiftWirelessFinalise, "w")
    output.write(filledIn)
    output.close()
    os.chmod(runswiftWirelessFinalise, 0o777)
    # Sync it to the robot.
    subprocess.check_call(templateParameters["copy"] % (runswiftWirelessFinalise, robotName),
                          shell=True)
    print('Completed copy of updated runswiftwireless service to {}'
          .format(robotName))
    print('You might wish to restart wifi (4 chest button presses)')
