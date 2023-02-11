#!/usr/bin/env python3

# Script to run multiple simulations and record results?

# Default values

# file to read positions from for team 18
# file to read positions from for team 19/opp
# number of times to run positions
# how much to vary the psoitions by
# where to write results to

import subprocess
import numpy as np
import os
import time


# TODO get teams from app.py eventually

# num_trials
team = 18
opp_team = 19
# num_robots = 2
# num_opp_robots = 1
# sd_for_error = 10

mul_sim_log = []

def main(num_trials, num_robots, num_opp_robots, sd_for_error):
    # get positions for team 18
    mul_sim_log = []
    our_positions = []

    with open("pose.txt", "r") as pose_file:
        our_positions = [tuple(x.split(' ')) for x in pose_file.read().strip().split('\n')]
    
    # get positions for opp
    opp_positions = []
    with open("opp_pose.txt", "r") as opp_pose_file:
        opp_positions = [tuple(x.split(' ')) for x in opp_pose_file.read().strip().split('\n')]

    print (our_positions, opp_positions)

    if opp_positions[0][0] == '':
        print('no opposition, quitting')
        return

    # start running sims
    for trial in range(num_trials):        
        # set randomness and write to files
        
        # remove file if it exists somehow
        if os.path.exists("pose_tmp.txt"):
            os.remove("pose_tmp.txt")

        # write positions to temp file
        with open("pose_tmp.txt", "w+") as pose_file:
            for pose in our_positions:
                x = int(np.random.normal(int(pose[0]), sd_for_error, 1)[0])
                y = int(np.random.normal(int(pose[1]), sd_for_error, 1)[0])
                print(x, y)
                pose_file.write(str(x) + " " + str(y) + " " + pose[2])
                pose_file.write('\n')

        # remove file if it exists somehow
        if os.path.exists("opp_pose_tmp.txt"):
            os.remove("opp_pose_tmp.txt")

        # write positions to temp file
        with open("opp_pose_tmp.txt", "w+") as pose_file:
            for pose in opp_positions:
                x = int(np.random.normal(int(pose[0]), sd_for_error, 1)[0])
                y = int(np.random.normal(int(pose[1]), sd_for_error, 1)[0])
                print(x, y)
                pose_file.write(str(x) + " " + str(y) + " " + pose[2])
                pose_file.write('\n')

        # rcssserver3d_command = "export SPARK_FAST_TIME=true; pkill -9 rcssserver3d; pkill -9 'RoboViz.jar'; rcssserver3d & roboviz.sh &" 
        rcssserver3d_command = "pkill -9 rcssserver3d; pkill -9 'RoboViz.jar'; rcssserver3d & roboviz.sh &"
        # rcssserver3d_command = "export SPARK_FAST_TIME=true; pkill -9 rcssserver3d; rcssserver3d &" 
        sim_command = "sim_kill; sim_run -t " + str(team) + " -n " + str(num_robots) + " -ra '--gamecontroller.state PLAYING --simulation.multiple_sims true' -f pose_tmp.txt; sim_run -t " + str(opp_team) + " -n " + str(num_opp_robots) + " -ra '--gamecontroller.state PLAYING --simulation.multiple_sims true' -f opp_pose_tmp.txt;"

        os.system(rcssserver3d_command)
        time.sleep(5) # allow roboviz to open
        
        ret = subprocess.run(sim_command, stderr=subprocess.PIPE, shell=True)

        # read mul_sim_log
        with open("mul_sim_18_1.log", "r") as log:
            mul_sim_log.append(log.read())

        os.system("sim_stop")
    return (mul_sim_log)
