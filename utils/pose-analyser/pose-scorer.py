#!/usr/bin/env python3
import sys
from analyse import readXYs
import math


def main():

    # 1st arg --> gt
    # 2nd arg --> localisation output file
    if(len(sys.argv) != 3):
        print("usage: {} 'ground truth file' 'robot output file'".format(sys.argv[0]))

    gt_x, gt_y, _, gt_time = readXYs(sys.argv[1])
    x, y, _, time = readXYs(sys.argv[2])

    gt_x, gt_y, robot_x, robot_y = matchTimeStamps(gt_x, gt_y, gt_time,
                                                   x, y, time)

    calculateRMSE(gt_x, gt_y, robot_x, robot_y)


def matchTimeStamps(gt_x, gt_y, gt_time, robot_x, robot_y, robot_time):

    # situation where gt is started before robot
    while(gt_time[0] < robot_time[0]):
        del gt_time[0]
        del gt_x[0]
        del gt_y[0]

    for i in range(len(gt_x)):
        # if reached end of robot localisation before end of ground truth
        if i >= (len(robot_x) - 1):
            # delete all ground truth & localisation after this point
            robot_x = robot_x[:i]
            robot_y = robot_y[:i]
            robot_time = robot_time[:i]

            gt_x = gt_x[:i]
            gt_y = gt_y[:i]
            gt_time = gt_time[:i]
            break

        j = i
        while (robot_time[j+1] < gt_time[i]):
            del robot_x[j]
            del robot_y[j]
            del robot_time[j]

    # if reached end of localisation after end of ground truth
    # delete everyhting after ground truth ended if len localisation > len gt
    if(len(robot_x) > len(gt_x)):
        robot_x = robot_x[:(len(gt_time))]
        robot_y = robot_y[:(len(gt_time))]
        robot_time = robot_time[:(len(gt_time))]

    return (gt_x, gt_y, robot_x, robot_y)


# calculates root mean squared error
def calculateRMSE(gt_x, gt_y, robot_x, robot_y):

    x_diff = []
    y_diff = []
    num_points = len(gt_x)
    for i in range(num_points):
        x_diff.append(gt_x[i] - robot_x[i])
        y_diff.append(gt_y[i] - robot_y[i])

    dist_err_squared = []
    for i in range(num_points):
        dist = math.hypot(x_diff[i], y_diff[i])
        dist_err_squared.append((dist * dist))

    # calculate mean
    final_err = 0
    for err in dist_err_squared:
        final_err = final_err + err
    final_err = math.sqrt(final_err/num_points)

    return final_err


if __name__ == "__main__":
    main()
