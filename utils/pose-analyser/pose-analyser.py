#!/usr/bin/env python3
import sys
import os
import matplotlib.pyplot as plt
import argparse

X_MAX = 4500
Y_MAX = 3000
FBO = 750  # field boundary offset


def main():
    args = parseArgs()
    x_lists, y_lists, theta_lists, time_lists = readXYFiles(args.files)
    if args.time_plot:
        plotTime(x_lists, y_lists, theta_lists, time_lists, args.files)
    else:
        plotField(x_lists, y_lists, args.files)


def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", help="filepaths of files to plot",
                        nargs='+')
    # optional argument
    parser.add_argument("-t", "--time_plot",
                        help="plot X, Y, and heading against time",
                        action="store_true")

    return parser.parse_args()


# plots x, y and heading vs time
def plotTime(x_lists, y_lists, theta_lists, time_lists, filenames):

    fig, (ax_x, ax_y, ax_theta) = plt.subplots(3, 1)
    max_time = max([max(j) for j in time_lists])
    min_time = min([min(j) for j in time_lists])

    ax_x.set_xlim(min_time, max_time)
    ax_x.set_ylim(-X_MAX-FBO, X_MAX+FBO)
    ax_x.set_title('X')

    for i in range(len(x_lists)):
        ax_x.plot(time_lists[i], x_lists[i], label=filenames[i])

    ax_y.set_xlim(min_time, max_time)
    ax_y.set_ylim(-Y_MAX-FBO, Y_MAX+FBO)
    ax_y.set_title('Y')

    for i in range(len(y_lists)):
        ax_y.plot(time_lists[i], y_lists[i], label=filenames[i])

    ax_theta.set_xlim(min_time, max_time)
    max_theta = max([max(j) for j in theta_lists])
    ax_theta.set_ylim(0, max_theta)
    ax_theta.set_title('Heading')

    for i in range(len(theta_lists)):
        ax_theta.plot(time_lists[i], theta_lists[i], label=filenames[i])

    fig.legend(loc='upper right')
    plt.show()


# plots X, Y coordinates on field image
def plotField(x_lists, y_lists, filenames):

    fig, ax = plt.subplots()
    img = plt.imread(os.path.dirname(os.path.realpath(__file__)) + "/field.png")
    ax.imshow(img, extent=[-X_MAX-FBO, X_MAX+FBO, -Y_MAX-FBO, Y_MAX+FBO])

    for i in range(len(x_lists)):
        plt.plot(x_lists[i], y_lists[i], label=filenames[i])

    plt.legend(loc='upper right')
    plt.show()


# reads x, y, theta and time from list of filepaths
def readXYFiles(filepaths):

    x_lists, y_lists, theta_lists, time_lists = ([] for i in range(4))

    for filepath in filepaths:

        x_list, y_list, theta_list, time_list = readXYs(filepath)

        x_lists.append(x_list)
        y_lists.append(y_list)
        theta_lists.append(theta_list)
        time_lists.append(time_list)

    return x_lists, y_lists, theta_lists, time_lists


# extracts x, y, theta and time from file
def readXYs(filepath):

    if not os.path.isfile(filepath):
        print("File path {} does not exist. Exiting...".format(filepath))
        sys.exit()

    x_list, y_list, theta_list, time_list = ([] for i in range(4))

    f = open(filepath, 'r')

    for line in f:
        x, y, theta, time = line.split()
        x_list.append(float(x))
        y_list.append(float(y))
        theta_list.append(float(theta))
        time_list.append(float(time))

    f.close()

    return x_list, y_list, theta_list, time_list


if __name__ == "__main__":
    main()
