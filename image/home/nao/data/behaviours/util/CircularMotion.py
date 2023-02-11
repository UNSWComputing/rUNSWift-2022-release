from math import pi, tan, sqrt
from util.MathUtil import normalisedTheta

"""
Description:
Functions to help calculate robot's circular motion around objects.
This can be used for walking in curves, walking around obstacles,
strafing around the ball, etc. etc.
"""


# Returns circumference of circle in mm
def circumference(radius_mm):
    return 2 * pi * radius_mm


# Returns period of circular motion in s
def period(radius_mm, tangent_vel_mm_s):
    return circumference(radius_mm) / tangent_vel_mm_s


# Returns angular velocity of circular motion in rad/s
def angular_velocity(radius_mm, tangent_vel_mm_s):
    return 2 * pi / period(radius_mm, tangent_vel_mm_s)


# Returns left to achieve circular motion in  mm/s
def left(radius_mm, tangent_vel_mm_s):
    return tangent_vel_mm_s * tan(angular_velocity(radius_mm, tangent_vel_mm_s))


# Returns the tangent heading to a circle, given the robot position /
# centre position in global coordinates, and clockwise / anticlockwise
# direction the robot is going to travel around the centre
def tangent_heading(robot_pos, centre_pos, clockwise=True):
    vec_centre_to_robot = robot_pos.minus(centre_pos)
    return normalisedTheta(vec_centre_to_robot.heading() + pi / 2 * (-1 if clockwise else 1))


# Returns the time to circle around, given
# 1. radius of the circle (mm)
# 2. angle between lines, from centre to start pos, and centre to final pos (rad)  # noqa
# 3. speed of walking (mm/s)
def time_to_circle(radius_mm, angle_between_rad, speed_mm_s):
    return period(radius_mm, speed_mm_s) * (abs(angle_between_rad) / (2 * pi))


# Distance to walk on from a point to another point on concentric circles
#   FROM theta1 on a circle of r1 (r1t1)
#   TO theta2 on a circle of r2 (r2t2)
# while increas/decreasing the RADIUS AT A CONSTANT RATE
def distance_from_r1t1_to_r2t2(r1_mm, r2_mm, theta1_rad, theta2_rad):
    # Integrating circumference over theta and radius, we obtain
    # the following equation
    circumference = (r1_mm * theta2_rad + r2_mm * theta2_rad - r1_mm * theta1_rad - r2_mm * theta1_rad) / 2

    d = sqrt((r2_mm - r1_mm) * (r2_mm - r1_mm) + (circumference * circumference))
    return d


# Wrapper for calculating distance_from_r1t1_to_r2t2,
#   FROM circle of r1 (mm)
#   TO circle of r2 (mm)
#   WITH change in theta (rad)
def distance_from_r1_to_r2_delta_theta(r1_mm, r2_mm, delta_theta_rad):
    theta1_rad = 0
    theta2_rad = delta_theta_rad
    return distance_from_r1t1_to_r2t2(r1_mm, r2_mm, theta1_rad, theta2_rad)
