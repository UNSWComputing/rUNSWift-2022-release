# flake8: noqa
# A script used to symmetrically copy the joint values on the right side of the body to the left
# side of the body for a single pos joint line, that starts with "!" and ends with the duration
# Example: python copy_right_side_to_left_side.py '! -7    29    115   -12   -58   -3    -102  -67   21    -87   94    39    15    -31   -89   101   35    -2    114   19    -5    1     83    0     0     1000'

import sys

NUM_JOINTS = 25

# enum for joints
(
    HY,
    HP,
    LSP,
    LSR,
    LEY,
    LER,
    LWY,
    LHYP,
    LHR,
    LHP,
    LKP,
    LAP,
    LAR,
    RHR,
    RHP,
    RKP,
    RAP,
    RAR,
    RSP,
    RSR,
    REY,
    RER,
    RWY,
    LH,
    RH,
) = range(25)

# Dictionary for corresponding symmetric joints
JOINT_CORRESPONDANCE = {
    LSP: RSP,
    LSR: RSR,
    LEY: REY,
    LER: RER,
    LWY: RWY,
    LHR: RHR,
    LHP: RHP,
    LKP: RKP,
    LAP: RAP,
    LAR: RAR,
}

# List of joints where the joint value has to be inverted
INVERT_JOINT_LIST = [LSR, LER, LEY, LWY, LHR, LAR]  # arm joints  # leg joints


if len(sys.argv) < 2:
    print("please provide a pos line!")
    exit()

if len(sys.argv) > 2:
    print("please put quotes around the argument")
    exit()

line = sys.argv[1]
line = line.split()
line.pop(0)  # remove the "!"
duration = line.pop(len(line) - 1)  # remove the duration, and store it to use later
line = [int(item) for item in line]  # convert all joint values to integers

# Copy right-side joint values to left-side
for key in JOINT_CORRESPONDANCE:
    line[key] = line[JOINT_CORRESPONDANCE[key]]

# Invert necessary joint values to achieve symmetry
for joint in INVERT_JOINT_LIST:
    line[joint] = -line[joint]

# Construct the pos line again
out = "! "
for num in line:
    out += str(num).ljust(6)
out += duration

# Print the output pos line!
print(out)
