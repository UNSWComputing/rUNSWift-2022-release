from BehaviourTask import BehaviourTask
from util.actioncommand import head
from util.Sensors import angles
from robot import Joints
from math import radians
from util.MathUtil import stdev
from collections import deque


class HeadFixedYawAndPitch(BehaviourTask):

    """
    Description:
    A headskill associated with looking at a specified neck yaw and neck pitch.
    """

    RECORD_LENGTH = 10
    YAW_STDEV = radians(0.1)
    PITCH_STDEV = radians(0.1)

    def _reset(self):
        self._yaw = 0
        self._pitch = 0
        self._curr_yaw = 0
        self._curr_pitch = 0
        self._yaw_record = deque()
        self._pitch_record = deque()

    def _tick(self, yaw=0, pitch=0, yaw_speed=0.75, pitch_speed=0.25):

        # If the target yaw or pitch has changed, reset everything!
        if not (similar(yaw, self._yaw) and similar(pitch, self._pitch)):
            self._reset()

        # Read in new yaw and pitch aims
        self._yaw = yaw
        self._pitch = pitch

        # update curr_yaw, curr_pitch from sensors
        currAngles = angles(self.world.blackboard)
        self._curr_pitch = currAngles[Joints.HeadPitch]
        self._curr_yaw = currAngles[Joints.HeadYaw]

        # update yaw and pitch records
        self._yaw_record.append(self._curr_yaw)
        self._pitch_record.append(self._curr_pitch)
        if len(self._yaw_record) > self.RECORD_LENGTH:
            self._yaw_record.popleft()
        if len(self._pitch_record) > self.RECORD_LENGTH:
            self._pitch_record.popleft()

        self.world.b_request.actions.head = head(yaw, pitch, False, yaw_speed, pitch_speed)

    # Whether the head has arrived at the aim yaw and pitch
    def arrived(self):
        return similar(self._curr_yaw, self._yaw) and similar(self._curr_pitch, self._pitch)

    # Whether the head cant move anymore towards the specified
    # yaw and pitch, due to neck joint angle limitations
    def cant_move_more(self):
        return (
            len(self._yaw_record) == self.RECORD_LENGTH
            and len(self._pitch_record) == self.RECORD_LENGTH
            and stdev(self._yaw_record) < self.YAW_STDEV
            and stdev(self._pitch_record) < self.PITCH_STDEV
        )


# Whether two angles are similar to each other
def similar(a, b):
    return abs(a - b) < radians(4)
