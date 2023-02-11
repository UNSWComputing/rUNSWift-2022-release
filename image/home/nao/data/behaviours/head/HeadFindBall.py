from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch
from util.Timer import Timer, WallTimer
from math import radians


class HeadFindBall(BehaviourTask):

    """
    Description:
    A Headskill associated with the ball_player looking for the ball
    A narrow scan is performed first, then a wider scan is performed
    """

    SEQUENCE_NARROW = [
        radians(40),
        radians(-40),
    ]

    SEQUENCE_WIDE = [
        radians(-60),
        radians(-30),
        radians(30),
        radians(60),
        radians(0),
    ]

    STARE_SECONDS = 0.4
    PITCH = radians(19)
    NARROW_PERIOD = 2.0  # seconds

    def _initialise_sub_tasks(self):
        self._sub_tasks = {
            "FixedYawAndPitch": HeadFixedYawAndPitch(self),
        }

    def _reset(self):
        self.PITCH = radians(19 + self.world.blackboard.kinematics.parameters.cameraPitchBottom)
        self._current_sub_task = "FixedYawAndPitch"
        self._timer = Timer(self.STARE_SECONDS * 1000000)  # convert to micro-seconds  # noqa
        self._yaw_aim = 0
        self._sequence_counter = 0
        self._currently_moving = False
        self._timer_since_start = WallTimer()
        self._sequence = self.SEQUENCE_NARROW

    def _tick(self):
        self._sequence = self._choose_sequence()

        # only restart timer when we've reached the position
        if self._currently_moving and (
            self._sub_tasks[self._current_sub_task].arrived()
            or self._sub_tasks[self._current_sub_task].cant_move_more()
        ):
            self._timer.restart()
            self._currently_moving = False

        if not self._currently_moving and self._timer.finished():
            self._increment_sequence_counter()
            self._yaw_aim = self._sequence[self._sequence_counter]
            self._currently_moving = True

        self._tick_sub_task(yaw=self._yaw_aim, pitch=self.PITCH, yaw_speed=0.8, pitch_speed=1.0)

    def _choose_sequence(self):
        if self._timer_since_start.elapsedSeconds() < self.NARROW_PERIOD:
            return self.SEQUENCE_NARROW
        else:
            return self.SEQUENCE_WIDE

    def _increment_sequence_counter(self):
        self._sequence_counter += 1
        if self._sequence_counter >= len(self._sequence):
            self._sequence_counter = 0
