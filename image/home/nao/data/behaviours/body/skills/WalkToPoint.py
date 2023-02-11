from BehaviourTask import BehaviourTask
from body.skills.Walk import Walk
from math import radians
from util.Global import myPos, myHeading
from util.MathUtil import normalisedTheta
from util.Vector2D import Vector2D
from util.ObstacleAvoidance import walk_vec_with_avoidance


class WalkToPoint(BehaviourTask):

    """
    Description:
    A skill associated with walking to a global point on the field.
    Hading error is the angle between the robot's heading and the vector to
    the target.
    If heading error is large, robot will first purely turn, to face
    the final_pos. If heading error is moderate, robot walks forwards,
    adjusting the turn depending on the error, correcting over one second.
    If heading error is small, robot will walk purely forwards, to reach
    a walk speed as fast as possible, and as stable as possible.
    """

    CLOSE_DISTANCE = 50  # mm
    NOT_CLOSE_DISTANCE = 150  # mm

    WALK_SPEED = 300  # mm / s
    TURN_RATE = 1.5  # rad / s

    HEADING_ERROR_TO_ONLY_TURN = radians(40)  # rad
    HEADING_ERROR_TO_ADJUST = radians(15)  # rad

    TIME_TO_FIX_HEADING = 1.0  # how much time we allow to align heading (s)

    def _initialise_sub_tasks(self):
        self._sub_tasks = {"Walk": Walk(self)}

    def _reset(self):
        self._current_sub_task = "Walk"
        self._heading_close = False
        self._position_close = False

    def _tick(self, final_pos=Vector2D(0, 0), use_avoidance=True, speed=1.0):
        self._my_pos_to_final_pos = final_pos.minus(myPos())

        if not self._position_close and self._pos_error() < self.CLOSE_DISTANCE:
            self._position_close = True
        elif self._position_close and self._pos_error() > self.NOT_CLOSE_DISTANCE:
            self._position_close = False

        if self._position_close:
            forward = 0
            left = 0
            turn = 0
        elif abs(self._heading_error()) > self.HEADING_ERROR_TO_ONLY_TURN:
            # If heading is very off, just turn, walking forwards with a big
            # turn is very unstable
            forward = 0
            left = 0
            turn = self.TURN_RATE if self._heading_error() > 0 else -self.TURN_RATE
        else:
            walk_vector = Vector2D(self.WALK_SPEED, 0)

            # Slow down if we're close to the final position
            # (to prevent overshooting)
            if self._pos_error() < 200:
                walk_vector.scale(0.5)

            if use_avoidance:
                walk_vector = walk_vec_with_avoidance(walk_vector)

            forward = walk_vector.x
            left = walk_vector.y

            if abs(self._heading_error()) > self.HEADING_ERROR_TO_ADJUST:
                # Aim to correct the heading error over an amount of time
                turn = self._heading_error() / self.TIME_TO_FIX_HEADING
            else:
                # If the error is small, just keep walking straight,
                # constantly changing turn causes instability
                turn = 0

        self._tick_sub_task(forward, left, turn, speed=speed)

    def _heading_error(self):
        return normalisedTheta(self._my_pos_to_final_pos.heading() - myHeading())

    def _pos_error(self):
        return self._my_pos_to_final_pos.length()
