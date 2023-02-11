# This is a sample behaviour to illustrate how to write your own
# By default, this behaviour will be run, but you can target any
# behaviour by using the -s flag on the command line

from BehaviourTask import BehaviourTask
from util.Timer import Timer
from body.skills.Walk import Walk


class WalkAround(BehaviourTask):
    def _initialise_sub_tasks(self):
        self._sub_tasks = {"Walk": Walk(self)}

    def _reset(self):
        self.timer = Timer(3000000).start()
        self._current_sub_task = "Walk"

    def _tick(self):
        if self.timer.finished():
            self._tick_sub_task(turn=1)
        else:
            self._tick_sub_task(forward=200)
