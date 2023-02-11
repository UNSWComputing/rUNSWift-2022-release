from BehaviourTask import BehaviourTask
from head.HeadFixedYawAndPitch import HeadFixedYawAndPitch


class HeadCentre(BehaviourTask):

    YAW = 0
    PITCH = 0

    def _initialise_sub_tasks(self):
        self._sub_tasks = {"HeadFixedYawAndPitch": HeadFixedYawAndPitch(self)}

    def _reset(self):
        self._current_sub_task = "HeadFixedYawAndPitch"

    def _tick(self):
        self._tick_sub_task(self.YAW, self.PITCH)
