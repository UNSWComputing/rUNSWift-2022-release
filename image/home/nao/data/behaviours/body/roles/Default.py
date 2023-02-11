from BehaviourTask import BehaviourTask
from test.WalkAround import WalkAround


class Default(BehaviourTask):
    def _initialise_sub_tasks(self):
        self._sub_tasks = {"WalkAround": WalkAround(self)}

    def _reset(self):
        self._current_sub_task = "WalkAround"
