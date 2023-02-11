class BehaviourTask(object):

    """
    Definitions of a BehaviourTask.
    "Leaf" - A node with no children. Calls an actioncommand.
    "Internal node" - A node with at least one child. Calls other tasks.
    """

    def __init__(self, parent=None, world=None):
        """__init__

        Override of constructor.

        If a parent class exists, copy world from parent. Otherwise, world
        should be provided as an argument. (Root BehaviourTask case)

        Initialise:
        - self._current_sub_task
        - self._sub_tasks
        - Other attributes as required (in _reset())

        DO NOT Override this.
        """
        if parent is not None:
            self.parent = parent
            self.world = self.parent.world
        else:
            self.world = world

        self._current_sub_task = None
        self._initialise_sub_tasks()
        self._reset()

    def _initialise_sub_tasks(self):
        """_initialise_sub_tasks

        Private function that initialises a map of sub tasks.

        Overriding:
        - Leaf: Don't override
        - Internal Node: Do override
        """
        self._sub_tasks = None

    def _reset(self):
        """_reset

        Private function to initialise and reset class attributes.

        Overriding:
        - Leaf: Optional
        - Internal Node: Optional
        """
        pass

    def _transition(self):
        """_transition.

        Private function for switching between sub tasks.

        Overriding:
        - Leaf: Don't override
        - Internal Node: Do override
        """
        pass

    def _tick(self, *args, **kwargs):
        """_tick.

        Private function to process information, calculate parameters to pass
        to sub tasks.

        Overriding:
        - Leaf: Do override
        - Internal Node: Optional. If overriden, ensure self._tick_sub_task()
                         is called.
        """
        if self._current_sub_task is not None:
            self._tick_sub_task()

    def reset(self):
        """reset.

        Public function to call _reset() and reset() of current sub task.

        DO NOT Override this.
        """

        self._reset()

        if self._current_sub_task is not None:
            self._sub_tasks[self._current_sub_task].reset()

    def tick(self, *args, **kwargs):
        """tick.

        Public function to:
        - Leaf: call _tick()
        - Internal Node: Call _transition(), reset tasks if necessary,
                         then calls _tick()

        DO NOT Override this.
        """

        self.world.behaviour_hierarchy = self.world.behaviour_hierarchy + self.__class__.__name__ + "."

        if self._current_sub_task is not None:
            previous_sub_task = self._current_sub_task
            self._transition()

            if self._current_sub_task != previous_sub_task:
                self._sub_tasks[self._current_sub_task].reset()

        self._tick(*args, **kwargs)

    def _tick_sub_task(self, *args, **kwargs):
        """_tick_sub_task

        Private function that ticks the current sub task with provided
        arguments.

        DO NOT Override this.
        """
        self._sub_tasks[self._current_sub_task].tick(*args, **kwargs)
