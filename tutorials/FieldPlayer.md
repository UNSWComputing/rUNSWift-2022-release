```python
CENTER_DIVE_THRES = 200
DANGEROUS_BALL_THRES = 300
DIVE_VEL_THRES = 50
```

Constants that determine the goalie behaviour.
[Elaborate...]

```python
FREE_KICK_TARGET = ENEMY_GOAL_BEHIND_CENTER.add(Vector2D(0, 200))
```

The location to aim during free-kicks I think ?

### The FieldPlayer Class

This is the top-most class for an outfield player. Of the underscore-prefixed methods shown below, the first 3 are used for controlling the state-machine while the last one `_tick()` is called by the C++ `runswift` executible. 

```python
class FieldPlayer(BehaviourTask):
    def _initialise_sub_tasks(self):

    def _reset(self):

    def _transition(self):

    def _tick(self):

    ... # other methods

```
