from BehaviourTask import BehaviourTask
from util.actioncommand import walk
from util.ObstacleAvoidance import sonar_left_obstacle, sonar_right_obstacle


class Walk(BehaviourTask):

    """
    Description:
    Skill associated with directly creating the walk actioncommand.
    SPEED should be increased for serious games.
    """

    # Range between 0.0 and 1.0. Use 0 in lab and 1 in serious games
    SPEED = 0.6

    def _tick(self, forward=0, left=0, turn=0, speed=1.0, allow_shuffle=True, cap_speed=True):

        if cap_speed:
            speed = min(self.SPEED, speed)

        useShuffle = sonar_left_obstacle() or sonar_right_obstacle() if allow_shuffle else False

        self.world.b_request.actions.body = walk(
            forward=forward,
            left=left,
            turn=turn,
            speed=speed,
            useShuffle=useShuffle,
            leftArmBehind=False,
            rightArmBehind=False,
        )
