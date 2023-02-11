from BehaviourTask import BehaviourTask


class Game(BehaviourTask):

    """
    Description:
    A skill to deal with a game environment. This task should be specific
    to a game of soccer, using a GameController.
    """

    def _reset(self):
        msg = "\n".join(
            (
                "",
                "# Congratulations runswift is running.",
                "# Remember to make the robot stiff (or limp) with ",
                '# two chest button presses, the robot should say "body stiff". ',
                "# ",
                "# You might like to try a specific skill, for example: ",
                "#     runswift -s WalkAround",
            )
        )
        print(msg)
