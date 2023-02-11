from Task import BehaviourTask


class GameController(BehaviourTask):
    def init(self):
        msg = '\n'.join((
            '',
            '# Congratulations runswift is running.',
            '# ',
            '# You might like to try a specific skill, for example: ',
            '#     runswift -s WalkInACircle',
        ))
        print(msg)

    def transition(self):
        pass

    def _tick(self):
        pass
