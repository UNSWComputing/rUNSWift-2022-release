import time
import sys
from TerminalGetch import getch

try:
    import inputs
except:
    print("To use joysticks install inputs by:")
    print("pip install inputs")
    sys.exit(1)


class Controller:
    def __init__(self, maxVelX, maxVelY, maxYaw):
        self._velForwards = 0
        self._velLeft = 0
        self._turn = 0
        self._maxVelForwards = maxVelX
        self._maxVelLeft = maxVelY
        self._maxTurn = maxYaw
        self._exitSet = False
        self._kick = False

    def __str__(self):
        return "kick={kick} forwardsVelocity={forwards} leftVelocity={left} turnVelocity={turn}\n".format(kick=self._kick, forwards=self._velForwards, left=self._velLeft, turn=self._turn)

    def stopMsg(self):
        return "kick={kick} forwardsVelocity={forwards} leftVelocity={left} turnVelocity={turn}\n".format(kick=False, forwards=0, left=0, turn=0)


    def clamp(self):
        if abs(self._velForwards) > abs(self._maxVelForwards):
            self._velForwards = self._maxVelForwards if self._velForwards > 0 else -self._maxVelForwards

        if abs(self._velLeft) > abs(self._maxVelLeft):
            self._velLeft = self._maxVelLeft if self._velLeft > 0 else -self._maxVelLeft

        if abs(self._turn) > abs(self._maxTurn):
            self._turn = self._maxTurn if self._turn > 0 else -self._maxTurn

    def displayUsageMsg(self):
        pass
    def update(self):
        pass
    def exit(self):
        return self._exitSet

class KeyboardController(Controller):
    def __init__(self, stepSize, turnStepSize, maxVelX, maxVelY, maxYaw):
        self._stepSize = stepSize
        self._turnStepSize = turnStepSize
        Controller.__init__(self, maxVelX, maxVelY, maxYaw)

    def update(self):
        self._kick = False

        char = getch()
        if (char == 'w'):
            self._velForwards += self._stepSize
        if (char == 'a'):
            self._velLeft -= self._stepSize
        if (char == 'd'):
            self._velLeft += self._stepSize
        if (char == 's'):
            self._velForwards -= self._stepSize
        if (char == ' '):
            self._velForwards = self._velLeft = self._turn = 0
        if (char == 'f'):
            self._turn -= self._turnStepSize
        if (char == 'h'):
            self._turn += self._turnStepSize

        if (char == 'q'):
            self._exitSet = True
        if (char == 'k'):
            self._kick = True

        self.clamp()

    def displayUsageMsg(self):
        print("Usage: q to quit w,a,s,d to change speed, space to stop, f and h to rotate, k to kick")

class GamePadController(Controller):
    def __init__(self, maxVelX, maxVelY, maxYaw, deadzoneRatio):
        Controller.__init__(self, maxVelX, maxVelY, maxYaw)
        self._deadzoneRatio = deadzoneRatio

    def update(self):
        events = inputs.get_gamepad()
        for event in events:
            val = event.state
            if event.code == 'ABS_X':
                val = self.applyDeadZone(val, 0, 255)
                self._velLeft = -mapRanges(val, 0, 255, -self._maxVelLeft, self._maxVelLeft)
            if event.code == 'ABS_Y':
                val = self.applyDeadZone(val, 0, 255)
                self._velForwards = mapRanges(val, 0, 255, self._maxVelForwards, -self._maxVelForwards)
            if event.code == 'ABS_RX':
                val = self.applyDeadZone(val, 0, 255)
                self._turn = -mapRanges(val, 0, 255, -self._maxTurn, self._maxTurn)

    def displayUsageMsg(self):
        print("Left joystick moves around, right to rotate")

    def applyDeadZone(self, val, min, max):
        midpoint = (min + max)/2.0
        range = abs(max - min)
        if (abs(val - midpoint) < (self._deadzoneRatio * range)):
            val = midpoint
        elif (abs(val - midpoint) > 0):
            val += (self._deadzoneRatio * range)
        else:
            val -= (self._deadzoneRatio * range)
        return val

def mapRanges(value, inputMin, inputMax, outMin, outMax):
    inputRange = inputMax - inputMin
    outRange = outMax - outMin
    unitScale = float(value - inputMin) / float(inputRange)
    return outMin + (unitScale * outRange)
