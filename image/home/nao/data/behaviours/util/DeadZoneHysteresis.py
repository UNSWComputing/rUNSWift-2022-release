class DeadZoneHysteresis(object):
    """
    Description:
    A hysteresis with a deadzone that prevents flickering
    decisions in continuous numbers.

    lower: lower boundary condition
    upper: upper boundary condition
    default: whether to start with upper. set to True if starting with upper.

    (https://xtronics.com/wiki/Dead-band_hysteresis.html)
    """

    def __init__(self, lower, upper, default_above=False):
        self._lower = lower
        self._upper = upper
        self._above = default_above

    # Function to evaluate hysteresis with new value
    # Returns True is hysteresis is above, otherwise False
    def evaluate(self, value):
        if self._above:
            if value < self._lower:
                self._above = False

        else:
            if value > self._upper:
                self._above = True

        return self._above


# Some example code to show how it works
if __name__ == "__main__":
    hyst = DeadZoneHysteresis(10, 20)
    print(hyst.evaluate(5))  # False
    print(hyst.evaluate(-2))  # False
    print(hyst.evaluate(17))  # False
    print(hyst.evaluate(22))  # True
    print(hyst.evaluate(13))  # True
    print(hyst.evaluate(3))  # False
