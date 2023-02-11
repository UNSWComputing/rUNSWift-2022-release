import time


class OAP(object):
    """
    A simple helper class which allows code to be executed at a slower frequency (useful for print outs)

    tick() returns true if the specified period has elapsed since tick was last called.

    Example:
    ```
    oap = OAP(0.5)
    while True:
        if oap.tick():
            print("This line should print every 0.5 seconds i.e. 2 fps")
    ```
    """

    def __init__(self, period):
        self._curr = time.time()
        self._period = period

    def tick(self):
        elapsed = time.time() - self._curr
        if elapsed > self._period:
            self._curr = time.time()
            return True
        return False
