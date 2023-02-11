# flake8: noqa
"""LED Overrides.

Allows a behaviour to request an override of the default eye behaviours during testing.
To use this, simple import this module, and then call the override function with a key and a color. Like so

LedOverride.override(LedOverride.rightEye, LEDColors.red)
# Note that LEDColors is an enum in Constants.
"""

import robot

rightEye = "rEye"
leftEye = "lEye"
rightFoot = "rFoot"
leftFoot = "lFoot"

_overrides = {}


def reset_led_override():
    global _overrides
    _overrides = {}


def override_request(request):
    """Override Request.

    Overrides parts of a request with the singleton in this module.
    @param request A behaviour request
    """
    global _overrides
    if rightEye in _overrides:
        request.actions.leds.rightEye = _overrides[rightEye]
    if leftEye in _overrides:
        request.actions.leds.leftEye = _overrides[leftEye]
    if rightFoot in _overrides:
        request.actions.leds.rightFoot = _overrides[rightFoot]
    if leftFoot in _overrides:
        request.actions.leds.leftFoot = _overrides[leftFoot]

    # TODO(Ritwik): Add more leds.


def override(key, value):
    global _overrides
    _overrides[key] = value
