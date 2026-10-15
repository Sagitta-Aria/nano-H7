"""Finite gyro-assisted translation protocol, with no automatic motion retry.

Distance is mean translation wheel pulses. Heading is captured by the MCU;
this command does not set a yaw zero or claim measured Cartesian position.
"""


def straight_command(direction, steps, sps, accel):
    """Validate one translation command; rotation and jogging use other APIs."""
    if direction not in ("FWD", "BACK", "LEFT", "RIGHT"):
        raise ValueError("陀螺仪辅助直线只支持前进、后退、左移和右移。")
    for value, name, minimum, maximum in (
        (steps, "步数", 1, 10000000), (sps, "速度", 1, 5000), (accel, "加速度", 1, 20000)):
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError("{}范围 {}～{}，必须为整数。".format(name, minimum, maximum))
    return "CHASSIS STRAIGHT {} {} {} {}".format(direction, steps, sps, accel)
