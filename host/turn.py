"""Shared Web/Tk turn command validation. Angles are degrees at the UI boundary;
the MCU owns yaw feedback, wheel control and completion. Never auto-arm here.
"""
from decimal import Decimal, InvalidOperation

# P is transmitted in hundredths as a signed 32-bit protocol integer.
# This is a wire-format ceiling, not an operator tuning recommendation.
KP_WIRE_MAX = 2147483647
# Demo in-place speed floor; mirror firmware TURN_MIN_SPS, not straight P.
TURN_MIN_SPS = 200


def gain_command(kp):
    """Encode nonnegative P without the former tuning cap; zero disables yaw.

    Keep exact hundredths and protocol representability, never clamp silently.
    This formats a setting and does not move or arm the board.
    """
    if isinstance(kp, bool):
        raise ValueError("P 必须是非负数，最多两位小数。")
    try:
        value = Decimal(str(kp))
    except InvalidOperation:
        raise ValueError("P 必须是非负数，最多两位小数。") from None
    if not value.is_finite() or value < 0:
        raise ValueError("P 必须是非负数，最多两位小数。")
    if value > Decimal(KP_WIRE_MAX) / 100:
        raise ValueError("P 过大，超出协议整数容量（P×100 不得超过 2147483647）。")
    if value*100 != (value*100).to_integral_value():
        raise ValueError("P 必须是非负数，最多两位小数。")
    return 'TURN CONFIG {}'.format(int(value*100))


def tuning_command(kp, tolerance, min_sps=None):
    """Encode shared yaw gain and completion tolerance, never round or move."""
    gain = gain_command(kp)
    if isinstance(tolerance, bool):
        raise ValueError('角度容差范围 0.01～10°，最多两位小数。')
    try:
        value = Decimal(str(tolerance))
    except InvalidOperation:
        raise ValueError('角度容差范围 0.01～10°，最多两位小数。') from None
    if not value.is_finite() or not Decimal('0.01') <= value <= 10 or value*100 != (value*100).to_integral_value():
        raise ValueError('角度容差范围 0.01～10°，最多两位小数。')
    body = '{} {}'.format(gain, int(value*100))
    if min_sps is not None:
        if type(min_sps) is not int or not 1 <= min_sps <= 5000:
            raise ValueError('最低转向轮速必须是 1～5000 的整数 step/s。')
        body += ' {}'.format(min_sps)
    return body


def turn_command(mode, angle, sps, accel):
    """Build one bounded TURN MOVE body; reject rounding and invalid input.

    REL uses heading at acceptance; ABS uses an explicit MCU turn zero.
    This only formats a command and must not be used as a completion signal.
    """
    if mode not in ("REL", "ABS") or isinstance(angle, bool):
        raise ValueError("请选择相对转动或转到零点角度。")
    try:
        degrees = Decimal(str(angle))
    except InvalidOperation:
        raise ValueError("角度范围 -180～180°，最多两位小数。") from None
    if not degrees.is_finite() or abs(degrees) > 180 or degrees * 100 != (degrees * 100).to_integral_value():
        raise ValueError("角度范围 -180～180°，最多两位小数。")
    if type(sps) is not int or not 1 <= sps <= 5000 or type(accel) is not int or not 1 <= accel <= 20000:
        raise ValueError("最高轮速范围 1～5000 step/s，加速度范围 1～20000 step/s²。")
    return f"TURN MOVE {mode} {int(degrees * 100)} {sps} {accel}"
