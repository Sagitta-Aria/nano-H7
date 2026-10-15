"""Preserve physical servo channel IDs and adapt only the host angle scale.

The user's physical check establishes S1=wire1, S2=wire2; HELLO pin labels must
not select another motor. Use in host UI/initialization, never as PWM calibration.
"""
import math

SERVO_LAYOUT = {
    1: {"pin": "PH6", "timer": 1, "travel": 180., "home": "120", "wire_travel": 270.},
    2: {"pin": "PH9", "timer": 2, "travel": 270., "home": "0", "wire_travel": 180.},
}


def wire_angle(fields, channel, degrees):
    """Convert a physical target to the SAME wire channel's angle coordinate.

    Retain the 500..2500 us envelope and round to firmware half-degree steps;
    no command is sent. Raw protocol commands bypass this host-only conversion.
    """
    layout = SERVO_LAYOUT[channel]
    value = float(degrees)
    if not math.isfinite(value) or not 0 <= value <= layout["travel"]:
        raise ValueError(f"S{channel} angle exceeds its physical travel")
    travel = int(fields.get(f"servo{channel}_travel", int(layout["wire_travel"] * 10))) / 10.
    if travel <= 0:
        raise ValueError(f"S{channel} firmware travel is invalid")
    rounded = math.floor(value * travel / layout["travel"] * 2 + .5) / 2.
    return f"{rounded:g}"


def display_angle(channel, pulse_us):
    """Report the physical angle scale from the returned pulse, not pin labels.

    This represents the commanded pulse only; it is not a position measurement.
    """
    return f"{(float(pulse_us) - 500) * SERVO_LAYOUT[channel]['travel'] / 2000:.1f}"


def initial_commands(fields):
    """Return connection-only targets using physical channel identity.

    Call after device identity checks, once per serial connection, never on a
    browser reload or status poll. All scales are checked before any motion.
    """
    count = int(fields.get("servo", 0))
    commands = []
    for channel, layout in SERVO_LAYOUT.items():
        if channel > count:
            continue
        commands.append(f"SERVO {channel} {wire_angle(fields, channel, layout['home'])}")
    return commands
