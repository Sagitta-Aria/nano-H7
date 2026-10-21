"""Route UI S1 to PH9/180 degrees and keep UI S2 on PE4/270 degrees.

UI IDs are physical controls, not firmware channel IDs. Resolve the wire route
before sending commands or interpreting telemetry; never use this as calibration.
"""
import math

SERVO_LAYOUT = {
    1: {"pin": "PH9", "timer": "TIM12_CH2", "travel": 180., "home": "120", "wire_travel": 180., "wire": 2},
    2: {"pin": "PE4", "timer": "TIM15_CH1N", "travel": 270., "home": "0", "wire_travel": 270., "wire": 1},
}


def wire_channel(fields, channel):
    """Map a UI control to its GPIO's firmware ID, including legacy PH9 firmware.

    Use for commands and telemetry, never to change physical angle limits.
    Missing metadata uses the current firmware's PE4=1, PH9=2 assignment.
    PH9 was the 270-degree output before PE4 existed; distinguish those layouts.
    """
    has_pe4 = any(fields.get(f'servo{wire}_pin') == 'PE4' for wire in (1, 2))
    if has_pe4:
        pins = ('PH9', 'PH6') if channel == 1 else ('PE4',)
    else:
        pins = ('PH6',) if channel == 1 else ('PH9',)
    for wire in (1, 2):
        if fields.get(f'servo{wire}_pin') in pins:
            return wire
    return SERVO_LAYOUT[channel]['wire']


def display_channel(fields, wire):
    """Map a firmware reply back to its UI control; unknown channels return None."""
    return next((ch for ch in SERVO_LAYOUT if wire_channel(fields, ch) == wire), None)


def wire_angle(fields, channel, degrees):
    """Convert a physical target using the resolved firmware channel's scale.

    Retain the 500..2500 us envelope and round to firmware half-degree steps;
    no command is sent. Raw protocol commands bypass this host-only conversion.
    """
    layout = SERVO_LAYOUT[channel]
    value = float(degrees)
    if not math.isfinite(value) or not 0 <= value <= layout["travel"]:
        raise ValueError(f"S{channel} angle exceeds its physical travel")
    wire = wire_channel(fields, channel)
    travel = int(fields.get(f"servo{wire}_travel", int(layout["wire_travel"] * 10))) / 10.
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
        wire = wire_channel(fields, channel)
        if wire > count:
            continue
        commands.append(f"SERVO {wire} {wire_angle(fields, channel, layout['home'])}")
    return commands
