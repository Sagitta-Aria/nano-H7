"""Programmatic control API for the H743 board: what the vision code calls.

host/agent.py is the transport-level client (request/reply, reconnects). This
module is the layer above it -- the one a mission script or a vision node uses
-- and it exists to make two mistakes impossible:

1. **Never re-send a motion command.** Motion bodies are not idempotent, so an
   automatic retry after a link glitch could duplicate a physical movement.
   Every motion call here goes through _motion(), which refuses retries at all.
2. **Never leave the board armed by accident.** The firmware stops the axes on
   its own 1000 ms after the last PING, but between "vision died" and "lease
   expired" the wheels keep turning. held_velocity() therefore re-sends the
   target on every pass and stops the board if a pass fails, so a stalled caller
   ends in a stop rather than in a runaway.

Values returned by status() are ints, not the strings the wire carries.

    board = Control("SIM")
    board.arm()
    board.hold_velocity((0, 0, 200, -200), seconds=2.0)   # spin in place 2 s
    board.move_axis(5, steps=400, sps=800)
    board.disarm()                                        # also stops
"""
import time

from .agent import Agent, AgentError, LinkLost, is_idempotent

# Motion commands: sent at most once, never retried, never replayed.
MOTION_PREFIXES = ("MOVE", "WHEELS", "ARM", "STOP", "CHASSIS")

# Fields that arrive as integers and are more useful as integers.
_INT_FIELDS = (
    "uptime_ms", "heap_free", "heap_min", "tasks", "stack_comm", "stack_motion",
    "stack_super", "led", "armed", "rx", "rx_err", "tx_err", "bad", "replay",
    "timeout", "motion_fault", "pos", "sps", "remaining", "active", "move_id",
    "axis", "cpu_hz", "apb_hz", "timer_hz", "flash_kb", "baud", "chassis_max",
    "aux_max", "invert", "total", "chassis", "straight", "assist", "target_cd", "error_cd",
)


def _typed(fields):
    """Copy a field dict, converting the numeric fields to int.

    Keys starting with "_" are metadata added by the agent (reply body, ok
    flag, sequence). They must survive: dropping them would hide whether the
    board accepted the command.
    """
    result = {}
    for key, value in fields.items():
        if key.startswith("_"):
            result[key] = value
            continue
        if key in _INT_FIELDS:
            try:
                result[key] = int(value)
                continue
            except (TypeError, ValueError):
                pass
        result[key] = value
    return result


class ControlError(AgentError):
    """The board refused a command, or answered something unusable."""


class Control:
    def __init__(self, port, agent=None):
        self.agent = agent if agent is not None else Agent(port)
        self.port = port

    # ---- lifecycle ----------------------------------------------------

    def connect(self):
        self.agent.connect()
        return self

    def close(self):
        self.agent.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- low level ----------------------------------------------------

    def _annotate(self, reply):
        """Typed fields plus the metadata a caller needs to judge the answer."""
        result = _typed(reply.fields)
        result["_ok"] = "1" if reply.ok else "0"
        result["_body"] = reply.body
        result["_sequence"] = reply.sequence
        return result

    def _query(self, body, attempts=2):
        """Idempotent command: safe to retry across a reconnect."""
        if not is_idempotent(body):
            raise ControlError("refusing to retry non-idempotent {!r}".format(body))
        reply = self.agent.request_resilient(body, attempts=attempts)
        if reply is None:
            raise ControlError("no reply to {!r}".format(body))
        return self._annotate(reply)

    def _motion(self, body):
        """Motion command: exactly one attempt, no reconnect-and-resend."""
        try:
            reply = self.agent.request(body)
        except LinkLost as exc:
            # Deliberately not retried. The caller must re-read status and
            # decide; guessing here could repeat a movement physically.
            raise ControlError(
                "motion {!r} was not confirmed: {}".format(body, exc))
        if reply is None:
            raise ControlError("no reply to {!r}".format(body))
        if not reply.ok:
            raise ControlError("{} rejected: {}".format(body, reply.body))
        return self._annotate(reply)

    # ---- identity and status ------------------------------------------

    def hello(self):
        reply = self.agent.hello
        return {} if reply is None else _typed(reply.fields)

    def require_protocol(self, version="1"):
        fields = self.hello()
        if fields.get("protocol") != version:
            raise ControlError("board speaks protocol {}, need {}".format(
                fields.get("protocol"), version))
        return fields

    def status(self):
        return self._query("STAT")

    def axis(self, index):
        return self._query("AXIS {}".format(int(index)))

    def chassis_status(self):
        return self._query("CHASSIS STAT")

    def straight(self, direction, steps, sps=500, accel=2000):
        """Finite gyro-assisted translation; returns acceptance, not arrival.

        Requires firmware straight=1 and fresh yaw. Never arms automatically.
        Completion/fault must be read from chassis_status() for this move_id.
        """
        from .straight import straight_command
        return self._motion(straight_command(direction, steps, sps, accel))

    # ---- motion --------------------------------------------------------

    def arm(self):
        """Requires a live lease; the firmware rejects ARM without one."""
        return self._motion("ARM 1")

    def stop(self):
        return self._motion("STOP")

    def disarm(self):
        return self._motion("ARM 0")

    def leds(self, green, blue=0):
        return self._query("LED {} {}".format(1 if green else 0, 1 if blue else 0))

    def wheels(self, speeds, accel=2000):
        """Atomic four-wheel target in step/s. Order: M1 M2 M3 M4."""
        if len(speeds) != 4:
            raise ControlError("wheels() needs exactly four speeds")
        body = "WHEELS {} {} {} {} {}".format(
            int(speeds[0]), int(speeds[1]), int(speeds[2]), int(speeds[3]), int(accel))
        return self._motion(body)

    def move_axis(self, index, steps, sps, accel=2000):
        """Finite single-axis move. Returns on acceptance, not completion."""
        return self._motion("MOVE {} {} {} {}".format(
            int(index), int(steps), int(sps), int(accel)))

    def wait_axis(self, index, move_id=None, timeout=30.0, interval=.1):
        """Block until a finite move finishes. Returns the final axis status.

        Uses active/remaining rather than timing, because the board reports the
        truth and a guessed duration would silently disagree under load.
        """
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.axis(index)
            if move_id is not None and last.get("move_id") != move_id:
                time.sleep(interval)
                continue
            if not last.get("active") and not last.get("remaining"):
                return last
            time.sleep(interval)
        raise ControlError(
            "axis {} did not finish within {:.1f}s: {}".format(index, timeout, last))

    # ---- the safe continuous-motion primitive --------------------------

    def hold_velocity(self, speeds, seconds, accel=2000, hz=10.0, on_stall="stop"):
        """Run four wheel speeds for `seconds`, then stop. Blocks.

        Every pass re-sends the target. The firmware needs that: its lease is
        1000 ms and a wheel target is not renewed by PING alone. If a pass fails
        the board is stopped before the error propagates, so a broken link or a
        frozen caller cannot leave the wheels running on the last command.

        hz must stay above 1/lease; 10 Hz gives ten renewals per lease window.
        """
        if hz <= 1.0:
            raise ControlError("hz must exceed the 1 Hz lease floor, got {}".format(hz))
        interval = 1.0 / hz
        deadline = time.monotonic() + seconds
        passes = 0
        try:
            while time.monotonic() < deadline:
                self.wheels(speeds, accel=accel)
                passes += 1
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(interval, remaining))
        finally:
            # Runs on success, on ControlError and on Ctrl-C alike. The firmware
            # lease is the backstop if even this stop cannot be delivered.
            if on_stall == "stop":
                try:
                    self.wheels((0, 0, 0, 0), accel=accel)
                except AgentError:
                    pass
        return {"seconds": seconds, "renewals": passes, "hz": hz,
                "speeds": tuple(int(s) for s in speeds)}
