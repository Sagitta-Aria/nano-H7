"""Explicit demo board, using the real wire protocol. It is not hardware proof.

Synthetic resource values exercise GUI rendering; motion is a coarse time-domain
model and must not be used to validate timer jitter or physical braking distance.
"""
from collections import OrderedDict
import math
import time
from .protocol import encode, decode, Framer
from .navigation_sim import NavigationSim
from .turn_sim import TurnSim
from .straight_sim import StraightSim
from .optical_flow import FlowDemo


class SimulatedBoard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.start = self.last = clock()
        self.heartbeat = self.start - 2
        self.armed = False
        self.lift_initialized = False
        self.lift_down_sign = 0
        self.aux_ranges = [[0, 8000], [0, 0]]
        self.rotation_referenced = self.rotation_configured = False
        self.led = 0
        self.rx = self.bad = self.replay_hits = self.timeouts = 0
        self.motion_fault = self.motion_fault_total = self.uncertain_axes = 0
        self.motion_reason = 'none'
        self.cache = OrderedDict()
        self.chassis_state = "idle"
        self.chassis_direction = "FWD"
        self.chassis_id = self.chassis_total = 0
        self.invert = 5  # Match the verified M1/M3 firmware default.
        self.jog_time = self.start - 2
        # Demo servo tables mirror S1's 270 deg and S2's 180 deg half-degree steps,
        # one entry per channel so the demo cannot hide a per-channel bug.
        self.servo_travel_steps = [540, 360]
        self.servo_steps = [180, 180]
        self.axes = [dict(pos=0, velocity=0., target=0, remaining=0, active=0,
                          move_id=0, finite=False, accel=2000, fraction=0.) for _ in range(7)]
        self.navigation = NavigationSim(self)
        self.turn = TurnSim(self)
        self.straight = StraightSim(self)
        self.flow = FlowDemo(self)

    def update(self):
        now = self.clock()
        if self.chassis_state == "jogging" and now - self.jog_time >= .4:
            self.chassis_halt()
        elapsed = now - self.last
        if self.armed and now - self.heartbeat >= 1:
            self.stop()
            self.timeouts += 1
        # Substeps keep the demo responsive when an update interval is irregular.
        dt = min(elapsed, 1.)
        was_paused = self.turn.paused
        if self.navigation.auto_imu and dt > 0:
            self.navigation.feedback(self.navigation.absolute_heading, True, 1, now)
        paused = self.turn.waiting(now)
        chassis_dt = 0. if was_paused or paused else dt
        for index, axis in enumerate(self.axes):
            if index < 4 and paused: continue
            if index < 4 and (self.navigation.busy() or self.turn.busy() or self.straight.busy()):
                continue
            if not axis["active"]:
                continue
            target = axis["target"]
            if axis["finite"]:
                brake = math.sqrt(2 * axis["accel"] * axis["remaining"])
                target = math.copysign(min(abs(target), max(1., brake)), target)
            axis_dt = chassis_dt if index < 4 else dt
            delta = max(-axis["accel"] * axis_dt, min(axis["accel"] * axis_dt, target - axis["velocity"]))
            axis["velocity"] += delta
            axis["fraction"] += abs(axis["velocity"]) * axis_dt
            steps = int(axis["fraction"])
            axis["fraction"] -= steps
            if axis["finite"]:
                steps = min(steps, axis["remaining"])
                axis["remaining"] -= steps
            axis["pos"] += steps * (1 if axis["velocity"] >= 0 else -1)
            if (axis["finite"] and not axis["remaining"]) or (target == 0 and axis["velocity"] == 0):
                axis["active"] = 0
                axis["velocity"] = axis["target"] = 0
        self.last = now
        self.navigation.update(now, chassis_dt)
        self.turn.update(now, chassis_dt)
        self.straight.update(now, chassis_dt)
        if self.chassis_state == "moving" and not any(a["active"] for a in self.axes[:4]):
            self.chassis_state = "done" if not any(a["remaining"] for a in self.axes[:4]) else "stopped"

    def stop(self):
        self.straight.stop()
        self.turn.stop()
        self.navigation.stop()
        self.armed = False
        self.chassis_state = "stopped"
        for axis in self.axes:
            axis.update(active=0, velocity=0., target=0)

    def chassis_halt(self):
        """Demo equivalent of stopping M1..M4 without affecting auxiliary axes."""
        self.straight.stop()
        self.chassis_state = "stopped"
        if self.navigation.state != "suspended":
            self.navigation.stop()
        self.turn.stop()
        for axis in self.axes[:4]:
            axis.update(active=0, velocity=0., target=0)

    def inject_motion_fault(self, reason='step_deadline'):
        """SIM-only ISR fault injection: stop outputs and revoke moving references."""
        self.motion_fault += 1; self.motion_fault_total += 1; self.motion_reason = reason
        self.uncertain_axes |= sum(1 << i for i,a in enumerate(self.axes) if a['active'])
        if self.uncertain_axes & 16:self.lift_initialized = False
        if self.uncertain_axes & 32:self.rotation_referenced = False
        self.stop()

    def motion_report(self):
        """History survives explicit idle/disarmed acknowledgement."""
        active=sum(1 << i for i,a in enumerate(self.axes) if a['active'])
        return (f'OK cmd=MOTION_STAT v=1 latched={int(bool(self.motion_fault))} '
                f'total={self.motion_fault_total} reason={self.motion_reason} '
                f'uncertain={self.uncertain_axes} active={active} armed={int(self.armed)}')

    def chassis_dispatch(self, sequence, args):
        matrix = {"FWD": (1, 1, 1, 1), "BACK": (-1, -1, -1, -1),
                  "LEFT": (1, -1, -1, 1), "RIGHT": (-1, 1, 1, -1),
                  "CCW": (-1, 1, -1, 1), "CW": (1, -1, 1, -1)}
        if args == ["HALT"]:
            self.chassis_halt()
            return "OK cmd=CHASSIS_HALT"
        if (self.navigation.busy() or self.turn.busy()) and args != ["STAT"]:
            return "ERR code=CHASSIS_BUSY"
        if len(args) == 2 and args[0] == "CONFIG":
            mask = self.number(args[1], 0, 15)
            if self.navigation.state == 'suspended' or any(a["active"] for a in self.axes[:4]):
                return "ERR code=CHASSIS_BUSY"
            self.invert = mask
            self.navigation.invalidate()
            args = ["STAT"]
        if args == ["STAT"]:
            remaining = max((a["remaining"] for a in self.axes[:4] if a["move_id"] == self.chassis_id), default=0)
            active = sum(1 << i for i, a in enumerate(self.axes[:4]) if a["active"])
            return (f"OK cmd=CHASSIS_STAT state={self.chassis_state} direction={self.chassis_direction} "
                    f"move_id={self.chassis_id} total={self.chassis_total} remaining={remaining} active={active} invert={self.invert} "
                    f"assist={int(self.straight.assisted)} target_cd={round(self.straight.target*100)} "
                    f"error_cd={round(self.straight.error*100)} fault={self.straight.fault} paused={int(self.turn.paused)}")
        if not args or (args[0] in ("MOVE", "STRAIGHT") and len(args) != 5) or (args[0] == "JOG" and len(args) != 4):
            return "ERR code=BAD_ARGUMENT"
        if args[0] not in ("MOVE", "JOG", "STRAIGHT") or args[1] not in matrix:
            return "ERR code=BAD_ARGUMENT"
        assisted = args[0] == "STRAIGHT"
        if assisted and (args[1] in ("CW", "CCW") or not self.turn.fresh(self.clock())):
            return "ERR code=STRAIGHT_NOT_READY_DIRECTION_BUSY_OR_IMU"
        finite = args[0] in ("MOVE", "STRAIGHT")
        steps = self.number(args[2], 1, 10000000) if finite else 0
        speed = self.number(args[-2], 1, 5000)
        accel = self.number(args[-1], 1, 20000)
        if not self.armed or self.chassis_state == "moving" or (finite and self.chassis_state == "jogging"):
            return "ERR code=DISARMED_BUSY_OR_FAULT"
        if self.chassis_state != "jogging" and any(a["active"] for a in self.axes[:4]):
            return "ERR code=DISARMED_BUSY_OR_FAULT"
        targets = [s * (-1 if self.invert & (1 << i) else 1) * speed for i, s in enumerate(matrix[args[1]])]
        renew = not finite and self.chassis_state == "jogging" and all(a["target"] == t and a["accel"] == accel for a, t in zip(self.axes, targets))
        if not renew:
            for axis, target in zip(self.axes, targets):
                axis.update(target=target, velocity=0., remaining=steps, active=1, finite=finite,
                            accel=accel, move_id=sequence, fraction=0.)
            self.chassis_id = sequence
        self.chassis_direction = args[1]
        if assisted:
            signs = [s * (-1 if self.invert & (1 << i) else 1) for i, s in enumerate(matrix[args[1]])]
            yaw = [s * (-1 if self.invert & (1 << i) else 1) for i, s in enumerate(matrix['CW'])]
            self.straight.start(signs, yaw, steps, speed, accel, sequence)
        else:
            self.straight.assisted, self.straight.fault = False, 'none'
        self.navigation.invalidate()
        self.chassis_state = "moving" if finite else "jogging"
        self.chassis_total = steps
        self.jog_time = self.clock()
        return f"OK cmd=CHASSIS_{args[0]} state=accepted move_id={self.chassis_id}"

    @staticmethod
    def number(text, minimum, maximum):
        body = text[1:] if text.startswith("-") else text
        if not text or not body.isascii() or not body.isdigit():
            raise ValueError("bad integer")
        value = int(text)
        if not minimum <= value <= maximum:
            raise ValueError("out of range")
        return value

    def receive(self, frame):
        self.update()
        self.rx += len(frame)
        try:
            request = decode(frame)
        except ValueError:
            self.bad += 1
            return b""
        if request.sequence in self.cache:
            original, response = self.cache[request.sequence]
            if original != request.body:
                return encode(request.sequence, "ERR code=SEQUENCE_REUSE")
            self.replay_hits += 1
            return response
        try:
            body = ("ERR code=COMMAND_TOO_LONG" if len(request.body) >= 128
                    else self.dispatch(request.sequence, request.body))
        except (ValueError, IndexError):
            body = "ERR code=BAD_ARGUMENT"
        response = encode(request.sequence, body)
        self.cache[request.sequence] = (request.body, response)
        if len(self.cache) > 8:
            self.cache.popitem(last=False)
        return response

    def dispatch(self, sequence, body):
        words = body.split()
        cmd, args = words[0], words[1:]
        if cmd == "HELLO" and not args:
            return "OK cmd=HELLO fw=0.2.29 protocol=1 board=SIMULATOR led_mask=3 red=fixed chassis=1 servo=2 servo_travel=2700 servo1_travel=2700 servo2_travel=1800 serial=dual lift=1 lift_max=8000 nav=1 nav_heading=1 nav_grid=1 nav_path=1 turn=1 aux_control=1 flow=1 servo1_pin=PE4 servo2_pin=PH9 turn_tuning=1 aux_live=1 turn_precision=1 straight=1 turn_band=1 imu_pause=1"
        if cmd == "NAV":
            return self.navigation.dispatch(sequence, args)
        if cmd == "FLOW":
            return self.flow.dispatch(args)
        if cmd == "TURN":
            return self.turn.dispatch(sequence, args)
        if cmd == "LIFT":
            return self.lift_dispatch(sequence, args)
        if cmd == "CHASSIS":
            return self.chassis_dispatch(sequence, args)
        if cmd == "PING" and not args:
            self.heartbeat = self.clock()
            return "OK cmd=PING"
        if cmd == "INFO" and not args:
            return "OK cmd=INFO cpu_hz=400000000 apb_hz=100000000 timer_hz=200000000 flash_kb=2048 uid=SIMULATED reset=0x00000000 rtos=11.2.0 baud=115200 chassis_max=5000 aux_max=3000 nav_reference=1"
        if cmd == "STAT" and not args:
            return (f"OK cmd=STAT uptime_ms={int((self.clock()-self.start)*1000)} heap_free=37000 heap_min=37000 tasks=4 "
                    f"stack_comm=4096 stack_motion=1700 stack_super=1700 led={self.led} armed={int(self.armed)} "
                    f"rx={self.rx} rx_err=0 tx_err=0 bad={self.bad} replay={self.replay_hits} timeout={self.timeouts} motion_fault={self.motion_fault}")
        if cmd == 'MOTION' and args == ['STAT']:
            return self.motion_report()
        if cmd == 'MOTION' and args == ['CLEAR']:
            if self.armed or any(a['active'] for a in self.axes):
                return 'ERR code=MOTION_CLEAR_REQUIRES_DISARMED_IDLE'
            self.motion_fault = 0
            self.chassis_halt(); self.navigation.invalidate(); self.navigation.committed = False
            self.navigation.state='stopped';self.navigation.fault='none'
            self.turn.state='stopped';self.turn.fault='none';self.straight.fault='none'
            return self.motion_report()
        if cmd == "LED" and len(args) == 2:
            green, blue = [self.number(a, 0, 1) for a in args]
            self.led = green | blue << 1
            return f"OK cmd=LED led={self.led}"
        if cmd == "ARM" and len(args) == 1:
            enabled = self.number(args[0], 0, 1)
            if not enabled:
                self.stop()
            elif self.clock() - self.heartbeat >= 1:
                return "ERR code=HEARTBEAT_REQUIRED"
            elif self.motion_fault:
                return 'ERR code=MOTION_FAULT_LATCHED'
            elif any(a["active"] for a in self.axes):
                return "ERR code=BUSY_OR_FAULT"
            else:
                self.armed = True
            return f"OK cmd=ARM armed={int(self.armed)}"
        if cmd == "STOP" and not args:
            self.stop()
            return "OK cmd=STOP armed=0"
        if cmd == "SERVO":
            return self.servo_dispatch(args)
        if cmd == "AUX":
            return self.aux_dispatch(sequence, args)
        if cmd == "MOVE" and len(args) == 4:
            index = self.number(args[0], 1, 7) - 1
            if index < 4 and (self.chassis_state in ("moving", "jogging") or self.navigation.busy() or self.turn.busy()):
                return "ERR code=CHASSIS_BUSY"
            steps = self.number(args[1], -10000000, 10000000)
            sps = self.number(args[2], 1, 5000 if index < 4 else 3000)
            accel = self.number(args[3], 1, 20000)
            axis = self.axes[index]
            if index == 5 and self.uncertain_axes & 32:
                return 'ERR code=AUX_NOT_REFERENCED'
            if index == 4:
                error = self.lift_error(steps)
                if error:
                    return "ERR code=" + error
            if index == 5 and self.rotation_configured and not (
                    self.aux_ranges[1][0] <= axis["pos"] + steps <= self.aux_ranges[1][1]):
                return "ERR code=DISARMED_BUSY_OR_RANGE"
            if not self.armed or axis["active"] or not steps:
                return "ERR code=DISARMED_BUSY_OR_RANGE"
            axis.update(target=sps if steps > 0 else -sps, remaining=abs(steps), active=1,
                        finite=True, accel=accel, move_id=sequence, fraction=0., velocity=0.)
            if index < 4:
                self.navigation.invalidate()
            return f"OK cmd=MOVE state=accepted axis={index+1} move_id={sequence}"
        if cmd == "WHEELS" and len(args) == 5:
            if self.chassis_state in ("moving", "jogging") or self.navigation.busy() or self.turn.busy():
                return "ERR code=CHASSIS_BUSY"
            speeds = [self.number(a, -5000, 5000) for a in args[:4]]
            accel = self.number(args[4], 1, 20000)
            if not self.armed:
                return "ERR code=DISARMED_BUSY_OR_REVERSE"
            for axis, speed in zip(self.axes, speeds):
                if axis["active"] and (axis["finite"] or speed * axis["velocity"] < 0):
                    return "ERR code=DISARMED_BUSY_OR_REVERSE"
            for axis, speed in zip(self.axes, speeds):
                axis.update(target=speed, finite=False, accel=accel, move_id=sequence,
                            active=int(speed != 0 or axis["velocity"] != 0))
            self.navigation.invalidate()
            return "OK cmd=WHEELS state=accepted"
        if cmd == "AXIS" and len(args) == 1:
            index = self.number(args[0], 1, 7) - 1
            axis = self.axes[index]
            return (f"OK cmd=AXIS axis={index+1} pos={axis['pos']} sps={int(axis['velocity'])} "
                    f"remaining={axis['remaining']} active={axis['active']} move_id={axis['move_id']}")
        return "ERR code=UNKNOWN_OR_BAD_ARGUMENT"

    def lift_error(self, steps):
        """Mirror firmware target limits for raw M5 MOVE as well as lift buttons."""
        if not self.lift_initialized:
            return "M5_NOT_REFERENCED"
        if self.lift_down_sign not in (-1, 1):
            return "M5_DIRECTION_UNKNOWN"
        target = (self.axes[4]["pos"] + steps) * self.lift_down_sign
        if not self.aux_ranges[0][0] <= target <= self.aux_ranges[0][1]:
            return "M5_SOFT_LIMIT"
        return None

    def lift_report(self):
        """Report simulated depth; no physical homing or sensor is implied."""
        axis = self.axes[4]
        depth = axis["pos"] * self.lift_down_sign
        minimum, maximum = self.aux_ranges[0]
        return (f"OK cmd=LIFT_STAT axis=5 initialized={int(self.lift_initialized)} "
                f"down_sign={self.lift_down_sign} pos={depth} max={maximum} "
                f"up_room={max(0, depth-minimum)} down_room={max(0, maximum-depth)} active={axis['active']} min={minimum}")

    def aux_report(self, axis):
        """Mirror board-owned logical positions and explicit M6 setup flags."""
        a = self.axes[axis-1]
        position = a["pos"] * (self.lift_down_sign if axis == 5 else 1)
        minimum, maximum = self.aux_ranges[axis-5]
        referenced = self.lift_initialized if axis == 5 else self.rotation_referenced
        configured = True if axis == 5 else self.rotation_configured
        return (f"OK cmd=AUX_STAT axis={axis} pos={position} min={minimum} max={maximum} "
                f"referenced={int(referenced)} configured={int(configured)} active={a['active']}")

    def aux_dispatch(self, sequence, args):
        """Demo counterpart of atomic AUX commands, including raw-MOVE bounds."""
        if len(args) < 2:
            return "ERR code=BAD_ARGUMENT"
        verb, axis = args[0], self.number(args[1], 5, 6)
        a = self.axes[axis-1]
        sign = self.lift_down_sign if axis == 5 else 1
        position = a["pos"] * sign
        referenced = self.lift_initialized if axis == 5 else self.rotation_referenced
        if verb == "STAT" and len(args) == 2:
            return self.aux_report(axis)
        if verb == "HALT" and len(args) == 2:
            a.update(active=0, velocity=0., target=0)
            return self.aux_report(axis)
        if verb == 'REFERENCE' and len(args) == 3:
            position=self.number(args[2],-1000000,1000000)
            low,high=self.aux_ranges[axis-5]
            if (self.armed or self.motion_fault or a['active'] or not self.uncertain_axes & (1 << (axis-1)) or
                    sign not in (-1,1) or not low <= position <= high):
                return 'ERR code=AUX_REFERENCE_REQUIRES_LOST_DISARMED_IDLE_IN_RANGE'
            a['pos']=position*sign;self.uncertain_axes &= ~(1 << (axis-1))
            if axis == 5:self.lift_initialized=True
            else:self.rotation_referenced=True
            return self.aux_report(axis)
        if verb == "ZERO" and len(args) == 2:
            low, high = self.aux_ranges[axis-5]
            if (self.armed or self.motion_fault or a['active'] or not low <= 0 <= high or
                    (axis == 5 and not self.uncertain_axes & 16)):
                return "ERR code=AUX_ZERO_REQUIRES_DISARMED_IDLE"
            a["pos"] = 0
            self.uncertain_axes &= ~(1 << (axis-1))
            if axis == 5:self.lift_initialized = True
            else:self.rotation_referenced = True
            return self.aux_report(axis)
        if verb == "CONFIG" and len(args) == 4:
            low, high = [self.number(v, -1000000, 1000000) for v in args[2:]]
            if (a["active"] or not referenced or low > high or
                    (axis == 5 and (low < 0 or low == high)) or not low <= position <= high):
                return "ERR code=AUX_CONFIG_RANGE_REFERENCE_OR_BUSY"
            self.aux_ranges[axis-5] = [low, high]
            if axis == 6:
                self.rotation_configured = True
            return self.aux_report(axis)
        if verb in ("GOTO", "STEP") and len(args) == 5:
            value = self.number(args[2], -2000000, 2000000)
            sps, accel = self.number(args[3], 1, 3000), self.number(args[4], 1, 20000)
            target = value if verb == "GOTO" else position + value
            low, high = self.aux_ranges[axis-5]
            if (not self.armed or a["active"] or not referenced or
                    (axis == 5 and sign not in (-1, 1)) or
                    (axis == 6 and not self.rotation_configured) or not low <= target <= high):
                return "ERR code=AUX_NOT_READY_BUSY_OR_LIMIT"
            if target != position:
                result = self.dispatch(sequence, f"MOVE {axis} {(target-position)*sign} {sps} {accel}")
                if result.startswith("ERR"):
                    return result
            return f"OK cmd=AUX_MOVE axis={axis} state=accepted move_id={sequence}"
        return "ERR code=BAD_ARGUMENT"

    def lift_dispatch(self, sequence, args):
        """First INIT is top; repeated INIT preserves both position and direction."""
        axis = self.axes[4]
        if args == ["INIT"]:
            if not self.lift_initialized and not (self.uncertain_axes & 16):
                if axis["active"]:
                    return "ERR code=M5_BUSY"
                axis["pos"] = 0
                self.lift_initialized = True
            return self.lift_report()
        if args == ["STAT"]:
            return self.lift_report()
        if len(args) == 2 and args[0] == "DIR":
            sign = self.number(args[1], -1, 1)
            if sign == 0:
                return "ERR code=BAD_ARGUMENT"
            if not self.lift_initialized or self.armed or axis["active"] or axis["pos"] != 0:
                return "ERR code=M5_DIR_REQUIRES_TOP_DISARMED"
            self.lift_down_sign = sign
            return self.lift_report()
        if len(args) == 5 and args[0] == "MOVE" and args[1] in ("UP", "DOWN"):
            steps = self.number(args[2], 1, 10000000)
            sign = self.lift_down_sign * (1 if args[1] == "DOWN" else -1)
            return self.dispatch(sequence, f"MOVE 5 {steps * sign} {args[3]} {args[4]}")
        return "ERR code=BAD_ARGUMENT"

    @staticmethod
    def angle_text(steps):
        """Degrees to one decimal, matching the firmware reply format."""
        return f"{steps // 2}.{(steps % 2) * 5}"

    def servo_reply(self, verb, channel):
        steps = self.servo_steps[channel]
        limit = self.servo_travel_steps[channel]
        pulse = 500 + (2000 * steps + limit // 2) // limit
        return (f"OK cmd={verb} ch={channel + 1} angle={self.angle_text(steps)} "
                f"us={pulse} on=1 pin={'PE4' if channel == 0 else 'PH9'} timer={'TIM15_CH1N' if channel == 0 else 'TIM12_CH2'}")

    def servo_dispatch(self, args):
        """Demo mirror of servo.c; the angle table is arithmetic here, data there."""
        if len(args) == 1 and args[0] in ("STAT", "?"):
            # The firmware answers one frame per channel in a single body.
            return " ".join(self.servo_reply("SERVO_STAT", ch)
                            for ch in range(len(self.servo_steps)))
        if len(args) != 2:
            return "ERR code=SERVO_BAD_ARGUMENT"
        channel = self.number(args[0], 1, len(self.servo_steps)) - 1
        text = args[1]
        if text.count(".") > 1 or not text.replace(".", "").isdigit():
            return "ERR code=SERVO_BAD_ARGUMENT"
        whole, _, frac = text.partition(".")
        if len(whole) < 1 or len(whole) > 3 or len(frac) > 1:
            return "ERR code=SERVO_BAD_ARGUMENT"
        tenths = int(whole) * 10 + (int(frac) if frac else 0)
        if tenths > self.servo_travel_steps[channel] * 5:
            return "ERR code=ANGLE_RANGE"
        self.servo_steps[channel] = tenths // 5
        return self.servo_reply("SERVO", channel)


class SimulatedSerial:
    """Small pyserial-compatible transport; no physical port is opened.

    Like pyserial, any use after close() raises instead of quietly succeeding,
    so callers that must react to a vanished board are actually exercised.
    """
    def __init__(self):
        self.board = SimulatedBoard()
        self.framer = Framer()
        self.buffer = bytearray()
        self.is_open = True

    def _require_open(self):
        if not self.is_open:
            raise OSError("simulated port is closed")

    @property
    def in_waiting(self):
        """Expose available bytes with the same semantics as pyserial."""
        self._require_open()
        return len(self.buffer)

    def write(self, data):
        self._require_open()
        for frame in self.framer.feed(data):
            self.buffer.extend(self.board.receive(frame))
        return len(data)

    def read(self, size=256):
        self._require_open()
        self.board.update()
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        if not result:
            time.sleep(.005)
        return result

    def close(self):
        self.board.stop()
        self.is_open = False
