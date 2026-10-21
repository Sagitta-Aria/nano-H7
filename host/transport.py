"""One serial worker owns all I/O. Tk only receives immutable queue events."""
import itertools
import heapq
import queue
import secrets
import threading
import time
from .protocol import encode, decode, Framer
from .simulator import SimulatedSerial
from .servo_config import initial_commands as servo_initial_commands


class EventQueue(queue.Queue):
    """Wake the web control pump when a worker publishes a complete response."""
    def __init__(self, wakeup):
        super().__init__()
        self.wakeup = wakeup

    def put(self, item, block=True, timeout=None):
        super().put(item, block, timeout)
        self.wakeup.set()


def list_ports():
    try:
        import serial  # Honor an unavailable parent dependency even if submodules were cached.
        from serial.tools.list_ports import comports
        from .imu import is_candidate
        return [(p.device, p.description) for p in comports() if not is_candidate(p)]
    except ImportError:
        return []


class Connection(threading.Thread):
    def __init__(self, port, events, expected_board=None, expected_uid=None, imu_provider=None):
        super().__init__(name="h743-serial", daemon=True)
        self.port, self.events = port, events
        # Automatic discovery validates identity before publishing or initializing.
        self.expected_board, self.expected_uid = expected_board, expected_uid
        self.identified = False
        self.commands = queue.PriorityQueue(maxsize=32)
        self.order = itertools.count()
        self.quit = threading.Event()
        self.wakeup = threading.Event()
        self.sequence = secrets.randbelow(0xFFFFFFFE) + 1
        self.framer = Framer()
        self.serial = None
        self.last_ping = 0.
        self.jog_lock = threading.Lock()
        self.jog = None
        self.camera_velocity = None
        self.camera_zero_ack = None
        self.last_camera_tx = 0.
        self.last_jog_tx = 0.
        self.stop_generation = 0
        self.imu_provider = imu_provider
        self.nav_heading = False
        self.last_imu_tx = 0.
        self.last_imu_key = None
        self.imu_was_valid = False
        self.imu_epoch = None
        self.last_imu_source = None
        self.bridge_epoch = secrets.randbelow(2147483646) + 1
        self.imu_paused = False
        self.imu_pause_started = None
        self.imu_pause_total = 0.
        self.imu_pause_lock = threading.Lock()

    def imu_pause_snapshot(self):
        """Read the worker's clock atomically; measurement ages stay untouched."""
        with self.imu_pause_lock:
            now = time.monotonic()
            total = self.imu_pause_total
            if self.imu_paused and self.imu_pause_started is not None:
                total += max(0., now-self.imu_pause_started)
            return self.imu_paused, total

    def absorb_imu_pause(self, fields):
        """Track the MCU hold before publishing its ACK to the control pump."""
        terminal = fields.get('armed') == '0' or fields.get('latched') == '1'
        if 'paused' not in fields and not terminal: return
        paused = fields.get('paused') == '1' and not terminal
        with self.imu_pause_lock:
            if paused == self.imu_paused: return
            now = time.monotonic()
            if paused:
                self.imu_pause_started = now
            elif self.imu_pause_started is not None:
                self.imu_pause_total += now-self.imu_pause_started
                self.imu_pause_started = None
            self.imu_paused = paused
        if paused: self.clear_camera_velocity()

    def forward_imu(self):
        """Send the newest verified sample before queued work; no telemetry retry.

        Duplicates never renew the MCU freshness deadline. A source timestamp
        rollback or USB session change changes epoch and cancels active cruise.
        SIM owns synthetic yaw; real hardware always requires the H723 provider.
        """
        if self.port == "SIM" or not self.nav_heading or self.imu_provider is None:
            return
        now = time.monotonic()
        if now - self.last_imu_tx < .05:
            return
        sample = self.imu_provider()
        key = sample["key"]
        if sample["epoch"] != self.imu_epoch or (key and self.last_imu_source and
                (key[1] < self.last_imu_source[1] or key[2] < self.last_imu_source[2])):
            self.bridge_epoch = self.bridge_epoch % 2147483647 + 1
            self.imu_epoch = sample["epoch"]
        if key:
            self.last_imu_source = key
        if sample["valid"]:
            if key == self.last_imu_key:
                return
            self.last_imu_key = key
        elif not self.imu_was_valid:
            return
        self.last_imu_tx = now
        self.imu_was_valid = sample["valid"]
        self.exchange("NAV IMU {} {} {}".format(sample["heading_cd"], int(sample["valid"]),
                                               self.bridge_epoch))

    def set_jog(self, body):
        """Latest-only intent, refreshed by the GUI event loop, not the worker.
        Expiry stops motion even when this thread can still send PING."""
        with self.jog_lock:
            self.jog = (body, time.monotonic() + .25)
        self.wakeup.set()

    def current_jog(self):
        with self.jog_lock:
            return self.jog

    def set_camera_velocity(self, body, tag):
        """Latest-only NX visual velocity, renewed by the control loop, not PING.

        A stalled controller expires independently of this serial worker. Only
        the camera executor uses this lease; ordinary WHEELS calls keep their
        existing protocol semantics. No motion command is queued here.
        """
        if self.imu_paused: return
        with self.jog_lock:
            if not self.camera_velocity or (body, tag) != (self.camera_velocity[0], self.camera_velocity[2]):
                self.camera_zero_ack = None
            self.camera_velocity = (body, time.monotonic()+.25, tag)
        self.wakeup.set()

    def clear_camera_velocity(self):
        """Cancel pending visual targets before stop/completion/reconnect."""
        with self.jog_lock:
            self.camera_velocity = None
            self.camera_zero_ack = None

    def current_camera_velocity(self):
        with self.jog_lock:
            return self.camera_velocity

    def submit(self, body, tag=None):
        """STOP clears obsolete pending requests and takes queue priority."""
        aux_halt = body in ("AUX HALT 5", "AUX HALT 6")
        turn_halt = body == "TURN STOP"
        if turn_halt:
            # Remove obsolete angle requests, preserving auxiliary commands.
            with self.commands.mutex:
                self.commands.queue[:] = [item for item in self.commands.queue
                    if not item[2].startswith(("TURN MOVE ", "TURN ZERO"))]
                heapq.heapify(self.commands.queue)
                self.commands.not_full.notify_all()
        if aux_halt:
            axis = body[-1]
            # Preserve other axes and chassis jog leases. Remove queued moves
            # of this axis so a stop cannot be followed by an obsolete restart.
            def obsolete(item):
                parts = item[2].split()
                return (len(parts) > 2 and parts[0] == "AUX" and parts[1] in ("STEP", "GOTO") and parts[2] == axis) or \
                       (len(parts) > 1 and parts[0] == "MOVE" and parts[1] == axis) or \
                       (axis == "5" and parts[:2] == ["LIFT", "MOVE"])
            with self.commands.mutex:
                self.commands.queue[:] = [item for item in self.commands.queue if not obsolete(item)]
                heapq.heapify(self.commands.queue)
                self.commands.not_full.notify_all()
        if body in ("STOP", "CHASSIS HALT", "NAV STOP", "ARM 0", "MOTION CLEAR"):
            with self.jog_lock:
                self.jog = None
                self.camera_velocity = None
                self.camera_zero_ack = None
                self.stop_generation += 1
            while True:
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
        try:
            priority = 0 if aux_halt or turn_halt or body in ("STOP", "CHASSIS HALT", "NAV STOP", "ARM 0", "MOTION CLEAR") else 1
            if tag == ('telemetry',): priority = 2
            self.commands.put_nowait((priority, next(self.order), body, tag))
            self.wakeup.set()
            return True
        except queue.Full:
            return False

    def close(self):
        with self.jog_lock:
            self.jog = None
            self.camera_velocity = None
            self.camera_zero_ack = None
        self.quit.set()
        self.wakeup.set()

    def next_sequence(self):
        self.sequence = self.sequence % 0xFFFFFFFF + 1
        return self.sequence

    def exchange(self, body, tag=None, publish=True):
        """Retry identical bytes/ID once. Never allocate a fresh ID for a retry.

        tag echoes back on the matching "reply" event. The heartbeat PING the
        worker sends on its own behalf carries no tag, so a caller that tags its
        requests can never mistake a heartbeat answer for its own. Reply events
        stay 2-tuples, so existing consumers keep working unchanged.
        """
        sequence = self.next_sequence()
        frame = encode(sequence, body)
        for attempt in range(1 if body.startswith("NAV IMU ") else 2):
            if tag and tag[0] == 'camera_velocity':
                intent = self.current_camera_velocity()
                if not intent or intent[0] != body or intent[2] != tag or intent[1] <= time.monotonic():
                    return None
            if body.startswith("CHASSIS JOG "):
                intent = self.current_jog()
                if not intent or intent[0] != body or intent[1] <= time.monotonic():
                    return None  # Never replay an obsolete keyboard direction.
            self.serial.write(frame)
            deadline = time.monotonic() + .3
            while time.monotonic() < deadline and not self.quit.is_set():
                # Block for the first byte only, then drain bytes already here.
                # read(256) waits out the timeout on every shorter reply.
                available = getattr(self.serial, 'in_waiting', 0)
                for packet in self.framer.feed(self.serial.read(min(256, available) if available else 1)):
                    try:
                        reply = decode(packet)
                    except ValueError as exc:
                        self.events.put(("warning", str(exc)))
                        continue
                    if reply.sequence == sequence:
                        if reply.ok: self.absorb_imu_pause(reply.fields)
                        # (body, reply[, tag]) — GUIs read the first two fields.
                        payload = (body, reply) if tag is None else (body, reply, tag)
                        if publish:
                            self.events.put(("reply", payload))
                        return reply
            if self.quit.is_set():
                raise InterruptedError("connection closing")
        raise TimeoutError(f"No reply: {body}")

    def run(self):
        try:
            if self.port == "SIM":
                self.serial = SimulatedSerial()
            else:
                import serial
                from .imu import is_candidate, usb_ports
                if any(p.device == self.port and is_candidate(p) for p in usb_ports()):
                    raise RuntimeError("ST USB CDC 请通过 H723 陀螺仪页连接；未发送 H743 指令。")
                try:
                    self.serial = serial.Serial(self.port, 115200, timeout=.02, write_timeout=.3)
                except OSError as exc:
                    # Name the device: a missing /dev/ttyUSB0 and a permission
                    # denial look identical otherwise, and which port was meant
                    # is the first thing anyone needs to know.
                    raise OSError("cannot open {}: {}".format(self.port, exc)) from exc
            hello = self.exchange("HELLO", publish=not self.expected_board)
            if not hello.ok or hello.fields.get("protocol") != "1":
                raise RuntimeError("Protocol version mismatch")
            self.nav_heading = hello.fields.get("nav_heading") == "1"
            info = None
            if self.expected_board:
                if hello.fields.get("board") != self.expected_board:
                    raise RuntimeError("H743 board identity mismatch")
                info = self.exchange("INFO", publish=False)
                import re
                uid = info.fields.get("uid", "")
                if not info.ok or not re.fullmatch(r"[0-9A-Fa-f]{24}", uid) or int(uid, 16) == 0:
                    raise RuntimeError("H743 UID missing or invalid")
                if self.expected_uid and uid.upper() != self.expected_uid.upper():
                    raise RuntimeError("H743 UID mismatch; refusing replacement device")
                self.identified = True
                self.events.put(("reply", ("HELLO", hello)))
                self.events.put(("reply", ("INFO", info)))
                # Recover communication only; never resume an old ARM or move.
                if not self.exchange("STOP").ok:
                    raise RuntimeError("H743 stop after reconnect rejected")
            # Establish M5's top on the first connection of this MCU boot.
            # Firmware makes INIT idempotent, so reconnecting below top cannot
            # reset its zero or extend the remaining downward allowance.
            if hello.fields.get("lift") == "1":
                initialized = self.exchange("LIFT INIT")
                if not initialized.ok:
                    raise RuntimeError("M5 reference initialization rejected: " + initialized.body)
            self.exchange("PING")
            self.last_ping = time.monotonic()
            if info is None:
                self.exchange("INFO")
            # Initialize once per serial connection, after board identity checks.
            # Web/Tk share this worker; browser refresh and polling never move it.
            for body in servo_initial_commands(hello.fields):
                initialized = self.exchange(body)
                if not initialized.ok:
                    raise RuntimeError("Servo initialization rejected: " + initialized.body)
            if hello.fields.get("servo"):
                self.exchange("SERVO STAT")
            self.events.put(("connected", self.port))
            while not self.quit.is_set():
                # Clear before checking the queue so submissions cannot lose a wake.
                self.wakeup.clear()
                # Process release/STOP ahead of heartbeats and renewed jogs.
                generation = self.stop_generation
                pending = None
                try:
                    pending = self.commands.get_nowait()
                except queue.Empty:
                    pass
                if pending is not None and pending[0] == 0:
                    self.exchange(pending[2], pending[3])
                    continue
                self.forward_imu()
                if self.imu_paused:
                    # Keep exact queued commands/IDs. No old visual velocity is
                    # replayed; safety commands above still run immediately.
                    if pending is not None and generation == self.stop_generation:
                        # The vacated worker slot may have been filled by a
                        # producer. Restore it atomically without losing work.
                        with self.commands.mutex:
                            if generation == self.stop_generation:
                                heapq.heappush(self.commands.queue, pending)
                                self.commands.not_empty.notify()
                    if time.monotonic()-self.last_ping >= .25:
                        self.exchange('PING')
                        self.exchange('STAT')
                        self.exchange('MOTION STAT')
                        self.exchange('NAV STAT')
                        self.last_ping = time.monotonic()
                    self.wakeup.wait(.01)
                    continue
                velocity = self.current_camera_velocity()
                if velocity and velocity[1] <= time.monotonic():
                    self.submit('STOP')
                    pending = None
                    continue
                intent = self.current_jog()
                if intent and intent[1] <= time.monotonic():
                    self.submit("CHASSIS HALT")
                    if pending is not None:
                        # Drop stale commands rather than restarting after a frozen UI.
                        pending = None
                    continue
                if time.monotonic() - self.last_ping >= .25:
                    reply = self.exchange("PING")
                    if not reply.ok:
                        raise RuntimeError("Heartbeat rejected")
                    self.last_ping = time.monotonic()
                intent = self.current_jog()
                if intent and time.monotonic() - self.last_jog_tx >= .15:
                    reply = self.exchange(intent[0])
                    if reply is not None and not reply.ok:
                        self.submit('STOP')
                    self.last_jog_tx = time.monotonic()
                velocity = self.current_camera_velocity()
                # Firmware 0.2.21 rejects updates during the final STEP high
                # even when the requested speed is zero. Once this exact zero
                # intent is ACKed, keep supervising its lease and polling idle,
                # without reissuing it into that stop tail. New intents, failed
                # ACKs and nonzero speeds retain their normal send behavior.
                zero_key = ((velocity[0], velocity[2]) if velocity and
                            velocity[0].startswith('WHEELS 0 0 0 0 ') else None)
                if (velocity and time.monotonic()-self.last_camera_tx >= .05
                        and (zero_key is None or zero_key != self.camera_zero_ack)):
                    reply = self.exchange(velocity[0], velocity[2])
                    self.last_camera_tx = time.monotonic()
                    if reply is not None and not reply.ok:
                        self.submit('STOP')
                    elif reply is not None and reply.ok and zero_key is not None:
                        with self.jog_lock:
                            current = self.camera_velocity
                            if current and (current[0], current[2]) == zero_key:
                                self.camera_zero_ack = zero_key
                if pending is not None and generation == self.stop_generation:
                    self.exchange(pending[2], pending[3])
                else:
                    self.wakeup.wait(.01)
        except InterruptedError:
            pass
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            if self.serial is not None:
                try:
                    if not self.expected_board or self.identified:
                        self.serial.write(encode(self.next_sequence(), "STOP"))
                except Exception:
                    pass  # Board lease also expires if the link is physically gone.
                try:
                    self.serial.close()
                except Exception as exc:
                    self.events.put(("warning", f"Port close: {exc}"))
            self.events.put(("disconnected", self.port))
