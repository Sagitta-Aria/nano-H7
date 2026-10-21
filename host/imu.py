"""Dedicated H723 USB IMU connection, identity handshake and bounded telemetry.

Never scans by opening arbitrary ports. A selected ST USB CDC candidate must
first stream IMU1, then answer a fresh challenge with exact board/MCU identity.
This channel never sends H743 motion, STOP or heartbeat commands.
"""
import collections
from dataclasses import asdict
import re
import secrets
import threading
import time

from .imu_protocol import parse_frame

USB_VID = 0x0483
USB_PID = 0x5740
HISTORY_SIZE = 1500


def usb_ports():
    """Enumerate USB metadata only; never open a port to discover its type."""
    try:
        import serial  # Cached list_ports must not hide an unavailable pyserial package.
        from serial.tools.list_ports import comports
        return list(comports())
    except ImportError:
        return []


def is_candidate(port):
    """ST CDC IDs are only candidates, not proof that a device is an H723."""
    return (getattr(port, "vid", None), getattr(port, "pid", None)) == (USB_VID, USB_PID)


def candidate_ports():
    """Return display metadata for explicitly selectable ST CDC candidates."""
    return [{"id": p.device, "description": p.description or "ST USB CDC",
             "serial_number": p.serial_number or "", "verified": False,
             "vid": USB_VID, "pid": USB_PID} for p in usb_ports() if is_candidate(p)]


def parse_identity(line, nonce):
    """Verify a challenge reply; generic serial names or old IMU1 data cannot
    substitute for an exact DM-MC02/H723/BMI088 identity and a nonzero UID."""
    try:
        parts = line.decode("ascii").strip().split()
        if parts[:2] != ["IMU!", nonce]:
            return None
        fields = dict(part.split("=", 1) for part in parts[2:])
    except (UnicodeDecodeError, ValueError):
        return None
    expected = {"board": "DM-MC02", "mcu": "STM32H723VGT6",
                "sensor": "BMI088", "protocol": "IMU1"}
    uid = fields.get("uid", "")
    if not all(fields.get(k) == v for k, v in expected.items()):
        return None
    if not re.fullmatch(r"[0-9a-fA-F]{24}", uid) or int(uid, 16) == 0:
        return None
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", fields.get("fw", "")):
        return None
    return fields


class LineStream:
    """Reassemble bounded ASCII lines across USB packets and read timeouts."""
    def __init__(self, serial):
        """The caller owns the serial object and closes it after the worker."""
        self.serial = serial
        self.partial = bytearray()
        self.discarding = False

    def read(self):
        """Return a complete line or None; oversized lines cannot grow memory."""
        chunk = self.serial.read_until(b"\n", size=769)
        if not chunk:
            return None
        if self.discarding:
            self.discarding = not chunk.endswith(b"\n")
            return None
        self.partial.extend(chunk)
        if len(self.partial) > 768:
            self.partial.clear()
            self.discarding = not chunk.endswith(b"\n")
            return None
        if not chunk.endswith(b"\n"):
            return None
        line = bytes(self.partial)
        self.partial.clear()
        return line


class ImuHub:
    """Own one independent IMU thread and share snapshots with Web/Tk views.

    History and pending updates are bounded. Caller must close on application
    exit. Unverified data is never published as measurements, even in preview.
    """
    def __init__(self, port_provider=candidate_ports, serial_factory=None):
        """Dependency injection is for protocol tests, not a UI bypass."""
        self.lock = threading.RLock()
        self.port_provider = port_provider
        self.serial_factory = serial_factory
        self.thread = None
        self.stop = threading.Event()
        self.history = collections.deque(maxlen=HISTORY_SIZE)
        self.pending = collections.deque(maxlen=200)
        self.latest = None
        self.state = "offline"
        self.message = "未连接 H723"
        self.identity = {}
        self.port = None
        self.generation = 0
        self.dirty = True
        self.last_new_sample = 0.
        self.rejected = 0
        self.revision = 0
        self.expected_uid = None

    def ports(self):
        """Metadata only: list candidates without side effects on any device."""
        return {"ports": self.port_provider(), "current": self.port}

    def connect(self, port):
        """Validate a current candidate before opening; never guess a COM name."""
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False, "H723 已连接或正在握手，请先断开。"
            candidate = next((p for p in self.port_provider() if p["id"] == port), None)
            if candidate is None:
                return False, "该端口不是当前 ST USB CDC 候选设备，未打开串口。"
            if not candidate.get("serial_number"):
                return False, "USB 设备没有序列号，无法可靠核对身份，未打开串口。"
            self.stop = threading.Event()
            self.port = port
            self.generation += 1
            self.identity = {}
            self.latest = None
            self.history.clear()
            self.pending.clear()
            self.rejected = 0
            self._status("handshaking", "正在核对 H723 身份…")
            self.thread = threading.Thread(target=self._run, args=(dict(candidate),),
                                           name="h723-usb-imu", daemon=True)
            self.thread.start()
        return True, "handshaking"

    def _status(self, state, message):
        """Update link state while holding the hub lock; caller owns wording."""
        self.state, self.message, self.dirty = state, message, True
        self.revision += 1

    def close(self):
        """Cancel reads/handshake without writing any command to the device."""
        with self.lock:
            self.stop.set()
            self._status("closing", "正在断开 H723…")
            thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1.)
        with self.lock:
            if not thread or not thread.is_alive():
                self._status("offline", "H723 已断开，历史数据已停止更新。")

    def _check_candidate(self, original):
        """Reject an unplug/replug that reassigned the selected COM name."""
        current = next((p for p in self.port_provider() if p["id"] == original["id"]), None)
        if current is None or current.get("serial_number") != original.get("serial_number"):
            raise RuntimeError("串口设备已变化，请刷新列表并重新选择。")

    def _handshake(self, stream):
        """Read IMU1 first, then send one nonce challenge; never write to a
        device that only matches the generic ST USB VID/PID but sends no IMU1."""
        deadline = time.monotonic() + 3.
        while not self.stop.is_set() and time.monotonic() < deadline:
            line = stream.read()
            if line is None:
                continue
            try:
                parse_frame(line)
                break
            except ValueError:
                continue
        else:
            raise RuntimeError("未收到 IMU1 陀螺仪数据，设备未确认为 H723；没有发送握手指令。")
        if self.stop.is_set():
            raise InterruptedError()
        nonce = secrets.token_hex(8)
        stream.serial.write(("IMU? " + nonce + "\n").encode("ascii"))
        deadline = time.monotonic() + 3.
        while not self.stop.is_set() and time.monotonic() < deadline:
            line = stream.read()
            if line is None:
                continue
            identity = parse_identity(line, nonce)
            if identity:
                if self.expected_uid and identity["uid"].upper() != self.expected_uid.upper():
                    raise RuntimeError("H723 UID 不匹配，拒绝连接替换设备。")
                return identity
        raise RuntimeError("H723 专用握手未通过；请烧录支持握手的陀螺仪固件，不能仅凭串口名识别。")

    def _run(self, candidate):
        """Serial owner; only verified samples reach either front end."""
        serial_port = None
        try:
            self._check_candidate(candidate)
            if self.serial_factory:
                serial_port = self.serial_factory(candidate["id"])
            else:
                import serial
                serial_port = serial.Serial(candidate["id"], 115200, timeout=.1,
                                             write_timeout=.3, exclusive=True)
            serial_port.reset_input_buffer()
            stream = LineStream(serial_port)
            identity = self._handshake(stream)
            self._check_candidate(candidate)
            with self.lock:
                if self.stop.is_set():
                    raise InterruptedError()
                self.identity = identity
                self.last_new_sample = time.monotonic()
                self._status("calibrating", "H723 身份已确认，等待静止校准…")
            started = time.monotonic()
            previous = None
            while not self.stop.is_set():
                line = stream.read()
                now = time.monotonic()
                if line:
                    try:
                        sample = parse_frame(line)
                    except ValueError:
                        with self.lock:
                            self.rejected += 1
                    else:
                        key = (sample.timestamp_ms, sample.sample_count)
                        if key != previous:
                            previous = key
                            point = asdict(sample)
                            point.update(valid=sample.valid, time_s=now - started)
                            with self.lock:
                                self.latest = point
                                self.last_new_sample = now
                                self.history.append(point)
                                self.pending.append(point)
                                if sample.valid:
                                    self._status("streaming", "H723 已握手 · 姿态数据有效")
                                elif sample.ready and not sample.calibrated:
                                    self._status("calibrating", "保持板子静止，正在校准零偏…")
                                else:
                                    self._status("invalid", "传感器数据无效，检查板端状态。")
                elapsed = now - self.last_new_sample
                if elapsed > .5:
                    with self.lock:
                        self._status("stale", "H723 数据已过期，图形停止更新。")
        except InterruptedError:
            pass
        except Exception as exc:
            with self.lock:
                if not self.stop.is_set():
                    self._status("error", str(exc))
        finally:
            if serial_port:
                serial_port.close()
            with self.lock:
                if self.stop.is_set():
                    self._status("offline", "H723 已断开，历史数据已停止更新。")
                self.dirty = True

    def snapshot(self, history=False):
        """Return immutable JSON data; measurements from stale links are invalid."""
        with self.lock:
            latest = dict(self.latest) if self.latest else None
            age = max(0., time.monotonic()-self.last_new_sample)
            if latest and (self.state != "streaming" or age >= .15):
                latest["valid"] = False
            result = {"state": self.state, "message": self.message, "port": self.port,
                      "identity": dict(self.identity), "latest": latest,
                      "generation": self.generation, "rejected": self.rejected,
                      "revision": self.revision, "sample_age_ms": round(age*1000, 1) if latest else None,
                      "navigation_valid": bool(latest and latest.get('valid') and self.identity)}
            if history:
                result["history"] = list(self.history)
            return result

    def navigation_feedback(self):
        """Read fresh verified yaw for H743. No USB I/O or motor commands here.

        BMI088 yaw is CCW about Z-up; navigation is clockwise from departure.
        The serial worker calls this at send time, never from a queued UI event.
        """
        with self.lock:
            point = self.latest
            valid = bool(point and point.get("valid") and self.identity and
                         self.state == "streaming" and time.monotonic() - self.last_new_sample < .15)
            key = (self.generation, point["timestamp_ms"], point["sample_count"]) if point else None
            angle = round(-point["yaw_deg"] * 100) if point else 0
            angle = (angle + 18000) % 36000 - 18000
            return {"key": key, "valid": valid, "heading_cd": angle,
                    "epoch": max(1, self.generation),
                    "age_ms": round(max(0., time.monotonic()-self.last_new_sample)*1000, 1) if point else None,
                    "reason": '' if valid else ('sample_timeout' if point and point.get('valid') else 'sensor_invalid')}

    def drain(self):
        """Batch telemetry for UI refresh; no unbounded per-sample event queue."""
        with self.lock:
            if not self.dirty and not self.pending:
                return []
            points = list(self.pending)
            self.pending.clear()
            self.dirty = False
            return [{"type": "imu", **self.snapshot(), "points": points}]
