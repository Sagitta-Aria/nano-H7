"""Linux USB capture isolated from robot I/O in a terminable subprocess.

The NX display and remembered task camera can open at service startup. OpenCV
is imported by the worker, never the server; failures cannot stop board control.
"""
import argparse
import copy
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path


def device_identity(device):
    """Use udev links as persistent IDs; videoN alone is never a binding."""
    aliases = []
    for directory in ("/dev/v4l/by-id", "/dev/v4l/by-path"):
        for link in sorted(Path(directory).glob("*")):
            if link.resolve() == Path(device).resolve():
                aliases.append(str(link))
    node = Path("/sys/class/video4linux") / Path(device).name
    physical = node.resolve().parent.parent
    # Multiple V4L2 interfaces of one USB camera share the USB parent.
    for parent in physical.parents:
        if parent.joinpath("idVendor").exists():
            physical = parent
            break
    return {"stable_id": aliases[0] if aliases else "", "aliases": aliases,
            "physical_id": str(physical)}


def csi_identity(label, root=None):
    """Match a Tegra sensor to its device-tree connector identity.

    CSI cameras generally lack a unique serial; identity follows the port and
    sensor path instead, independently of the assigned videoN number.
    """
    root = Path(root or "/proc/device-tree/tegra-camera-platform/modules")
    for module in sorted(root.glob("module*")):
        for driver in module.glob("drivernode*"):
            try:
                devname = driver.joinpath("devname").read_bytes().rstrip(b"\0").decode()
                path = driver.joinpath("proc-device-tree").read_bytes().rstrip(b"\0").decode()
                if devname and devname in label:
                    identity = "csi:" + path
                    return {"stable_id": identity, "aliases": [identity],
                            "physical_id": identity, "kind": "csi"}
            except (OSError, UnicodeDecodeError):
                continue
    return None


def devices():
    """Enumerate V4L2 capture nodes and native sizes; omit metadata-only nodes."""
    found = []
    if not sys.platform.startswith("linux"):
        return found
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        device = "/dev/" + node.name
        try:
            result = subprocess.run(["v4l2-ctl", "-d", device, "--list-formats-ext"],
                                    capture_output=True, text=True, timeout=3)
            sizes = sorted(set((int(w), int(h)) for w, h in
                               re.findall(r"Size: Discrete (\d+)x(\d+)", result.stdout)))
            if result.returncode or not sizes:
                continue
            label = node.joinpath("name").read_text().strip()
            identity = csi_identity(label) or dict(device_identity(device), kind="usb")
            found.append({"id": device, **identity, "label": label,
                          "resolutions": ["{}x{}".format(w, h) for w, h in sizes],
                          "mjpeg": "'MJPG'" in result.stdout})
        except (OSError, subprocess.TimeoutExpired):
            continue
    for index, item in enumerate(d for d in found if d["kind"] == "csi"):
        item["sensor_id"] = index
    return found


class CameraSlots:
    """Two independent capture hubs with persistent USB or CSI bindings.

    Enumerating never binds a new camera. A missing remembered camera remains
    missing, even when another camera takes its former videoN device number.
    """
    def __init__(self, path=None):
        self.path = Path(path) if path is not None else Path(__file__).with_name("cameras.json")
        self.lock = threading.RLock()
        self.hubs = {"1": CameraHub(display=os.environ.get('H743_CAMERA_DISPLAY') == '1'), "2": CameraHub(qr=True)}
        self.bindings = json.loads(self.path.read_text(encoding="utf-8")) if self.path.exists() else {}

    def listing(self, slot):
        """Expose the remembered ID and only devices available to this slot."""
        with self.lock:
            found = devices()
            binding = self.bindings.get(slot)
            other = "2" if slot == "1" else "1"
            other_binding = self.bindings.get(other)
            other_device = next((d for d in found if other_binding and
                                 other_binding["id"] in d.get("aliases", [])), None)
            other_active = self.hubs[other].snapshot().get("device")
            if other_device is None:
                other_device = next((d for d in found if d["id"] == other_active), None)
            available = [d for d in found if not other_device or
                         d.get("physical_id", d["id"]) != other_device.get("physical_id", other_device["id"])]
            selected = next((d for d in available if binding and binding["id"] in d.get("aliases", [])), None)
            return {"supported": sys.platform.startswith("linux"), "devices": available,
                    "binding": binding, "bound_device": selected["id"] if selected else None}

    def bind(self, slot, device):
        """Remember an explicitly selected USB identity without opening it."""
        with self.lock:
            selected = next((d for d in self.listing(slot)["devices"] if d["id"] == device), None)
            if not selected:
                raise ValueError("摄像头未连接或已属于另一个栏目，请刷新设备。")
            if not selected.get("stable_id"):
                raise ValueError("设备没有稳定标识，无法记住编号。")
            if self.hubs[slot].process:
                raise ValueError("请先关闭本栏目摄像头，再修改绑定。")
            binding = {"id": selected["stable_id"], "label": selected["label"],
                       "kind": selected.get("kind", "usb")}
            updated = dict(self.bindings, **{slot: binding})
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(updated, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self.path)
            self.bindings = updated
            return binding

    def start(self, slot, device, resolution):
        """Resolve a saved identity at each open and refuse cross-slot use."""
        with self.lock:
            listing = self.listing(slot)
            if listing["binding"]:
                if not listing["bound_device"]:
                    raise ValueError("已绑定的摄像头未连接，请接回原摄像头或明确重新绑定。")
                if device and device != listing["bound_device"]:
                    raise ValueError("所选设备与本栏目记住的摄像头不同，请先重新绑定。")
                device = listing["bound_device"]
            if not any(d["id"] == device for d in listing["devices"]):
                raise ValueError("摄像头未连接或已属于另一个栏目。")
            return self.hubs[slot].start(device, resolution)

    def stop(self, slot):
        with self.lock:
            self.hubs[slot].stop()
            return self.hubs[slot].snapshot()

    def close(self):
        for slot in self.hubs:
            self.stop(slot)


class CameraHub:
    """Own NX raw recognition and an independent, manually requested photo."""
    def __init__(self, display=False, qr=False):
        self.lock = threading.RLock()
        self.process = None
        self.jpeg = None
        self.frame = {"state": "idle", "message": "尚未打开 Nano 摄像头。"}
        self.sequence = 0
        self.frame_time = 0.
        self.changed = threading.Condition(self.lock)
        self.capturing = False
        self.tracking_id = None
        self.tracking_serial = 0
        self.tracking_report = None
        self.tracking_error = None
        self.tuning_serial = 0
        self.tuning_result = None
        self.display = display
        self.qr = qr

    def snapshot(self):
        """Copy state quickly without doing device I/O on the serial pump."""
        with self.lock:
            state = copy.deepcopy(self.frame)
            if self.qr and state.get('qr'):
                result = state['qr']
                if state['state'] != 'ready' or time.time() - result.get('captured_at', 0) > 3:
                    result.update(status='stale', code=None, groups=[], digits=[], task_plan=None)
            if state["state"] == "opening" and time.monotonic() - self.frame_time > 10:
                state.update(state="error", message="摄像头采集超时，请关闭后重新打开。")
            return state

    def image(self):
        with self.lock:
            return self.jpeg

    def capture(self):
        """Request exactly one photo; timeout/unplug cannot block robot workers."""
        with self.changed:
            process = self.process
            if not process or self.frame["state"] != "ready":
                raise ValueError("请先打开 Nano 摄像头，等待设备准备完成。")
            if self.capturing:
                raise ValueError("正在拍照，请等待本次拍照完成。")
            sequence = self.sequence
            self.capturing = True
            try:
                process.stdin.write(b'SNAP\n')
                process.stdin.flush()
                self.changed.wait_for(lambda: self.sequence != sequence or self.process is not process, timeout=8)
                if self.process is not process:
                    raise ValueError("摄像头已关闭或断开，请重新打开。")
                if self.sequence == sequence:
                    raise ValueError("拍照超时，请关闭摄像头后重新打开。")
                return self.snapshot()
            finally:
                self.capturing = False

    def tune(self, config):
        """Request one diagnostic raw frame in the existing camera subprocess.

        No second camera is opened and no tracking or motor command is sent.
        Matching request IDs prevent a timed-out result satisfying a later call.
        """
        with self.changed:
            process = self.process
            if not process or self.frame['state'] != 'ready':
                raise ValueError('请先打开 NX 摄像头一。')
            if self.tracking_id is not None:
                raise ValueError('请先停止视觉对准或识别。')
            self.tuning_serial += 1
            identity = self.tuning_serial
            self.tuning_result = None
            request = dict(request_id=identity, config=config)
            process.stdin.write(('TUNE '+json.dumps(request, ensure_ascii=True)+'\n').encode('ascii'))
            process.stdin.flush()
            if not self.changed.wait_for(lambda: self.tuning_result is not None or self.process is not process, timeout=8):
                raise ValueError('调参预览超时，请重试。')
            if self.process is not process or self.tuning_result is None:
                raise ValueError('摄像头已关闭或断开。')
            result = self.tuning_result
            if result.get('error'):
                raise ValueError(result['error'])
            return result

    def start_tracking(self, config, phase=None):
        """Start NX-local raw-frame recognition, independent of photo requests."""
        if self.qr:
            raise ValueError('摄像头2仅用于二维码任务码识别。')
        with self.changed:
            if not self.process or self.frame['state'] != 'ready':
                raise ValueError('请先打开 NX 摄像头一。')
            self.tracking_serial += 1
            self.tracking_id = self.tracking_serial
            self.tracking_report = self.tracking_error = None
            request = {'stream_id': self.tracking_id, 'config': config, 'phase': phase}
            self.process.stdin.write(('TRACK '+json.dumps(request, ensure_ascii=True)+'\n').encode('ascii'))
            self.process.stdin.flush()

    def stop_tracking(self):
        """End recognition only; the camera and last manual photo stay available."""
        with self.changed:
            identity, self.tracking_id = self.tracking_id, None
            self.tracking_report = self.tracking_error = None
            if identity is not None and self.process:
                try:
                    self.process.stdin.write(('UNTRACK '+str(identity)+'\n').encode('ascii'))
                    self.process.stdin.flush()
                except (OSError, ValueError):
                    pass
            self.changed.notify_all()

    def tracking_sample(self, after_monotonic, after_frame_id=0, timeout=3):
        """Wait for a new local detection after motion; never request a JPEG."""
        with self.changed:
            identity = self.tracking_id
            def ready():
                report = self.tracking_report
                return (self.tracking_id != identity or self.tracking_error or not self.process or
                        (report and report['frame_id'] > after_frame_id and
                         report['captured_monotonic'] >= after_monotonic))
            if identity is None or not self.changed.wait_for(ready, timeout=timeout):
                raise ValueError('NX 连续识别未产生新的有效帧。')
            if self.tracking_id != identity or not self.process:
                raise ValueError('NX 连续识别已停止或摄像头断开。')
            if self.tracking_error:
                raise ValueError(self.tracking_error)
            return copy.deepcopy(self.tracking_report)

    def start(self, device, resolution):
        """Validate native modes before replacing a working capture process."""
        selected = next((item for item in devices() if item["id"] == device), None)
        if not selected:
            raise ValueError("未找到 Nano 上的摄像头，请检查 摄像头连接并刷新设备。")
        if resolution not in selected["resolutions"]:
            raise ValueError("Nano 摄像头不支持所选分辨率，当前画面保持不变。")
        width, height = resolution.split("x")
        with self.lock:
            self.stop()
            executable = os.environ.get('H743_CAMERA_PYTHON', sys.executable) if (self.display or self.qr) else sys.executable
            command = [executable, "-u", "-m", "host.camera", "--worker", device,
                       "--width", width, "--height", height]
            if self.display:
                command.append('--display')
            if self.qr:
                command.append('--qr')
            if selected["mjpeg"]:
                command.append("--mjpeg")
            if selected.get("kind") == "csi":
                command.extend(["--sensor-id", str(selected["sensor_id"])])
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                       cwd=str(Path(__file__).resolve().parents[1]))
            self.process = process
            self.frame_time = time.monotonic()
            self.frame = {"state": "opening", "message": "正在打开 Nano 摄像头…",
                          "device": device, "label": selected["label"], "requested": resolution}
            threading.Thread(target=self._read, args=(process,), daemon=True,
                             name="nano-camera-reader").start()
        return self.snapshot()

    def stop(self):
        """Terminate even a driver blocked in read(); never wait on that thread."""
        with self.lock:
            self.tracking_id = None
            self.tracking_report = self.tracking_error = None
            process, self.process = self.process, None
            self.jpeg = None
            self.frame = {"state": "idle", "message": "Nano 摄像头已关闭，设备已释放。"}
            self.changed.notify_all()
        if process:
            try:
                process.terminate()
            except ProcessLookupError:
                pass  # The reader may already have reaped this worker.
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=1)

    def _read(self, process):
        """Route raw detection metadata and photos to separate, generation-safe caches."""
        error = "Nano 摄像头已断开或无法读取，请检查 摄像头连接。"
        try:
            while True:
                line = process.stdout.readline(16384)
                if not line:
                    break
                header = json.loads(line)
                if header.get("error"):
                    error = header["error"]
                    break
                if header.get('qr_only'):
                    with self.changed:
                        if self.process is not process: return
                        self.frame['qr'] = header['qr']
                        self.changed.notify_all()
                    continue
                if header.get('vision_only'):
                    # Continuous metadata has its own cache. A manual photo
                    # cannot replace a detection, and a detection cannot satisfy
                    # a waiting photo request or advance its photo sequence.
                    with self.changed:
                        if self.process is not process: return
                        if header.get('stream_id') == self.tracking_id:
                            self.tracking_report = header.get('vision')
                            self.tracking_error = header.get('vision_error')
                            self.changed.notify_all()
                    continue
                length = header.pop("length")
                if not 0 <= length <= 16 * 1024 * 1024:
                    raise ValueError("invalid camera frame length")
                data = process.stdout.read(length)
                if len(data) != length:
                    break
                if header.get('tuning_only'):
                    with self.changed:
                        if self.process is not process: return
                        if header.get('request_id') == self.tuning_serial:
                            self.tuning_result = json.loads(data)
                            self.changed.notify_all()
                    continue
                with self.lock:
                    if self.process is not process:
                        return
                    if length:
                        self.sequence += 1
                        self.jpeg = data
                        header["taken_at"] = time.time()
                    self.frame_time = time.monotonic()
                    self.frame.update(header, state="ready", sequence=self.sequence)
                    self.changed.notify_all()
        except (OSError, ValueError, KeyError, TypeError):
            pass
        finally:
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass  # stop() can terminate/reap concurrently on Python 3.8.
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=1)
            process.stdout.close()
            process.stdin.close()
            with self.lock:
                if self.process is process:
                    self.process = None
                    self.jpeg = None
                    self.frame.update(state="error", message=error)
                    self.changed.notify_all()


def csi_pipeline(sensor_id, width, height):
    """Convert Jetson Argus CSI/ISP output to BGR for the manual photo worker."""
    return ("nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM), "
            "width={}, height={}, format=NV12 ! nvvidconv ! "
            "video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
            "appsink name=photo max-buffers=1 drop=true sync=false").format(sensor_id, width, height)


class CsiCapture:
    """Use the system Gst/Argus binding even with a pip OpenCV lacking Gst.

    This adapter exists only inside the isolated photo worker. Frames stay on
    Nano; JPEG encoding and delivery happen only after an explicit SNAP.
    """
    def __init__(self, sensor_id, width, height):
        import gi
        gi.require_version("Gst", "1.0")
        gi.require_version("GstVideo", "1.0")
        from gi.repository import Gst, GstVideo
        self.Gst, self.GstVideo = Gst, GstVideo
        Gst.init(None)
        self.pipeline = Gst.parse_launch(csi_pipeline(sensor_id, width, height))
        self.sink = self.pipeline.get_by_name("photo")
        self.properties = {}
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.opened = self.grab()
        except Exception:
            self.release()
            raise

    def sample(self):
        sample = self.sink.emit("try-pull-sample", 8 * self.Gst.SECOND)
        if sample is None:
            message = self.pipeline.get_bus().pop_filtered(self.Gst.MessageType.ERROR)
            if message:
                error, _ = message.parse_error()
                raise RuntimeError("CSI 摄像头采集失败：" + error.message)
            raise RuntimeError("CSI 摄像头采集超时，请检查排线、传感器驱动及 Argus 服务。")
        caps = sample.get_caps()
        info = self.GstVideo.VideoInfo()
        info.from_caps(caps)
        self.properties = {3: info.width, 4: info.height,
                           5: info.fps_n / info.fps_d if info.fps_d else 0}
        return sample, info

    def isOpened(self):
        return self.opened

    def get(self, property_id):
        return self.properties.get(property_id, 0)

    def grab(self):
        self.sample()
        return True

    def read(self):
        import numpy as np
        sample, info = self.sample()
        buffer = sample.get_buffer()
        data = buffer.extract_dup(0, buffer.get_size())
        frame = np.ndarray((info.height, info.width, 3), dtype=np.uint8,
                           buffer=data, strides=(info.stride[0], 3, 1)).copy()
        return True, frame

    def release(self):
        self.pipeline.set_state(self.Gst.State.NULL)


class CameraFrameSource:
    """One camera owner; continuous raw frames for control, shutters for viewing.

    The grab thread drains the driver while tracking is active and retains only
    its newest raw frame. Neither detector latency nor JPEG encoding can build a
    queue of old camera buffers. When idle it waits for an explicit shutter.
    """
    def __init__(self, capture):
        self.capture = capture
        self.changed = threading.Condition()
        self.continuous = self.shutter = self.closed = False
        self.flush_next = True
        self.sequence = 0
        self.latest = self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True, name='nx-camera-raw')
        self.thread.start()

    def stream(self, enabled):
        with self.changed:
            if enabled and not self.continuous: self.flush_next = True
            self.continuous = enabled
            self.changed.notify_all()

    def next_frame(self, after=0, shutter=False, timeout=8):
        """Return a distinct raw frame; only manual use requests a shutter."""
        with self.changed:
            if shutter:
                after = self.sequence
                self.shutter = True
                self.changed.notify_all()
            if not self.changed.wait_for(lambda: self.sequence > after or self.error or self.closed, timeout):
                raise TimeoutError('等待 NX 相机新帧超时。')
            if self.error: raise RuntimeError(self.error)
            if self.closed: raise RuntimeError('NX 相机已关闭。')
            return self.latest

    def close(self):
        with self.changed:
            self.closed = True
            self.changed.notify_all()
        self.thread.join(timeout=1)

    def _run(self):
        try:
            while True:
                with self.changed:
                    self.changed.wait_for(lambda: self.closed or self.continuous or self.shutter)
                    if self.closed: return
                    flush = not self.continuous or self.flush_next
                    self.flush_next = False
                    self.shutter = False
                if flush:
                    for _ in range(4):
                        if not self.capture.grab(): raise RuntimeError('NX 摄像头采集失败。')
                captured_mono, captured_wall = time.monotonic(), time.time()
                ok, frame = self.capture.read()
                if not ok: raise RuntimeError('NX 摄像头已断开或无法采集。')
                with self.changed:
                    self.sequence += 1
                    self.latest = (self.sequence, captured_mono, captured_wall, frame)
                    self.changed.notify_all()
        except Exception as error:
            with self.changed:
                self.error = str(error)
                self.changed.notify_all()
        finally:
            self.capture.release()


def present_local_window():
    """Leave GNOME's overview so the requested NX full-screen camera is visible.

    Run once after the first image, never during ordinary control updates.
    Non-GNOME desktops simply keep the normal OpenCV window presentation.
    """
    if not sys.platform.startswith('linux'): return
    try:
        subprocess.run(['gdbus','call','--session','--dest','org.gnome.Shell',
                        '--object-path','/org/gnome/Shell','--method','org.gnome.Shell.Eval',
                        'Main.overview.hide();'],capture_output=True,timeout=2)
    except (OSError,subprocess.TimeoutExpired):
        pass


def worker(device, width, height, mjpeg, sensor_id=None, display=False, qr=False):
    """Run local raw acquisition/recognition and an independent photo command.

    TRACK emits coordinates only. SNAP emits a photo only. Their consumers and
    scheduling are independent; a browser need not be connected for alignment.
    """
    capture = None
    source = None
    tracking = None
    monitor = None
    monitor_thread = None
    qr_thread = None
    monitor_stopped = threading.Event()
    display_report = [None]
    protocol_output = sys.stdout.buffer
    output_lock = threading.Lock()
    def emit(header, data=b''):
        header = dict(header, length=len(data))
        with output_lock:
            protocol_output.write(json.dumps(header, ensure_ascii=True).encode()+b'\n'+data)
            protocol_output.flush()
    def track(config, identity, stopped, phase=None):
        after = 0
        try:
            from .camera_vision import detect
            from .ring_perception import RingOCR
            from .ring_detector import RingDetector
            detector = RingDetector() if config['kind'] == 'ring' else None
            ocr = RingOCR() if phase in ('1', '2', '3') and 'stack_layout' not in config else None
            while not stopped.is_set():
                try:
                    frame_id, mono, wall, raw = source.next_frame(after, timeout=.2)
                except TimeoutError:
                    continue
                after = frame_id
                report = detect(raw.copy(), config, wall, ocr=ocr, expected=phase,
                                ring_detector=detector.detect if detector else None)
                report.update(frame_id=frame_id, captured_monotonic=mono, stream_id=identity,
                              processing_ms=round((time.monotonic()-mono)*1000, 1))
                if not stopped.is_set():
                    display_report[0] = report
                    emit({'vision_only': True, 'stream_id': identity, 'vision': report})
        except Exception as error:
            if not stopped.is_set():
                emit({'vision_only': True, 'stream_id': identity, 'vision_error': str(error)})
        finally:
            if 'detector' in locals() and detector is not None:
                detector.close()
            if 'ocr' in locals() and ocr is not None:
                ocr.close()
    try:
        import cv2
        cv2.setNumThreads(1)
        capture = CsiCapture(sensor_id, width, height) if sensor_id is not None else cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError("无法打开 Nano 摄像头，请检查权限或是否被其他程序占用。")
        if sensor_id is None:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # Request the fastest mode; V4L2 clamps to the selected
            # resolution's hardware cap (640x480 ~120, 800x600/720p ~60).
            capture.set(cv2.CAP_PROP_FPS, 120)
            # This UVC driver halves acquisition rate with one buffer: decoding
            # owns it while the next exposure needs a queued buffer (14.9 vs
            # 29.8 FPS at 1024x768). The reader drains these two continuously;
            # perception still receives only its newest frame, never a FIFO.
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            if mjpeg:
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ready = {"length": 0, "width": actual_width, "height": actual_height,
                 "fps": capture.get(cv2.CAP_PROP_FPS), "message": "Nano 摄像头已打开，点击“拍照”获取一帧。"}
        emit(ready)
        source = CameraFrameSource(capture)
        if qr:
            from .camera_qr import TaskQR
            decoder = TaskQR()
            source.stream(True)
            def qr_frames():
                """Continuously decode camera two without requiring a browser shutter."""
                after = 0
                try:
                    while not monitor_stopped.is_set():
                        try:
                            frame_id, mono, wall, raw = source.next_frame(after, timeout=.2)
                        except TimeoutError:
                            continue
                        after = frame_id
                        report = decoder.process(raw, wall)
                        report.update(frame_id=frame_id, processing_ms=round((time.monotonic()-mono)*1000, 1))
                        emit({'qr_only': True, 'qr': report})
                        monitor_stopped.wait(.1)
                except Exception as error:
                    emit({'error': '二维码识别停止：'+str(error)})
            qr_thread = threading.Thread(target=qr_frames, daemon=True, name='nx-camera-qr')
            qr_thread.start()
        # GUI calls belong to the process main thread. stdin has its own
        # reader so a quiet browser cannot freeze the physical NX monitor.
        commands = queue.Queue()
        def read_commands():
            try:
                for command in sys.stdin.buffer: commands.put(command)
            finally:
                commands.put(None)
        threading.Thread(target=read_commands,daemon=True,name='camera-commands').start()
        display = bool(display and not qr and os.environ.get('DISPLAY'))
        last_display = 0.
        first_display = True
        if display:
            from .camera_monitor import RecognitionMonitor
            monitor = RecognitionMonitor()
            cv2.namedWindow('NX Camera 1', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('NX Camera 1',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
            cv2.setMouseCallback('NX Camera 1', monitor.on_mouse)
            source.stream(True)
            def monitor_frames():
                """Keep idle recognition alive; alignment takes perception priority."""
                from .camera_vision import VisionSettings
                after = 0
                while not monitor_stopped.is_set():
                    if tracking is not None:
                        monitor_stopped.wait(.05)
                        continue
                    try:
                        frame_id, mono, _, raw = source.next_frame(after, timeout=.2)
                        after = frame_id
                        config = VisionSettings(Path(__file__).with_name('camera_alignment.json')).snapshot()
                        if tracking is None:
                            monitor.process(raw.copy(), config, mono)
                    except TimeoutError:
                        continue
                    except Exception as error:
                        monitor.failed(error)
                        monitor_stopped.wait(.5)
            monitor_thread = threading.Thread(target=monitor_frames, daemon=True, name='nx-monitor-recognition')
            monitor_thread.start()
        while True:
            if display and time.monotonic()-last_display >= 1/30:
                try:
                    _,_,_,raw = source.next_frame(timeout=.05)
                except TimeoutError:
                    pass
                else:
                    frame = monitor.draw(raw, display_report[0], tracking is not None)
                    cv2.imshow('NX Camera 1',frame)
                    if first_display:
                        first_display = False
                        threading.Thread(target=present_local_window,daemon=True,name='nx-camera-present').start()
                    last_display = time.monotonic()
                key = cv2.waitKey(1) & 0xff
                if key in (ord('1'),ord('2')) and tracking is None:
                    monitor.select('digit' if key == ord('1') else 'color')
                if key in (27,ord('q')):
                    cv2.destroyWindow('NX Camera 1')
                    display = False
                    monitor_stopped.set()
                    source.stream(tracking is not None)
            try: command = commands.get(timeout=.01 if display else .1)
            except queue.Empty: continue
            if command is None: break
            if command.startswith(b'TUNE '):
                request = json.loads(command[5:])
                try:
                    if qr or tracking:
                        raise ValueError('摄像头正在执行其他识别任务。')
                    import base64
                    from .vision_tuning import preview
                    _, _, wall, raw = source.next_frame(shutter=True)
                    report, annotated, mask = preview(raw.copy(), request['config'], wall)
                    images = {}
                    for name, img in (('image', annotated), ('mask', mask)):
                        if img.shape[1] > 960:
                            img = cv2.resize(img, (960, max(1, round(img.shape[0]*960/img.shape[1]))))
                        ok, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if not ok: raise ValueError('调参图像编码失败。')
                        images[name] = 'data:image/jpeg;base64,'+base64.b64encode(encoded).decode('ascii')
                    result = dict(report=report, **images)
                except Exception as error:
                    result = dict(error=str(error))
                emit(dict(tuning_only=True, request_id=request['request_id']),
                     json.dumps(result, ensure_ascii=True).encode('ascii'))
                continue
            if command.startswith(b'TRACK '):
                if qr: continue
                request = json.loads(command[6:])
                if tracking: tracking[1].set()
                display_report[0] = None
                stopped = threading.Event()
                tracking = (request['stream_id'], stopped)
                source.stream(True)
                threading.Thread(target=track, args=(request['config'], request['stream_id'], stopped, request.get('phase')),
                                 daemon=True, name='nx-camera-recognition').start()
                continue
            if command.startswith(b'UNTRACK '):
                if tracking and int(command[8:]) == tracking[0]:
                    tracking[1].set(); tracking = None
                    display_report[0] = None
                    source.stream(display)
                continue
            if command.strip() != b"SNAP":
                continue
            _, _, captured_at, raw = source.next_frame(shutter=True)
            frame = raw.copy()
            actual_height, actual_width = frame.shape[:2]
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("摄像头图像编码失败。")
            data = jpeg.tobytes()
            message = "拍照完成，画面为这一帧，不会持续传输。" if (actual_width, actual_height) == (width, height) else "已拍照，设备实际尺寸与请求不同，请查看实际分辨率。"
            header = {"length": len(data), "width": actual_width, "height": actual_height,
                      "fps": capture.get(cv2.CAP_PROP_FPS), "message": message,
                      "vision": None, "captured_at": captured_at}
            emit(header, data)
    except Exception as error:
        emit({'error': str(error)})
    finally:
        monitor_stopped.set()
        if qr_thread:
            qr_thread.join(timeout=2)
        if monitor_thread:
            monitor_thread.join(timeout=5)
        if monitor and (not monitor_thread or not monitor_thread.is_alive()):
            monitor.close()
        if display:
            cv2.destroyAllWindows()
        if tracking: tracking[1].set()
        if source:
            source.close()
        elif capture:
            capture.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--mjpeg", action="store_true")
    parser.add_argument("--sensor-id", type=int)
    parser.add_argument("--display", action='store_true')
    parser.add_argument("--qr", action='store_true')
    args = parser.parse_args()
    worker(args.worker, args.width, args.height, args.mjpeg, args.sensor_id, args.display, args.qr)
