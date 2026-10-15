"""Linux USB capture isolated from robot I/O in a terminable subprocess.

Only explicit page actions open devices. OpenCV is imported by the worker,
never by the server; driver/dependency failures cannot stop board control.
"""
import argparse
import json
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
        self.hubs = {"1": CameraHub(), "2": CameraHub()}
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
    """Own one shared camera and one photo; there is no continuous preview."""
    def __init__(self):
        self.lock = threading.RLock()
        self.process = None
        self.jpeg = None
        self.frame = {"state": "idle", "message": "尚未打开 Nano 摄像头。"}
        self.sequence = 0
        self.frame_time = 0.
        self.changed = threading.Condition(self.lock)
        self.capturing = False

    def snapshot(self):
        """Copy state quickly without doing device I/O on the serial pump."""
        with self.lock:
            state = dict(self.frame)
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
                process.stdin.write(b"SNAP\n")
                process.stdin.flush()
                self.changed.wait_for(lambda: self.sequence != sequence or self.process is not process, timeout=8)
                if self.process is not process:
                    raise ValueError("摄像头已关闭或断开，请重新打开。")
                if self.sequence == sequence:
                    raise ValueError("拍照超时，请关闭摄像头后重新打开。")
                return self.snapshot()
            finally:
                self.capturing = False

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
            command = [sys.executable, "-u", "-m", "host.camera", "--worker", device,
                       "--width", width, "--height", height]
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
            process, self.process = self.process, None
            self.jpeg = None
            self.frame = {"state": "idle", "message": "Nano 摄像头已关闭，设备已释放。"}
            self.changed.notify_all()
        if process:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)

    def _read(self, process):
        """Consume length-framed JPEGs; old workers cannot overwrite new state."""
        error = "Nano 摄像头已断开或无法读取，请检查 摄像头连接。"
        try:
            while True:
                line = process.stdout.readline(4096)
                if not line:
                    break
                header = json.loads(line)
                if header.get("error"):
                    error = header["error"]
                    break
                length = header.pop("length")
                if not 0 <= length <= 16 * 1024 * 1024:
                    raise ValueError("invalid camera frame length")
                data = process.stdout.read(length)
                if len(data) != length:
                    break
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
                process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
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


def worker(device, width, height, mjpeg, sensor_id=None):
    """Hold a V4L2 device, wait for a shutter command, and encode only one JPEG."""
    capture = None
    try:
        import cv2
        cv2.setNumThreads(1)
        capture = CsiCapture(sensor_id, width, height) if sensor_id is not None else cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError("无法打开 Nano 摄像头，请检查权限或是否被其他程序占用。")
        if mjpeg and sensor_id is None:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if sensor_id is None:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, 30)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ready = {"length": 0, "width": actual_width, "height": actual_height,
                 "fps": capture.get(cv2.CAP_PROP_FPS), "message": "Nano 摄像头已打开，点击“拍照”获取一帧。"}
        sys.stdout.buffer.write(json.dumps(ready, ensure_ascii=True).encode() + b"\n")
        sys.stdout.buffer.flush()
        for command in sys.stdin.buffer:
            if command.strip() != b"SNAP":
                continue
            # Discard queued driver buffers so a shutter after a long idle does
            # not return the old frame from when the camera was first opened.
            for _ in range(4):
                if not capture.grab():
                    raise RuntimeError("Nano 摄像头已断开或采集失败，请重新打开。")
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("Nano 摄像头已断开或采集失败，请重新打开。")
            actual_height, actual_width = frame.shape[:2]
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise RuntimeError("摄像头图像编码失败。")
            data = jpeg.tobytes()
            message = "拍照完成，画面为这一帧，不会持续传输。" if (actual_width, actual_height) == (width, height) else "已拍照，设备实际尺寸与请求不同，请查看实际分辨率。"
            header = {"length": len(data), "width": actual_width, "height": actual_height,
                      "fps": capture.get(cv2.CAP_PROP_FPS), "message": message}
            sys.stdout.buffer.write(json.dumps(header, ensure_ascii=True).encode() + b"\n" + data)
            sys.stdout.buffer.flush()
    except Exception as error:
        sys.stdout.buffer.write(json.dumps({"error": str(error)}, ensure_ascii=True).encode() + b"\n")
        sys.stdout.buffer.flush()
    finally:
        if capture:
            capture.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--mjpeg", action="store_true")
    parser.add_argument("--sensor-id", type=int)
    args = parser.parse_args()
    worker(args.worker, args.width, args.height, args.mjpeg, args.sensor_id)
