"""Exercise the real H723 Tk tab with synthetic USB only, without opening COM."""
import time
import tkinter as tk

from host.gui import Console
from host.imu import ImuHub
from tests.gui_smoke import wait
from tests.test_imu import CANDIDATE, UsbFixture


def main():
    """Verify handshake/readouts, coordinate drawing and invalidation on stream loss."""
    root = tk.Tk()
    root.withdraw()
    app = Console(root)
    panel = app.imu
    device = UsbFixture()
    panel.hub.close()
    panel.hub = ImuHub(port_provider=lambda: [dict(CANDIDATE)], serial_factory=lambda p: device)
    try:
        panel.refresh()
        assert panel.port.get() == "", "no automatic port selection"
        panel.port.set(next(iter(panel.ports)))
        app.notebook.select(panel.parent)
        root.update()
        panel.connect()
        wait(root, lambda: panel.frame["state"] == "streaming")
        assert "STM32H723VGT6" in panel.identity.get()
        assert "X 7.000" in panel.readouts.get()
        assert "Y 8.000" in panel.readouts.get()
        panel.mode.set("加速度计（含重力）")
        panel.draw()
        assert "X 4.000" in panel.readouts.get()
        assert "Y 5.000" in panel.readouts.get()
        for canvas in panel.charts:
            assert any(canvas.type(item) == "text" for item in canvas.find_all())
            assert any(canvas.type(item) == "line" for item in canvas.find_all())
        device.silent = True
        wait(root, lambda: panel.frame["state"] == "stale")
        assert "X —" in panel.readouts.get()
        assert panel.points, "retain history after stream loss"
        panel.close()
        wait(root, lambda: panel.frame["state"] == "offline")
        assert all(data.startswith(b"IMU? ") for data in device.writes)
        print("H723 Tk smoke OK: explicit selection, handshake, accel modes, axes and stale invalidation.")
    finally:
        panel.close()
        root.destroy()


if __name__ == "__main__":
    main()
