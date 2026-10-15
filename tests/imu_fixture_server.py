"""Loopback-only browser fixture: simulated H743 plus synthetic H723 USB.

Never opens hardware. The _fixture routes exist only in this test executable,
not in the product HTTP handler. Stop this process after browser verification.
"""
import argparse
import math
from pathlib import Path
import sys
import threading
from http.server import ThreadingHTTPServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host.imu import ImuHub
from host.webapp import Console
from host.webserver import Handler, Session
from tests.test_imu import CANDIDATE, UsbFixture, frame


class PlotFixture(UsbFixture):
    """Generate visibly changing readings so plots can be inspected in a browser."""
    def read_until(self, delimiter, size):
        """Reuse challenge handling and change only synthetic measurement fields."""
        data = super().read_until(delimiter, size)
        if data.startswith(b"IMU1,"):
            fields = data.decode().strip().split(",")
            phase = self.count / 30.
            fields[8:11] = [f"{10*math.sin(phase):.3f}", f"{8*math.cos(phase):.3f}", f"{self.count*.04:.3f}"]
            fields[17:20] = [f"{.8*math.sin(phase):.4f}", f"{.6*math.cos(phase):.4f}", "0.0200"]
            return (",".join(fields) + "\r\n").encode()
        return data


def main():
    """Start synthetic devices and expose only an explicit local test server."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18774)
    args = parser.parse_args()
    device = PlotFixture()
    console = Console()
    console.imu = ImuHub(port_provider=lambda: [dict(CANDIDATE)], serial_factory=lambda p: device)
    console.connect("SIM")
    session = Session(console)

    class FixtureHandler(Handler):
        """Test-only controls let the browser exercise stream failure visually."""
        def do_POST(self):
            """Handle explicit fixture changes, then delegate product API routes."""
            if self.path == "/_fixture/silent":
                self.body()
                device.silent = True
                self.json({"ok": True})
            elif self.path == "/_fixture/wrong-board":
                self.body()
                device.silent = False
                device.mcu = "STM32H743IIT6"
                self.json({"ok": True})
            else:
                super().do_POST()

    FixtureHandler.session = session
    server = ThreadingHTTPServer(("127.0.0.1", args.port), FixtureHandler)
    server.daemon_threads = True
    worker = threading.Thread(target=session.run, daemon=True)
    worker.start()
    print("IMU fixture ready", flush=True)
    try:
        server.serve_forever(.1)
    except KeyboardInterrupt:
        pass
    finally:
        session.stopping.set()
        console.imu.close()
        console.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
