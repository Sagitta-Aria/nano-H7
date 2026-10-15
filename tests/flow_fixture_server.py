"""Isolated SIM-only HTTP fixture for browser QA; never opens hardware ports."""
import tempfile
import threading
from pathlib import Path
from http.server import ThreadingHTTPServer
from host.webapp import Console
from host.webserver import Session, Handler

if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        console = Console(root/"nav.json", root/"aux.json", root/"flow.json")
        session = Session(console)
        handler = type("FlowHandler", (Handler,), {"session": session})
        server = ThreadingHTTPServer(("127.0.0.1", 8786), handler)
        server.daemon_threads = True
        console.connect("SIM")
        threading.Thread(target=session.run, daemon=True).start()
        print("Isolated optical flow demo: http://127.0.0.1:8786/", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            session.stopping.set(); console.disconnect(); console.imu.close()
            session.cameras.close(); server.server_close()
