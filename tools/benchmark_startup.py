"""Measure click-to-motion through the real host queue with a simulated UART.

No physical ports/cameras are opened and all settings live in a temporary folder.
--source can point to an extracted release for a comparable before/after run.
The UART model charges 10 bits/byte at 115200 and pyserial's 20 ms read timeout;
it excludes MCU compute, USB scheduling, IMU calibration and camera cold start.
"""
import argparse
import json
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--entry', choices=('competition', 'navigation'), default='competition')
    args = parser.parse_args()
    sys.path.insert(0, str(args.source.resolve()))
    from host.protocol import decode
    from host.simulator import SimulatedSerial
    from host.webapp import Console

    class WireSerial(SimulatedSerial):
        """Deliver complete responses after their wire cost, with pyserial reads."""
        def __init__(self):
            super().__init__()
            self.available_at = 0.
            self.records = []

        @property
        def in_waiting(self):
            return len(self.buffer) if time.monotonic() >= self.available_at else 0

        def write(self, data):
            before = len(self.buffer)
            result = super().write(data)
            self.available_at = time.monotonic() + (len(data)+len(self.buffer)-before)*10/115200
            self.records.append((time.monotonic(), decode(data).body))
            return result

        def read(self, size=256):
            deadline = time.monotonic()+.02
            if size > len(self.buffer):
                time.sleep(.02)
            else:
                time.sleep(max(0., min(deadline, self.available_at)-time.monotonic()))
            return super().read(size) if time.monotonic() >= self.available_at else b''

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        stream = WireSerial()
        console = Console(root/'navigation.json', root/'auxiliary.json', root/'flow.json')
        def pump():
            if hasattr(console, 'wakeup'): console.wakeup.clear()
            console.drain(); console.tick()
            if hasattr(console, 'wakeup'): console.wakeup.wait(.01)
            else: time.sleep(.05)

        with patch('host.transport.SimulatedSerial', return_value=stream):
            console.connect('SIM')
            connection = console.connection
            try:
                deadline = time.monotonic()+10
                while not console.device.get('_serial') or console.settings_pending or not connection.commands.empty():
                    if time.monotonic() > deadline: raise RuntimeError('SIM startup timed out')
                    pump()
                if args.entry == 'navigation':
                    for command in ('NAV RESET 0 0', 'ARM 1', 'NAV STAT', 'TURN STAT', 'NAV ALIGN'):
                        console.submit(command)
                    deadline = time.monotonic()+5
                    while not console.armed or console.navigation.get('ready') != '1' or not console.nav_align:
                        if time.monotonic() > deadline: raise RuntimeError('SIM preparation timed out')
                        pump()
                begin = time.monotonic()
                ok, reason = console.competition.start() if args.entry == 'competition' else console.navigation_action('start')
                if not ok: raise RuntimeError(reason)
                while not any(axis['active'] for axis in stream.board.axes[:4]):
                    if time.monotonic()-begin > 15: raise RuntimeError(console.hello())
                    pump()
                records = [(round((stamp-begin)*1000, 1), command) for stamp, command in stream.records if stamp >= begin]
                report = dict(source=str(args.source), entry=args.entry, modeled_baud=115200,
                              click_to_motion_ms=round((time.monotonic()-begin)*1000, 1),
                              commands_before_motion=len(records), commands=records)
                print(json.dumps(report, ensure_ascii=True, indent=2))
            finally:
                console.disconnect(); connection.close(); connection.join(2)


if __name__ == '__main__':
    main()
