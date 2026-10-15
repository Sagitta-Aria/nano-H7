"""Optional Nano device discovery; restore links without replaying motion.

Use only with --auto-devices. Metadata scanning opens no arbitrary UARTs;
CH340 H743 candidates require a board/UID handshake, ST CDC uses ImuHub.
The first verified MCU UID is persisted so renumbered ports remain usable and
different boards cannot silently replace the paired devices. No auto ARM.
"""
import json
from pathlib import Path
import threading
import time

from .imu import usb_ports

CONFIG_PATH = Path(__file__).with_name('device_links.json')
H743_USB_IDS = {(0x1A86, 0x7523), (0x1A86, 0x55D4)}


class AutoDevices:
    """Nonblocking discovery driven by the server pump, independent of browsers.

    Serial I/O remains in existing workers. A two-second cadence retries forever
    and rotates candidates; only verified identities are written to settings.
    Disabling stops future attempts but leaves established connections intact.
    """
    def __init__(self, console, path=CONFIG_PATH, port_provider=usb_ports, interval=2.):
        self.console, self.path = console, Path(path)
        self.port_provider, self.interval = port_provider, interval
        self.lock = threading.RLock()
        self.config = {'enabled': True, 'h743_uid': None, 'h723_uid': None}
        if self.path.exists():
            self.config.update(json.loads(self.path.read_text(encoding='utf-8')))
        self.next_scan = 0.
        self.cursor = {'h743': 0, 'h723': 0}
        self.message = '自动连接已启用；等待 H743 / H723'

    def _save(self):
        """Atomically persist pairing; never modify motor/navigation settings."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.config, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.path)

    def set_enabled(self, enabled):
        """Change discovery policy; disabling does not stop connected hardware."""
        with self.lock:
            self.config['enabled'] = bool(enabled)
            self.next_scan = 0.
            self._save()
            self.message = '自动连接已启用' if enabled else '自动连接已暂停'
        self.console.note('info', self.message)

    def snapshot(self):
        """Publish policy/status without exposing unverified identities."""
        with self.lock:
            return {'enabled': self.config['enabled'], 'message': self.message,
                    'h743_uid': self.config['h743_uid'], 'h723_uid': self.config['h723_uid']}

    def _remember(self, kind, uid):
        """Bind only the first handshake-verified UID, including across reboot."""
        if uid and not self.config[kind + '_uid']:
            self.config[kind + '_uid'] = uid.upper()
            self._save()

    def tick(self, now=None):
        """Enumerate/retry without sleeping, opening devices, or blocking the pump."""
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.config['enabled'] or now < self.next_scan:
                return
            self.next_scan = now + self.interval
            console = self.console
            with console.lock:
                worker = console.connection
                h743_ready = bool(worker and console.device.get('_serial') and
                                  console.device.get('board') == 'H743_CB_V1.5')
                if h743_ready:
                    self._remember('h743', console.device.get('uid'))
                h743_busy = worker is not None
            imu = console.imu
            frame = imu.snapshot()
            if frame.get('identity'):
                self._remember('h723', frame['identity'].get('uid'))
            with imu.lock:
                h723_busy = bool(imu.thread and imu.thread.is_alive())
            ports = self.port_provider()
            if not h743_busy:
                candidates = [p for p in ports if (p.vid, p.pid) in H743_USB_IDS]
                if candidates:
                    index = self.cursor['h743'] % len(candidates)
                    self.cursor['h743'] += 1
                    console.connect(candidates[index].device, expected_board='H743_CB_V1.5',
                                    expected_uid=self.config['h743_uid'])
            if not h723_busy:
                candidates = [p for p in imu.ports()['ports'] if p.get('serial_number')]
                if candidates:
                    index = self.cursor['h723'] % len(candidates)
                    self.cursor['h723'] += 1
                    imu.expected_uid = self.config['h723_uid']
                    console.connect_imu(candidates[index]['id'])
            h743_status = '已连接' if h743_ready else '等待设备 / 重试握手'
            h723_status = '已连接' if frame['state'] in ('streaming', 'calibrating', 'invalid', 'stale') else '等待设备 / 重试握手'
            message = '自动连接：H743 ' + h743_status + ' · H723 ' + h723_status
            if message != self.message:
                self.message = message
                console.note('info', message)
