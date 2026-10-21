"""Timestamped vision mailbox adapter; never owns a camera or serial port.

Use inside an existing camera loop. request() is a cached network snapshot and
publish() queues only the newest result, so HTTP latency cannot block capture.
The producer must supply capture time from before inference, never delivery time.
"""
import copy
import json
import threading
import urllib.request
import uuid


def sample_for(request, report, source, frame_id, captured_at):
    """Match actual detector identity; never label another object as the target.

    report follows nx_vision_live.report(), or {qr_text: str} for QR. The caller
    must reset detector votes on each new request and discard pre-request work.
    """
    if not request:
        return None
    sample = {k: request[k] for k in ('run_id', 'request_id', 'kind', 'target')}
    sample.update(source=source, frame_id=frame_id, captured_at=captured_at, found=False)
    kind, target = request['kind'], request['target']
    if kind == 'qr':
        text = report.get('qr_text')
        if isinstance(text, str) and text:
            sample.update(found=True, text=text)
        return sample
    if kind == 'block':
        detected = report.get('color_block') if report.get('mode') == 'color' else None
        matched = detected and detected.get('color') == target
    else:
        detected = report
        matched = (report.get('mode') == 'digit' and report.get('ring') is True
                   and report.get('digit') is not None and str(report['digit']) == target)
    if matched and detected.get('error_dx') is not None and detected.get('error_dy') is not None:
        sample.update(found=True, error_dx=detected['error_dx'], error_dy=detected['error_dy'])
    return sample


class VisionPublisher:
    """Single latest-result HTTP worker; close it when the camera loop exits.

    Default is same-host loopback. Remote deployments need a controller session
    token and synchronised clocks. Network failures drop results, never replay.
    Each instance stamps its own session_id; recreate it when frame numbering
    restarts, retain it across ordinary network disconnects.
    """
    def __init__(self, url='http://127.0.0.1:8770', token=None):
        self.url = url.rstrip('/')
        self.session_id = uuid.uuid4().hex
        self.headers = {'Content-Type': 'application/json'}
        if token:
            self.headers['X-Session'] = token
        self.lock = threading.Lock()
        self.current = self.latest = None
        self.error = ''
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self._run, name='mission-vision', daemon=True)
        self.worker.start()

    def request(self):
        """Return a detached cached request; no I/O occurs on the camera thread."""
        with self.lock:
            return copy.deepcopy(self.current)

    def publish(self, sample):
        """Replace any unsent frame; old captures cannot accumulate in a queue."""
        with self.lock:
            self.latest = copy.deepcopy(sample)
            if self.latest is not None:
                self.latest['session_id'] = self.session_id

    def _json(self, path, payload=None):
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        req = urllib.request.Request(self.url + path, data=data, headers=self.headers)
        with urllib.request.urlopen(req, timeout=.5) as response:
            return json.load(response)

    def _run(self):
        """Refresh requests first, then send at most one matching result per pass."""
        while not self.closed.is_set():
            try:
                request = self._json('/api/mission/request')['request']
                with self.lock:
                    self.current = request
                    sample, self.latest = self.latest, None
                if sample and request and all(sample.get(k) == request[k] for k in ('run_id', 'request_id')):
                    self._json('/api/mission/vision', {'sample': sample})
                self.error = ''
            except (OSError, ValueError, KeyError) as error:
                with self.lock:
                    self.current = self.latest = None
                self.error = str(error)
            self.closed.wait(.05)

    def close(self):
        """Stop network work; no motor command or camera operation is issued."""
        self.closed.set()
        self.worker.join(timeout=1.2)
