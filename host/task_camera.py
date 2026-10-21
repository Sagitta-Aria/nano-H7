"""Serialized camera handoff for the NX mission, off the serial/control pump.

Only remembered identities are opened. Cancellation invalidates pending work;
an in-flight opening is closed before the worker can publish a ready result.
"""
import threading
import time


class TaskCameras:
    """One lazy worker changes camera ownership; all public calls are nonblocking."""
    def __init__(self, slots):
        self.slots = slots
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.thread = None
        self.closed = False
        self.generation = 0
        self.requested = None
        self.result = dict(state='idle', slot=None, error='')

    def request(self, slot, *, timeout=10):
        """Switch bound cameras off-pump; B may wait until ready/error/cancellation."""
        if slot not in ('1', '2'):
            raise ValueError('摄像头栏目无效。')
        with self.lock:
            if self.closed:
                raise ValueError('相机任务控制已关闭。')
            self.generation += 1
            token = self.generation
            self.requested = (token, slot, timeout, False)
            self.result = dict(state='opening', slot=slot, error='', token=token)
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, daemon=True, name='nx-task-camera-switch')
                self.thread.start()
            self.changed.set()
            return token

    def release(self, slot):
        """Close the completed route's stream off-pump; never open another slot."""
        if slot not in ('1', '2'):
            raise ValueError('摄像头栏目无效。')
        with self.lock:
            if self.closed:
                raise ValueError('相机任务控制已关闭。')
            self.generation += 1
            token = self.generation
            self.requested = (token, slot, None, True)
            self.result = dict(state='closing', slot=slot, error='', token=token)
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, daemon=True, name='nx-task-camera-switch')
                self.thread.start()
            self.changed.set()
            return token

    def snapshot(self):
        """Copy transition state without device I/O or joining a worker."""
        with self.lock:
            return dict(self.result)

    def cancel(self):
        """Invalidate a pending transition; settled cameras retain their state."""
        with self.lock:
            self.generation += 1
            self.requested = None
            self.result = dict(state='idle', slot=None, error='')
            self.changed.set()

    def close(self):
        """Stop ownership changes before the server closes its camera slots."""
        with self.lock:
            self.closed = True
            self.generation += 1
            self.requested = None
            self.changed.set()
        if self.thread:
            self.thread.join(timeout=10)

    def _current(self, token):
        """Check a generation before/after each potentially blocking device call."""
        with self.lock:
            return not self.closed and self.generation == token

    def _run(self):
        """Consume the latest desired slot; stale requests cannot report success."""
        while True:
            self.changed.wait()
            with self.lock:
                self.changed.clear()
                if self.closed: return
                request, self.requested = self.requested, None
            if request is None: continue
            token, slot, timeout, closing = request
            opened = False
            failed = False
            try:
                if not self._current(token): continue
                if closing:
                    self.slots.stop(slot)
                    with self.lock:
                        if not self.closed and token == self.generation:
                            self.result = dict(state='idle', slot=slot, error='', token=token)
                    continue
                self.slots.stop('2' if slot == '1' else '1')
                if not self._current(token): continue
                listing = self.slots.listing(slot)
                if not listing['binding']:
                    raise ValueError('摄像头{}尚未记住固定ID，请接入并绑定后再运行。'.format(slot))
                device = next((d for d in listing['devices'] if d['id'] == listing['bound_device']), None)
                if device is None:
                    raise ValueError('已绑定的摄像头{}未连接。'.format(slot))
                if not self._current(token): continue
                hub = self.slots.hubs[slot]
                state = hub.snapshot()
                if state['state'] != 'ready' or state.get('device') != device['id']:
                    modes = device['resolutions']
                    self.slots.start(slot, device['id'], '1280x720' if '1280x720' in modes else modes[-1])
                    opened = True
                deadline = None if timeout is None else time.monotonic()+timeout
                while self._current(token):
                    state = hub.snapshot()
                    if state['state'] == 'ready': break
                    if state['state'] == 'error': raise ValueError(state.get('message', '相机打开失败。'))
                    if deadline is not None and time.monotonic() >= deadline:
                        raise ValueError('摄像头{}打开超时。'.format(slot))
                    self.changed.wait(.05)
                with self.lock:
                    if not self.closed and token == self.generation:
                        self.result = dict(state='ready', slot=slot, error='', token=token)
            except Exception as error:
                failed = True
                with self.lock:
                    if not self.closed and token == self.generation:
                        self.result = dict(state='error', slot=slot, error=str(error), token=token)
            finally:
                if opened and (failed or not self._current(token)):
                    self.slots.stop(slot)
