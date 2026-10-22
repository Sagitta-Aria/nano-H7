"""Touch launcher for the two-round material flow, shown full-screen on the NX panel.

The panel boots straight into this window: pick O or X as the cruise start, then
one button starts the saved run with its saved parameters. It only talks to the
local HTTP API, so a launcher fault can never touch the serial link or move the
robot by itself.

The panel is an 800x480 canvas with fixed coordinates: NX has one screen size,
and absolute placement keeps the touch targets stable no matter which CJK font
the box happens to provide.
"""
import argparse
import json
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid

import tkinter as tk
from tkinter import font as tkfont

# Local self-check only: without DPI awareness a scaled Windows desktop reports
# 800x480 while the capture is taken in larger physical pixels. The NX panel is
# a fixed 800x480 screen, so this never affects the deployed path.
if sys.platform == 'win32':
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

DEFAULT_URL = 'http://127.0.0.1:8770'
FONT_FAMILIES = ('Noto Sans CJK SC', 'Noto Sans CJK', 'WenQuanYi Zen Hei',
                 'Source Han Sans SC', 'Microsoft YaHei', 'SimHei')

WIDTH, HEIGHT = 800, 480
BACKGROUND = '#0b0f14'
CARD = '#161d26'
TEXT = '#e8eef6'
MUTED = '#8fa3b8'
GREEN = '#1d7f5f'
RED = '#8f2c2c'
RED_OFF = '#243039'
GREY = '#2a3440'
SELECT = '#1f4b6e'

ORIGIN_BOX = {'O': (150, 76, 300, 156), 'X': (312, 76, 462, 156)}
START_BOX = (24, 264, 560, 424)
STOP_BOX = (576, 264, 776, 424)

PHASES = {
    'idle': '等待启动',
    'stopped': '已停止',
    'connecting': '正在连接设备',
    'preparing': '正在恢复参数并建立起点',
    'running': '路线执行中',
    'done': '流程完成',
    'fault': '启动或执行失败',
}

MATERIAL_PHASES = {
    'material_handoff': '到站交接',
    'material_align': '视觉校准',
    'material_camera': '等待摄像头',
    'material_move': '点间移动',
    'material_cycle': '抓取与放置',
    'material_return': '回位到站',
}


def api(base, path, payload=None, timeout=4.):
    """One request against the local console; raises on transport errors.

    A refused mutation (4xx) returns the console's JSON body instead of raising
    so the panel can show the reason, for example a second controller.
    """
    data = None if payload is None else json.dumps(payload).encode('utf-8')
    request = urllib.request.Request(base + path, data=data,
                                     headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode('utf-8'))
    except urllib.error.HTTPError as error:
        try:
            return json.loads(error.read().decode('utf-8'))
        except Exception:
            return {'ok': False, 'error': '上位机返回 {}'.format(error.code)}


def station_name(state):
    mission = state.get('mission') or {}
    stations = mission.get('stations') or []
    index = mission.get('start_station') or 0
    if 0 <= index < len(stations):
        return stations[index].get('name', '?')
    return '?'


def x_ready(state):
    """X can only be started when it is configured and the firmware declares it."""
    device = state.get('device') or {}
    mission = state.get('mission') or {}
    if device.get('nav_reference') != '1':
        return False, 'X 起点需要 0.2.29 固件（INFO nav_reference=1）'
    if not any(str(s.get('name', '')).upper() == 'X' for s in mission.get('stations') or []):
        return False, '巡航配置里还没有 X 点'
    return True, 'X 位于 (0,28800)、车头朝 -180°；先内移，再转 0° 直达 A'


def vehicle_line(state):
    if state.get('offline'):
        return '上位机未响应'
    device = state.get('device') or {}
    if state.get('link') != 'connected':
        return '设备未连接'
    armed = '已使能' if state.get('armed') else '未使能'
    return 'H743 已连接 · {} · 固件 {}'.format(armed, device.get('fw') or '?')


def phase_line(state):
    if state.get('offline'):
        return '等待上位机…'
    competition = state.get('competition') or {}
    task = state.get('task') or {}
    state_name = competition.get('state') or 'idle'
    text = PHASES.get(state_name, state_name)
    if state_name == 'running':
        material = task.get('state') or ''
        if material in MATERIAL_PHASES:
            text += '：' + MATERIAL_PHASES[material]
        elif material:
            text += '：' + material
    return text


def error_line(state):
    if state.get('offline'):
        return ''
    competition = state.get('competition') or {}
    task = state.get('task') or {}
    return competition.get('error') or task.get('error') or ''


def running(state):
    competition = state.get('competition') or {}
    return competition.get('state') in ('connecting', 'preparing', 'running')


def start_payload(choice):
    """Only O/X are launchable; anything else is rejected before the request."""
    name = (choice or 'O').strip().upper()
    if name not in ('O', 'X'):
        raise ValueError('起点只能是 O 或 X。')
    return {'full_flow': True, 'start_station': name}


class Launcher:
    """Fixed 800x480 canvas panel; polling and requests stay off the GUI thread."""

    def __init__(self, url=DEFAULT_URL, fullscreen=True, snapshot=None):
        self.url = url
        self.choice = 'O'
        self.state = {}
        self.message = ''
        self._busy = None
        self.token = uuid.uuid4().hex
        self.stop = threading.Event()
        self.root = tk.Tk()
        self.root.title('两轮搬运 · 一键启动')
        self.root.configure(bg=BACKGROUND)
        self.root.geometry('{}x{}'.format(WIDTH, HEIGHT))
        self.root.resizable(False, False)
        if fullscreen:
            self.root.attributes('-fullscreen', True)
        self.root.attributes('-topmost', True)
        self.family = self._family()
        self._build()
        self.stream = None
        threading.Thread(target=self._poll, name='launcher-poll', daemon=True).start()
        threading.Thread(target=self._session, name='launcher-session', daemon=True).start()
        self.root.after(400, self._tick)
        if snapshot:
            self.root.after(1500, lambda: self._snapshot(snapshot))

    def _family(self):
        available = set(tkfont.families(self.root))
        for family in FONT_FAMILIES:
            if family in available:
                return family
        return 'TkDefaultFont'

    def _font(self, size, weight='normal'):
        return (self.family, size, weight)

    def _build(self):
        canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT, bg=BACKGROUND,
                           highlightthickness=0)
        canvas.pack(fill='both', expand=True)
        self.canvas = canvas
        canvas.create_text(24, 30, text='两轮搬运', anchor='w', fill=TEXT,
                           font=self._font(26, 'bold'))
        canvas.create_text(300, 36, text='一键启动', anchor='w', fill=MUTED,
                           font=self._font(15))
        self.vehicle_id = canvas.create_text(776, 34, text='', anchor='e', fill=MUTED,
                                             font=self._font(11))
        canvas.create_text(24, 116, text='巡航起点', anchor='w', fill=MUTED,
                           font=self._font(14))
        self.origin_items = {}
        for name in ('O', 'X'):
            x0, y0, x1, y1 = ORIGIN_BOX[name]
            box = canvas.create_rectangle(x0, y0, x1, y1, fill=GREY, outline='')
            label = canvas.create_text((x0 + x1) // 2, (y0 + y1) // 2,
                                       text='{} 点'.format(name), fill=TEXT,
                                       font=self._font(20, 'bold'))
            self.origin_items[name] = (box, label)
        self.hint_id = canvas.create_text(24, 172, text='', anchor='nw', fill=MUTED,
                                          font=self._font(11), width=720)
        self.start_box = canvas.create_rectangle(*START_BOX, fill=GREEN, outline='')
        self.start_label = canvas.create_text((START_BOX[0] + START_BOX[2]) // 2,
                                              (START_BOX[1] + START_BOX[3]) // 2,
                                              text='开 始', fill='white',
                                              font=self._font(42, 'bold'))
        self.stop_box = canvas.create_rectangle(*STOP_BOX, fill=RED_OFF, outline='')
        self.stop_label = canvas.create_text((STOP_BOX[0] + STOP_BOX[2]) // 2,
                                             (STOP_BOX[1] + STOP_BOX[3]) // 2,
                                             text='停 止', fill=MUTED,
                                             font=self._font(24, 'bold'))
        canvas.create_rectangle(0, 434, WIDTH, HEIGHT, fill=CARD, outline='')
        self.phase_id = canvas.create_text(24, 448, text='', anchor='w', fill=TEXT,
                                           font=self._font(12))
        self.message_id = canvas.create_text(24, 466, text='', anchor='w', fill=MUTED,
                                             font=self._font(9), width=752)
        canvas.tag_bind(self.start_box, '<Button-1>', lambda event: self._start())
        canvas.tag_bind(self.start_label, '<Button-1>', lambda event: self._start())
        canvas.tag_bind(self.stop_box, '<Button-1>', lambda event: self._stop())
        canvas.tag_bind(self.stop_label, '<Button-1>', lambda event: self._stop())
        for name in ('O', 'X'):
            for item in self.origin_items[name]:
                canvas.tag_bind(item, '<Button-1>', lambda event, n=name: self._choose(n))

    # ---- actions ---------------------------------------------------------
    def _choose(self, name):
        if running(self.state):
            return
        ready, why = x_ready(self.state) if name == 'X' else (True, '')
        if not ready:
            self.message = why
            self._refresh()
            return
        self.choice = name
        self.message = ''
        self._refresh()

    def _start(self):
        if running(self.state):
            return
        try:
            payload = start_payload(self.choice)
        except ValueError as error:
            self.message = str(error)
            self._refresh()
            return
        self.message = '正在启动：{} 点出发…'.format(self.choice)
        self._refresh()

        def work():
            try:
                result = self._mutate('/api/competition/start', payload)
                self.message = result.get('detail') or result.get('error') or ''
            except Exception as error:
                self.message = '启动失败：' + str(error)
            self._refresh()
        threading.Thread(target=work, name='launcher-start', daemon=True).start()

    def _stop(self):
        if not running(self.state):
            return
        self.message = '正在停止…'
        self._refresh()

        def work():
            try:
                result = self._mutate('/api/command', {'body': 'STOP'})
                if not result.get('ok'):
                    self.message = result.get('error') or '停止被拒绝。'
                else:
                    self._mutate('/api/mission/action', {'action': 'stop'})
                    self.message = '已发送停止。'
            except Exception as error:
                self.message = '停止请求失败：' + str(error)
            self._refresh()
        threading.Thread(target=work, name='launcher-stop', daemon=True).start()

    # ---- polling ---------------------------------------------------------
    def _session(self):
        """Join the console as a page and keep its control lease fresh.

        The panel is the standing operator interface, so it registers once and
        renews the lease; a browser debug page still can take control while the
        robot is idle, and the panel re-claims on its next press.
        """
        while not self.stop.is_set():
            try:
                if self.stream is None:
                    self.stream = urllib.request.urlopen(
                        self.url + '/api/events?token=' + self.token, timeout=8.)
                    api(self.url, '/api/session/claim', {'token': self.token}, timeout=4.)
                else:
                    api(self.url, '/api/session/heartbeat', {'token': self.token}, timeout=2.)
            except Exception:
                try:
                    if self.stream is not None:
                        self.stream.close()
                except Exception:
                    pass
                self.stream = None
            time.sleep(1.5)

    def _mutate(self, path, payload):
        """Send a mutation as the panel session, reclaiming control once if needed."""
        data = dict(payload, token=self.token)
        result = api(self.url, path, data)
        if not result.get('ok') and '正在控制' in (result.get('error') or ''):
            claim = api(self.url, '/api/session/claim', {'token': self.token})
            if not claim.get('ok'):
                return claim
            result = api(self.url, path, data)
        return result

    def _poll(self):
        while not self.stop.is_set():
            try:
                self.state = api(self.url, '/api/state', timeout=2.)
            except Exception:
                self.state = {'offline': True}
            time.sleep(.5)

    def _tick(self):
        self._refresh()
        self.root.after(400, self._tick)

    def _refresh(self):
        state = self.state
        busy = running(state)
        canvas = self.canvas
        canvas.itemconfigure(self.vehicle_id, text=vehicle_line(state))
        canvas.itemconfigure(self.phase_id, text=phase_line(state))
        error = error_line(state)
        canvas.itemconfigure(self.message_id, text=error or self.message,
                             fill='#ff9a9a' if error else MUTED)
        for name, (box, label) in self.origin_items.items():
            ready, _ = x_ready(state) if name == 'X' else (True, '')
            chosen = name == self.choice
            canvas.itemconfigure(box, fill=SELECT if chosen and ready else GREY)
            canvas.itemconfigure(label, fill=TEXT if ready else MUTED)
        _, why = x_ready(state) if self.choice == 'X' else (True, '')
        canvas.itemconfigure(
            self.hint_id,
            text=why if self.choice == 'X' else
            'O 位于地图原点、朝向 0°。按“开始”自动连接、恢复参数、建立起点、使能并跑两轮搬运。')
        canvas.itemconfigure(self.start_box, fill=GREY if busy else GREEN)
        canvas.itemconfigure(self.start_label, fill=MUTED if busy else 'white')
        canvas.itemconfigure(self.stop_box, fill=RED if busy else RED_OFF)
        canvas.itemconfigure(self.stop_label, fill='white' if busy else MUTED)
        if busy != self._busy:
            # 运行期间让出最前层：摄像头预览和任务码全屏卡必须看得见。
            # 空闲时再回到最前，保证开机/结束都停在启动页。
            self._busy = busy
            self.root.attributes('-topmost', not busy)
            if busy:
                self.root.lower()
            else:
                self.root.lift()

    def _snapshot(self, path):
        from PIL import ImageGrab
        self.root.update_idletasks()
        self.root.update()
        x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
        width, height = self.root.winfo_width(), self.root.winfo_height()
        ImageGrab.grab(bbox=(x, y, x + width, y + height)).save(path)
        print('snapshot saved:', path, width, height)
        self.close()

    def close(self):
        self.stop.set()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        self.root.mainloop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default=DEFAULT_URL, help='local console base URL')
    parser.add_argument('--windowed', action='store_true', help='do not go full screen')
    parser.add_argument('--snapshot', help='save a screenshot and exit (self check)')
    args = parser.parse_args(argv)
    Launcher(url=args.url, fullscreen=not args.windowed, snapshot=args.snapshot).run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
