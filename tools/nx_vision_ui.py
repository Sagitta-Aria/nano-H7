"""Touch mode controls beside the NX preview, outside camera coordinates.

The panel only queues a requested mode. The live loop owns actual algorithm
switching, so GUI callbacks never wait for inference or touch GPU resources.
"""

from pathlib import Path

import cv2
import numpy as np


class TouchModePanel:
    """Append a 160-pixel touch strip to the unscaled camera preview.

    NX's 640x480 camera plus this strip fills its 800x480 display. Pre-render
    labels once per layout; Chinese text uses the NX system CJK font and Pillow.
    """

    def __init__(self):
        self.width = 160
        self.camera_width = 0
        self.height = 0
        self.panels = {}
        self.buttons = {}
        self.pressed = self.pending = None

    def _build(self, height):
        """Cache both button states; font rendering stays outside the frame loop."""
        from PIL import Image, ImageDraw, ImageFont
        font_path = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
        fonts = {size: ImageFont.truetype(str(font_path), size) for size in (16, 22, 30)}
        self.height = height
        self.buttons = dict(digit=(12, int(height*.28), 148, int(height*.47)),
                            color=(12, int(height*.51), 148, int(height*.70)))
        for selected in ('digit', 'color'):
            panel = Image.new('RGB', (self.width, height), (19, 26, 35))
            draw = ImageDraw.Draw(panel)
            draw.text((80, 38), '识别切换', font=fonts[22], anchor='mm', fill=(240, 245, 250))
            draw.text((80, 76), '点击选择模式', font=fonts[16], anchor='mm', fill=(166, 180, 197))
            for mode, label in (('digit', '数字'), ('color', '色块')):
                active = mode == selected
                x0, y0, x1, y1 = self.buttons[mode]
                draw.rounded_rectangle((x0, y0, x1, y1), radius=12,
                                       fill=(29, 112, 83) if active else (38, 50, 66),
                                       outline=(80, 230, 174) if active else (79, 98, 121), width=2)
                draw.text((80, (y0+y1)//2-9), label, font=fonts[30], anchor='mm', fill='white')
                draw.text((80, y1-19), '当前模式' if active else '点击切换', font=fonts[16],
                          anchor='mm', fill=(185, 255, 223) if active else (180, 195, 213))
            draw.text((80, height-49), '每次运行一种', font=fonts[16], anchor='mm', fill=(166, 180, 197))
            self.panels[selected] = cv2.cvtColor(np.asarray(panel), cv2.COLOR_RGB2BGR)

    def compose(self, image, mode):
        """Keep the camera image intact and append controls in display space only."""
        height, self.camera_width = image.shape[:2]
        if height != self.height:
            self._build(height)
        return np.hstack((image, self.panels[mode]))

    def _hit(self, x, y):
        """Hit-test panel buttons; camera pixels and panel gaps have no action."""
        x -= self.camera_width
        for mode, (x0, y0, x1, y1) in self.buttons.items():
            if x0 <= x < x1 and y0 <= y < y1:
                return mode
        return None

    def on_mouse(self, event, x, y, flags, param):
        """Treat touch as a left click; require press and release on one button."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pressed = self._hit(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            released = self._hit(x, y)
            if self.pressed is not None and self.pressed == released:
                self.pending = released
            self.pressed = None

    def take_mode(self):
        """Consume one pending request on the main loop, outside the GUI callback."""
        mode, self.pending = self.pending, None
        return mode
