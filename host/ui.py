"""Shared palette, ttk theme, scrollable tab body and small saved-UI state.

Both windows draw from here so colours, fonts and scroll behaviour cannot drift
apart, and every tab body scrolls instead of silently clipping its last widgets.
"""
import json
import os
import re
import tkinter as tk
from tkinter import ttk

# --- palette ---------------------------------------------------------------
BG = "#f3f5f8"
CARD = "#ffffff"
INK = "#17243b"
MUTED = "#627087"
GREEN = "#13846a"
GREEN_ACTIVE = "#0c6954"
RED = "#b43c49"
RED_ACTIVE = "#93313c"
HDR = "#111c2d"
LOG_BG = "#111c2d"
LOG_FG = "#d6e6ee"
HINT = "#90ddc7"

LED_FIXED = "#dd5b61"
LED_OFF = "#c7ced8"
LED_GREEN = "#20ad78"
LED_BLUE = "#468ee8"

FONT = "Microsoft YaHei UI"
NUM = "Segoe UI"
MONO = "Consolas"

# Widget classes that scroll natively; the wheel must stay theirs.
_SELF_SCROLLING = ("Treeview", "Text", "TCombobox", "TSpinbox", "Spinbox", "Scrollbar", "TScrollbar")


def apply_theme(root):
    """One place for every ttk style, including hover/pressed feedback."""
    root.configure(bg=BG)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", font=(FONT, 10), background=BG, foreground=INK)
    style.configure("TButton", padding=(12, 6))
    style.map("TButton", background=[("active", "#e2e7ee"), ("pressed", "#d3dae4")])
    style.configure("Accent.TButton", background=GREEN, foreground="white")
    style.map("Accent.TButton", background=[("active", GREEN_ACTIVE), ("pressed", GREEN_ACTIVE)])
    style.configure("Stop.TButton", background=RED, foreground="white")
    style.map("Stop.TButton", background=[("active", RED_ACTIVE), ("pressed", RED_ACTIVE)])
    style.configure("TNotebook.Tab", padding=(20, 8))
    style.configure("TLabelframe", padding=12)
    style.configure("TLabelframe.Label", font=(FONT, 11, "bold"), background=BG, foreground=INK)
    style.configure("Treeview", rowheight=26, background="white", fieldbackground="white")
    style.configure("Treeview.Heading", font=(FONT, 10, "bold"))
    style.configure("TCheckbutton", background=BG)
    return style


class ScrollArea(ttk.Frame):
    """Vertically scrollable tab body; the bar hides itself when it is not needed.

    The canvas requests a small height so a tab never forces the window taller
    than the screen; content is revealed by scrolling instead of being cut off.
    """

    def __init__(self, parent, height=300, padding=14):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, background=BG, highlightthickness=0, height=height)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        # Pack order matters: the bar must claim its strip before the canvas expands.
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.content = ttk.Frame(self.canvas, padding=padding)
        self._window = self.canvas.create_window(0, 0, window=self.content, anchor="nw")
        self.content.bind("<Configure>", self._content_changed)
        self.canvas.bind("<Configure>", self._canvas_resized)
        self._bar_shown = True

    def _content_changed(self, _event=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        self._sync_bar()

    def _canvas_resized(self, event):
        self.canvas.itemconfigure(self._window, width=event.width)
        self._sync_bar()

    def _sync_bar(self):
        needed = self.content.winfo_reqheight() > self.canvas.winfo_height()
        if needed == self._bar_shown:
            return
        self._bar_shown = needed
        if needed:
            self.scrollbar.pack(side="right", fill="y", before=self.canvas)
        else:
            self.scrollbar.pack_forget()
            self.canvas.yview_moveto(0)

    def _wheel(self, event):
        if self.content.winfo_reqheight() <= self.canvas.winfo_height():
            return None
        self.canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        return "break"

    def bind_wheel_tree(self):
        """Call once after the page is built; native scrollers keep their wheel."""
        self._bind_wheel(self.content)
        return self

    def _bind_wheel(self, widget):
        widget.bind("<MouseWheel>", self._wheel, add="+")
        for child in widget.winfo_children():
            if child.winfo_class() in _SELF_SCROLLING:
                continue
            self._bind_wheel(child)


class Prefs:
    """Best-effort memory of window size and chosen port.

    Stored next to the package so nothing outside the repository is written, and
    every failure (missing file, read-only folder) degrades to plain defaults.
    """

    PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".ui_state.json")
    GEOMETRY = re.compile(r"^\d{3,5}x\d{3,5}[+-]\d{1,5}[+-]\d{1,5}$")

    def __init__(self):
        self.data = {}
        try:
            with open(self.PATH, "r", encoding="utf-8") as handle:
                loaded = json.load(handle)
            if isinstance(loaded, dict):
                self.data = loaded
        except (OSError, ValueError):
            self.data = {}

    def text(self, key, default=""):
        value = self.data.get(key, default)
        return value if isinstance(value, str) else default

    def geometry(self, default):
        saved = self.text("geometry")
        return saved if self.GEOMETRY.match(saved) else default

    def put(self, key, value):
        self.data[key] = value

    def save(self):
        try:
            with open(self.PATH, "w", encoding="utf-8") as handle:
                json.dump(self.data, handle, ensure_ascii=False, indent=1)
        except OSError:
            pass  # A read-only checkout must still start and run.
