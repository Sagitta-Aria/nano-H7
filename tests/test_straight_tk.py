"""Real Tk form submission checks, using a mocked transport and no hardware."""
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest
from host.chassis_panel import ChassisPanel
from host.protocol import Reply


class StraightTkTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk(); self.root.withdraw()
        notebook=ttk.Notebook(self.root)
        armed=tk.StringVar(value='已使能')
        self.app=SimpleNamespace(root=self.root,notebook=notebook,connected=True,send=Mock(),values={'armed':armed})
        parent=ttk.Frame(notebook); notebook.add(parent)
        self.panel=ChassisPanel(self.app,parent)
        self.panel.reply('HELLO',Reply(1,'OK cmd=HELLO chassis=1 straight=1'))

    def tearDown(self): self.root.destroy()

    def test_checkbox_changes_actual_command_and_does_not_change_ordinary_move(self):
        self.panel.assist.set(True); self.panel.direction.set('后退'); self.panel.move()
        self.app.send.assert_called_with('CHASSIS STRAIGHT BACK 1000 500 2000')
        self.panel.assist.set(False); self.panel.move()
        self.app.send.assert_called_with('CHASSIS MOVE BACK 1000 500 2000')

    def test_old_firmware_and_rotation_cannot_send_assist(self):
        self.panel.assist.set(True)
        for old in (True,False):
            self.panel.assist_supported=not old; self.panel.direction.set('原地右旋 ↻')
            self.app.send.reset_mock()
            with patch('host.chassis_panel.messagebox.showerror') as error:
                self.panel.move()
            error.assert_called_once(); self.app.send.assert_not_called()


if __name__=='__main__': unittest.main()
