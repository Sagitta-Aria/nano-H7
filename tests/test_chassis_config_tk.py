"""Tk regression for unsent DIR edits across initialization and status replies."""
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
from host.chassis_panel import ChassisPanel
from host.protocol import Reply
from host.webapp import Console as WebConsole


class ChassisConfigTkTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk(); self.root.withdraw()
        notebook = ttk.Notebook(self.root)
        self.app = SimpleNamespace(root=self.root, notebook=notebook, connected=True,
                                  send=Mock(return_value=True))
        parent = ttk.Frame(notebook); notebook.add(parent)
        self.panel = ChassisPanel(self.app, parent)
        self.panel.reply("HELLO", Reply(1, "OK cmd=HELLO chassis=1"))

    def tearDown(self):
        self.root.destroy()

    def report(self, mask, command="CHASSIS STAT", name="CHASSIS"):
        self.panel.reply(command, Reply(2, "OK cmd={} state=idle direction=FWD move_id=0 total=0 remaining=0 active=0 invert={}".format(name, mask)))

    def mask(self):
        return sum(1 << i for i, var in enumerate(self.panel.invert) if var.get())

    def select(self, mask):
        for i, var in enumerate(self.panel.invert): var.set(bool(mask & (1 << i)))

    def test_first_late_status_preserves_already_selected_boxes(self):
        self.select(5)
        self.report(10)
        self.assertEqual(self.mask(), 5)

    def test_polls_keep_draft_and_accepted_config_cannot_erase_newer_edits(self):
        self.report(10)
        self.select(5)
        self.panel.configure()
        self.app.send.assert_called_with("CHASSIS CONFIG 5")
        self.select(7)
        self.report(5, "CHASSIS CONFIG 5")
        self.assertEqual(self.mask(), 7)
        self.report(5)
        self.assertEqual(self.mask(), 7)

    def test_actual_and_simulator_status_formats_initialize_and_follow_board(self):
        self.report(5, name="CHASSIS_STAT")
        self.assertTrue(self.panel.config_received)
        self.assertEqual(self.mask(), 5)
        self.report(10)
        self.assertEqual(self.mask(), 10)

    def test_confirmation_rejection_and_reconnect(self):
        self.report(10)
        self.select(5)
        self.panel.reply("CHASSIS CONFIG 5", Reply(3,"ERR code=CHASSIS_BUSY"))
        self.report(10)
        self.assertEqual(self.mask(), 5)
        self.report(5, "CHASSIS CONFIG 5")
        self.report(10)
        self.assertEqual(self.mask(), 10)
        self.select(5)
        self.panel.reply("HELLO", Reply(4,"OK cmd=HELLO chassis=1"))
        self.report(10)
        self.assertEqual(self.mask(), 10)


class ChassisConfigWebTests(unittest.TestCase):
    def test_real_board_report_reaches_snapshot_and_page(self):
        console = WebConsole()
        for name, mask in (("CHASSIS", 5), ("CHASSIS_STAT", 10)):
            reply = Reply(1, "OK cmd={} state=idle direction=FWD move_id=0 total=0 remaining=0 active=0 invert={}".format(name, mask))
            events = console._reply("CHASSIS STAT", reply)
            self.assertEqual(console.hello()["chassis"]["invert"], str(mask))
            self.assertEqual([e["type"] for e in events], ["chassis"])


if __name__ == "__main__": unittest.main()
