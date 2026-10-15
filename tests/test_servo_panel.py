"""Tk servo mapping checked against unchanged simulated firmware replies."""
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest

from host.protocol import Reply
from host.servo_panel import ServoPanel


class ServoPanelTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.app = SimpleNamespace(send=Mock(return_value=True))
        self.panel = ServoPanel(self.app, ttk.Frame(self.root))
        self.panel.reply("HELLO", Reply(1, "OK cmd=HELLO servo=2 servo1_travel=2700 servo2_travel=1800 servo1_pin=PH9 servo2_pin=PH6"))

    def test_defaults_selection_and_polling_do_not_send_or_overwrite_targets(self):
        self.assertEqual(self.panel.angle.get(), "120")
        self.assertEqual(self.panel.travel, 180)
        self.panel.reply("SERVO STAT", Reply(2, "OK cmd=SERVO_STAT ch=1 angle=90.0 us=1167 on=1 OK cmd=SERVO_STAT ch=2 angle=90.0 us=1500 on=1"))
        self.assertEqual(self.panel.angle.get(), "120")
        self.panel.channel.set("2")
        self.panel.select_channel()
        self.assertEqual(self.panel.angle.get(), "0")
        self.assertEqual(self.panel.travel, 270)
        self.assertIn("PH9 / TIM12_CH2", self.panel.heading.get())
        self.app.send.assert_not_called()

    def test_commands_and_replies_route_to_correct_pins(self):
        self.panel.send()
        self.app.send.assert_called_with("SERVO 1 180")
        self.panel.reply("SERVO 1 180", Reply(2, "OK cmd=SERVO ch=1 angle=180.0 us=1833 pin=PH9"))
        self.assertIn("120.0°", self.panel.state.get())
        self.panel.channel.set("2")
        self.panel.select_channel()
        self.panel.send()
        self.app.send.assert_called_with("SERVO 2 0")
        self.panel.reply("SERVO 2 0", Reply(3, "OK cmd=SERVO ch=2 angle=0.0 us=500 pin=PH6"))
        self.assertIn("0.0°", self.panel.state.get())
        self.panel.send(270)
        self.app.send.assert_called_with("SERVO 2 180")

    @patch("host.servo_panel.messagebox.showerror")
    def test_ph6_limit_and_calibration(self, showerror):
        self.panel.send(180.5)
        self.app.send.assert_not_called()
        showerror.assert_called_once()
        self.panel.pulse.set("1500")
        self.panel.send_pulse()
        self.app.send.assert_called_with("SERVO 1 135")
        self.panel.channel.set("2")
        self.panel.select_channel()
        self.panel.send_pulse()
        self.app.send.assert_called_with("SERVO 2 90")

    def test_pin_metadata_already_in_ui_order_is_not_swapped_twice(self):
        self.panel.reply("HELLO", Reply(4, "OK cmd=HELLO servo=2 servo1_travel=1800 servo2_travel=2700 servo1_pin=PH6 servo2_pin=PH9"))
        self.panel.send()
        self.app.send.assert_called_with("SERVO 1 120")
        self.panel.channel.set("2")
        self.panel.select_channel()
        self.panel.send()
        self.app.send.assert_called_with("SERVO 2 0")


if __name__ == "__main__":
    unittest.main()
