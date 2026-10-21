"""Physical connector contract: PH9 is 180 degrees; PE4 is 270 degrees.

Exercise host targets through protocol replies, without touching real hardware.
"""
import unittest

from host.protocol import encode, decode
from host.servo_config import SERVO_LAYOUT, initial_commands, wire_angle, display_angle, wire_channel
from host.simulator import SimulatedBoard


class ServoPinMappingTests(unittest.TestCase):
    def test_ph9_180_channel_does_not_steal_pe4_270_channel(self):
        """PH9 means 180 degrees only in the layout that also advertises PE4."""
        for fields, expected in (({'servo1_pin':'PE4', 'servo2_pin':'PH9'}, [2, 1]),
                                 ({'servo1_pin':'PH9', 'servo2_pin':'PE4'}, [1, 2]),
                                 ({'servo1_pin':'PH9', 'servo2_pin':'PH6'}, [2, 1])):
            self.assertEqual([wire_channel(fields, ch) for ch in (1, 2)], expected)

    def test_initial_targets_reach_the_original_physical_connectors(self):
        board = SimulatedBoard()
        hello = decode(board.receive(encode(1, 'HELLO'))).fields
        replies = [decode(board.receive(encode(i, command))).fields
                   for i, command in enumerate(initial_commands(hello), 2)]
        by_pin = {reply['pin']: reply for reply in replies}
        self.assertEqual(by_pin['PH9']['angle'], '120.0')
        self.assertEqual(by_pin['PH9']['us'], '1833')
        self.assertEqual(by_pin['PE4']['angle'], '0.0')
        self.assertEqual(by_pin['PE4']['us'], '500')

    def test_layout_limits_and_readback_follow_pins(self):
        self.assertEqual((SERVO_LAYOUT[1]['pin'], SERVO_LAYOUT[1]['travel']), ('PH9', 180))
        self.assertEqual((SERVO_LAYOUT[2]['pin'], SERVO_LAYOUT[2]['travel']), ('PE4', 270))
        self.assertEqual(wire_angle({'servo1_travel':'2700','servo2_travel':'1800'}, 1, 120), '120')
        self.assertEqual(wire_angle({'servo1_travel':'2700','servo2_travel':'1800'}, 2, 270), '270')
        self.assertEqual(display_angle(1, 1500), '90.0')
        self.assertEqual(display_angle(2, 1500), '135.0')
        with self.assertRaises(ValueError):
            wire_angle({}, 1, 180.5)


if __name__ == '__main__':
    unittest.main()
