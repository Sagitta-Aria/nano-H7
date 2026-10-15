"""Headless optical-flow host tests; suitable for Nano Python 3.8 without ARM tools."""
import copy
import queue
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from host.optical_flow import DEFAULT, FlowSettings, config_command
from host.protocol import Reply
from host.webapp import Console
from host.simulator import SimulatedBoard


class HostFlowTests(unittest.TestCase):
    def test_save_validation_and_protocol_quantisation(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/"flow.json"; s = FlowSettings(p)
            c = copy.deepcopy(DEFAULT); c.update(height_mm="30.125", gain_x="1.002345", angle_deg="-90.25", map=7)
            s.save(c)
            self.assertEqual(FlowSettings(p).snapshot(), c)
            self.assertIn("30125 1002345 1000000 7 -9025 1", config_command(c))
            for key, value in (("height_mm", "NaN"), ("height_mm", True), ("height_mm", "30.0001"),
                               ("gain_x", 0), ("angle_deg", "90.001"), ("min_quality", 0), ("map", True)):
                invalid = dict(c, **{key:value})
                with self.assertRaises(ValueError):
                    s.save(invalid)
                self.assertEqual(s.snapshot(), c)

    def test_web_actions_polling_and_new_page_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            c = Console(optical_path=Path(directory)/"flow.json")
            worker = SimpleNamespace(submit=Mock(return_value=True), commands=queue.Queue())
            c.connection = worker; c.device = {"flow":"1"}
            self.assertTrue(c.flow_action("start")[0]); worker.submit.assert_called_with("FLOW START")
            self.assertTrue(c.flow_action("apply")[0]); worker.submit.assert_called_with(config_command(DEFAULT))
            reply = Reply(1, "OK cmd=FLOW_STAT x_um=300")
            events = c._reply("FLOW STAT", reply)
            self.assertEqual(events, [{"type":"flow", "fields":reply.fields}])
            self.assertEqual(c.hello()["flow"]["x_um"], "300")
            c.connection = None
            self.assertFalse(c.flow_action("start")[0])

    def test_demo_stop_holds_record_but_totals_advance(self):
        now = [0.]; b = SimulatedBoard(clock=lambda: now[0])
        b.flow.dispatch(["START"]); now[0]=1
        stopped = b.flow.dispatch(["STOP"])
        now[0]=2; current=b.flow.dispatch(["STAT"])
        def fields(text): return dict(p.split("=",1) for p in text.split() if "=" in p)
        self.assertEqual(fields(stopped)["rx_um"], fields(current)["rx_um"])
        self.assertNotEqual(fields(stopped)["x_um"], fields(current)["x_um"])

    def test_demo_rotates_each_increment_with_heading(self):
        now = [0.]; b = SimulatedBoard(clock=lambda: now[0])
        now[0] = 1.
        b.navigation.feedback(b.flow.initial_yaw + 90, True, 1, now[0])
        reply = b.flow.dispatch(["WORLD"])
        f = dict(p.split("=", 1) for p in reply.split() if "=" in p)
        self.assertAlmostEqual(int(f["wx_um"]), b.flow.total[1], delta=1)
        self.assertAlmostEqual(int(f["wy_um"]), -b.flow.total[0], delta=1)
        self.assertEqual(f["heading"], "1")


if __name__ == "__main__":
    unittest.main()
