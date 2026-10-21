"""Exercise compiled chassis commands and live GUI-worker safety boundaries."""
import queue
import time
import unittest
from tests.test_arm import ArmImage, SCRATCH, TIM8
from host.protocol import encode, decode
from host.transport import Connection
from host.simulator import SimulatedBoard
from tests.chassis_fixture import MATRIX


class ArmChassisTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmImage()
        self.arm.call("motion_init")
        self.arm.call("motion_lift_init")
        self.arm.call("motion_lift_direction", 1)
        self.arm.write32(0x4001101C, 1 << 7)
        self.sequence = 100
        self.request("PING")
        self.request("ARM 1")

    def request(self, body, sequence=None):
        self.sequence += 1
        self.arm.transmitted.clear()
        self.arm.uc.mem_write(SCRATCH + 4096, encode(sequence or self.sequence, body).rstrip(b"\n") + b"\0")
        self.arm.call("process_line", SCRATCH + 4096)
        return decode(bytes(self.arm.transmitted))

    def group_edge(self):
        launch = self.arm.read32(TIM8 + 0x34)
        self.arm.write32(TIM8 + 0x24, launch)
        self.arm.write32(TIM8 + 0x10, 0x1E)
        self.arm.call("TIM8_CC_IRQHandler")
        self.arm.write32(TIM8 + 0x10, 0)

    def test_motion_fault_clear_requires_idle_keeps_history_and_invalidates_pose(self):
        self.assertFalse(self.request('MOTION CLEAR').ok,'must disarm first')
        self.request('ARM 0');self.request('NAV RESET 0 0');self.request('ARM 1')
        self.assertTrue(self.request('MOVE 1 100 5000 2000').ok)
        self.arm.edge(0,late=3000)
        self.assertEqual(self.request('MOTION STAT').fields['reason'],'step_deadline')
        self.request('STOP')
        self.assertEqual(self.request('ARM 1').fields['code'],'MOTION_FAULT_LATCHED')
        self.assertTrue(self.request('MOTION CLEAR').ok)
        f=self.request('MOTION STAT').fields
        self.assertEqual((f['latched'],f['total'],f['reason']),('0','1','step_deadline'))
        self.assertEqual(self.request('NAV STAT').fields['ready'],'0')
        self.assertEqual(self.arm.call('motion_armed'),0)
        self.assertTrue(self.request('ARM 1').ok)
        self.assertTrue(self.request('MOVE 1 2 100 2000').ok)

    def test_fault_invalidates_moving_aux_reference_without_rezeroing(self):
        self.request('MOVE 5 100 3000 2000');self.arm.edge(4)
        self.request('MOVE 1 100 5000 2000');self.arm.edge(0,late=3000)
        self.assertEqual(self.request('AUX STAT 5').fields['referenced'],'0')
        pos=self.arm.status(4)[0]
        self.assertTrue(self.request('MOTION CLEAR').ok)
        self.request('LIFT INIT')
        self.assertEqual(self.arm.status(4)[0],pos)
        self.assertEqual(self.request('AUX STAT 5').fields['referenced'],'0')
        self.request('ARM 1')
        self.assertFalse(self.request('MOVE 5 1 100 1000').ok)
        self.request('ARM 0')
        # Operator has physically restored the lift to its top before this command.
        self.assertTrue(self.request('AUX ZERO 5').ok)
        self.assertEqual(self.request('AUX STAT 5').fields['referenced'],'1')
        self.assertEqual(self.arm.status(4)[0],0)

    def test_motion_clear_does_not_truncate_a_stopping_high_pulse(self):
        self.request('MOVE 1 100 100 1000');self.arm.edge(0)
        self.request('STOP')
        self.assertFalse(self.request('MOTION CLEAR').ok)
        self.arm.edge(0)
        self.assertTrue(self.request('MOTION CLEAR').ok)

    def test_lost_lift_reference_can_be_restored_inside_nonzero_bounds(self):
        self.request('MOVE 5 100 3000 2000')
        for _ in range(40):self.arm.edge(4)
        self.request('AUX HALT 5')
        self.assertTrue(self.request('AUX CONFIG 5 10 100').ok)
        self.request('MOVE 5 10 3000 2000');self.arm.edge(4,late=3000)
        self.assertFalse(self.request('AUX REFERENCE 5 10').ok,'clear the fault first')
        self.request('MOTION CLEAR')
        self.assertFalse(self.request('AUX ZERO 5').ok,'zero is outside these bounds')
        self.assertFalse(self.request('AUX REFERENCE 5 101').ok)
        # Operator has physically placed the lift at its known 10-pulse mark.
        self.assertTrue(self.request('AUX REFERENCE 5 10').ok)
        self.assertEqual(self.request('AUX STAT 5').fields['pos'],'10')
        self.assertFalse(self.request('AUX REFERENCE 5 20').ok,'healthy reference cannot be overwritten')

    def test_controller_fault_clear_does_not_fabricate_fresh_imu(self):
        self.request('NAV IMU 0 1 1')
        self.assertTrue(self.request('TURN MOVE REL 9000 500 2000').ok)
        self.arm.call('turn_tick',1000)
        self.assertEqual(self.request('TURN STAT').fields['state'],'fault')
        self.assertEqual(self.arm.call('motion_faults'),0)
        self.request('STOP');self.assertTrue(self.request('MOTION CLEAR').ok)
        self.assertEqual(self.request('TURN STAT').fields['state'],'stopped')
        self.request('NAV IMU 0 0 1');self.request('ARM 1')
        self.assertFalse(self.request('TURN MOVE REL 9000 500 2000').ok)

    def test_six_moves_exact_equal_steps_and_common_compare(self):
        self.assertTrue(self.request("CHASSIS CONFIG 0").ok)
        positions = [0] * 4
        for direction, signs in MATRIX.items():
            self.assertTrue(self.request(f"CHASSIS MOVE {direction} 7 5000 2000").ok)
            for _ in range(14):
                values = [self.arm.read32(TIM8 + 0x34 + 4*i) for i in range(4)]
                self.assertEqual(len(set(values)), 1)
                self.group_edge()
                self.arm.call("motion_tick")
            self.arm.call("chassis_tick", 10)
            reply = self.request("CHASSIS STAT")
            self.assertEqual(reply.fields["state"], "done")
            self.assertEqual(reply.fields["remaining"], "0")
            positions = [p + 7*s for p, s in zip(positions, signs)]
            self.assertEqual([self.arm.status(i)[0] for i in range(4)], positions)
            self.assertEqual([self.arm.status(i)[0] for i in range(4, 7)], [0, 0, 0])

    def test_polarity_default_and_ram_calibration(self):
        self.assertEqual(self.request("CHASSIS STAT").fields["invert"], "5")
        self.request("CHASSIS MOVE FWD 1 100 2000")
        self.group_edge(); self.group_edge()
        self.arm.call("chassis_tick", 1)
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [-1, 1, -1, 1])
        self.request("CHASSIS CONFIG 10")
        self.request("CHASSIS MOVE FWD 1 100 2000")
        self.group_edge(); self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [0, 0, 0, 0])

    def test_busy_atomic_rejection_and_auxiliary_independence(self):
        self.request("MOVE 2 100 100 2000")
        self.assertFalse(self.request("CHASSIS MOVE FWD 10 100 2000").ok)
        self.assertEqual([self.arm.status(i)[4] for i in range(4)], [0, 1, 0, 0])
        self.request("CHASSIS HALT")
        self.assertTrue(self.request("CHASSIS MOVE LEFT 10 100 2000").ok)
        self.assertFalse(self.request("MOVE 1 10 100 2000").ok)
        self.assertFalse(self.request("WHEELS 100 100 100 100 2000").ok)
        self.assertFalse(self.request("CHASSIS CONFIG 0").ok)
        self.assertTrue(self.request("MOVE 5 50 100 2000").ok)
        self.request("CHASSIS HALT")
        self.assertEqual(self.arm.status(4)[4], 1)
        self.assertEqual(self.arm.call("motion_armed"), 1)

    def test_no_partial_effect_of_invalid_or_duplicate_move(self):
        for text in ["CHASSIS MOVE FWD 0 100 2000", "CHASSIS MOVE BAD 10 100 2000",
                     "CHASSIS MOVE LEFT 10 5001 2000", "CHASSIS MOVE CW 10 100 0"]:
            self.assertFalse(self.request(text).ok)
            self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.request("CHASSIS MOVE FWD 10 100 2000", sequence=1000)
        self.group_edge(); self.group_edge()
        self.assertTrue(self.request("CHASSIS MOVE FWD 10 100 2000", sequence=1000).ok)
        self.assertEqual([self.arm.status(i)[2] for i in range(4)], [9]*4)

    def test_jog_lease_expires_despite_ping_and_does_not_replay_refresh(self):
        self.request("CHASSIS JOG FWD 500 2000", sequence=1000)
        self.arm.write32(self.arm.symbols["xTickCount"], 350)
        self.request("PING")
        self.request("CHASSIS JOG FWD 500 2000", sequence=1000)
        self.arm.call("chassis_tick", 400)
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.assertEqual(self.request("CHASSIS STAT").fields["state"], "stopped")
        self.assertEqual(self.arm.call("motion_armed"), 1)

    def test_jog_reversal_waits_for_falling_edges_then_common_launch(self):
        self.request("CHASSIS CONFIG 0")
        self.request("CHASSIS JOG FWD 500 2000")
        self.group_edge()  # all STEP high
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [1]*4)
        self.request("CHASSIS JOG BACK 500 2000")
        self.arm.call("chassis_tick", 1)
        self.assertEqual(self.request("CHASSIS STAT").fields["state"], "switching")
        self.group_edge()  # finish old highs, no extra rising edge
        self.arm.call("chassis_tick", 2)
        self.assertEqual(self.request("CHASSIS STAT").fields["state"], "jogging")
        self.assertEqual(len({self.arm.read32(TIM8+0x34+4*i) for i in range(4)}), 1)
        self.group_edge()
        self.assertEqual([self.arm.status(i)[0] for i in range(4)], [0]*4)

    def test_global_stop_cancels_pending_restart(self):
        self.request("CHASSIS JOG FWD 500 2000")
        self.group_edge()
        self.request("CHASSIS JOG BACK 500 2000")
        self.request("STOP")
        self.group_edge()
        self.arm.call("chassis_tick", 2)
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)
        self.request("ARM 1")
        self.arm.call("chassis_tick", 3)
        self.assertEqual(self.arm.call("motion_chassis_active"), 0)


class ChassisHostTests(unittest.TestCase):
    def test_clear_discards_pending_moves_and_jog_without_arming(self):
        worker=Connection('SIM',queue.Queue())
        worker.set_jog('CHASSIS JOG FWD 500 2000')
        worker.submit('ARM 1');worker.submit('MOVE 5 100 300 1000')
        worker.submit('MOTION CLEAR')
        self.assertIsNone(worker.current_jog())
        self.assertEqual(worker.commands.qsize(),1)
        self.assertEqual(worker.commands.get_nowait()[2],'MOTION CLEAR')

    def test_simulated_finite_moves_and_auxiliary_halt(self):
        now = [10.]
        board = SimulatedBoard(clock=lambda: now[0])
        seq = 0
        def send(body):
            nonlocal seq
            seq += 1
            return decode(board.receive(encode(seq, body)))
        send("LIFT INIT"); send("LIFT DIR 1"); send("PING"); send("ARM 1"); send("CHASSIS CONFIG 0")
        position = [0]*4
        for name, signs in MATRIX.items():
            self.assertTrue(send(f"CHASSIS MOVE {name} 10 500 2000").ok)
            for _ in range(100):
                now[0] += .01
                send("PING")
            self.assertEqual(send("CHASSIS STAT").fields["state"], "done")
            position = [p+10*s for p, s in zip(position, signs)]
            self.assertEqual([a["pos"] for a in board.axes[:4]], position)
        send("MOVE 5 1000 100 2000");send("CHASSIS JOG LEFT 100 2000")
        now[0] += .41;send("PING")
        self.assertTrue(board.axes[4]["active"])
        self.assertFalse(any(a["active"] for a in board.axes[:4]))

    def test_worker_stops_jog_when_gui_stops_refreshing(self):
        events = queue.Queue()
        worker = Connection("SIM", events)
        worker.start()
        try:
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                if events.get(timeout=1)[0] == "connected":
                    break
            worker.submit("ARM 1")
            while True:
                kind, value = events.get(timeout=1)
                if kind == "reply" and value[0] == "ARM 1":
                    self.assertTrue(value[1].ok); break
            worker.set_jog("CHASSIS JOG FWD 500 2000")
            while True:
                kind, value = events.get(timeout=1)
                if kind == "reply" and value[0].startswith("CHASSIS JOG"):
                    break
            time.sleep(.55)
            self.assertFalse(any(a["active"] for a in worker.serial.board.axes[:4]))
            self.assertTrue(worker.serial.board.armed)
            self.assertLess(time.monotonic() - worker.last_ping, .4)
        finally:
            worker.close();worker.join(2)

    def test_cancelled_jog_is_not_written_or_retried(self):
        from host.simulator import SimulatedSerial
        worker = Connection("SIM", queue.Queue());worker.serial = SimulatedSerial()
        worker.set_jog("CHASSIS JOG FWD 500 2000")
        worker.submit("CHASSIS HALT")
        self.assertIsNone(worker.exchange("CHASSIS JOG FWD 500 2000"))
        self.assertEqual(worker.serial.board.rx, 0)


if __name__ == "__main__":
    unittest.main()
