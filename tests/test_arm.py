"""Execute the actual Keil AXF functions in Unicorn, with peripheral registers
mapped as memory. This validates C logic, NOT STM32 peripheral timing/electrics.
Install tools/test-requirements.txt into .testdeps before running.
"""
from pathlib import Path
import ctypes
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".testdeps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR
from unicorn.arm_const import UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_CPU_ARM_CORTEX_M7
from elftools.elf.elffile import ELFFile
from host.protocol import encode, decode

RAM = 0x20000000
SCRATCH = 0x24000000
TIM8 = 0x40010400
TIM5 = 0x40000C00
TIM12 = 0x40001800
TIM15 = 0x40014000


class ArmImage:
    def __init__(self):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M7)
        # Match the MCU startup FPU enable; navigation now executes VFP casts.
        self.uc.reg_write(UC_ARM_REG_C1_C0_2, 0xF << 20)
        self.uc.reg_write(UC_ARM_REG_FPEXC, 1 << 30)
        for address, size in [(0x08000000, 0x200000), (RAM, 0x20000), (SCRATCH, 0x10000),
                              (0x40000000, 0x30000), (0x58000000, 0x30000),
                              (0xE0000000, 0x2000), (0xE000E000, 0x2000)]:
            self.uc.mem_map(address, size)
        with (ROOT / "firmware/build/robot_h743.axf").open("rb") as file:
            elf = ELFFile(file)
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    self.uc.mem_write(segment["p_vaddr"], segment.data())
            # Recreate C runtime scatter initialization at execution addresses.
            # ARM linker load segments can store initial RW bytes in flash.
            for section in elf.iter_sections():
                if section["sh_flags"] & 2 and section["sh_type"] == "SHT_PROGBITS":
                    self.uc.mem_write(section["sh_addr"], section.data())
            table = elf.get_section_by_name(".symtab")
            self.symbols = {s.name: s["st_value"] for s in table.iter_symbols()}
        self.pending_sr = {}
        self.transmitted = bytearray()
        self.wireless_transmitted = bytearray()
        self.cycle_step = 0
        # MMIO-only UART model: empty TX registers, no real wire timing.
        self.write32(0x4001101C, 1 << 7)
        self.write32(0x4000781C, 1 << 7)
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self.peripheral_write)
        self.uc.hook_add(UC_HOOK_MEM_READ, self.peripheral_read)
        # Keil can switch to compressed RW data when the initialized globals
        # grow. Section bytes then hold compressed load data, not RAM values.
        # Execute each real scatter initializer instead of assuming raw copy.
        base = self.symbols["Region$$Table$$Base"]
        limit = self.symbols["Region$$Table$$Limit"]
        for address in range(base, limit, 16):
            source, target, size, initializer = struct.unpack("<4I", self.uc.mem_read(address, 16))
            self.call(initializer, source, target, size)

    def peripheral_write(self, uc, access, address, size, value, user):
        # Timer SR is rc_w0, not normal RAM: writing 1 preserves an existing flag.
        if address in (TIM8 + 0x10, TIM5 + 0x10):
            previous = self.pending_sr.get(address, self.read32(address))
            self.pending_sr[address] = previous & value
        if address == 0x40011028:  # USART1 TDR
            self.transmitted.append(value & 255)
        if address == 0x40007828:  # UART7 TDR
            self.wireless_transmitted.append(value & 255)

    def peripheral_read(self, uc, access, address, size, value, user):
        if address == 0xE0001004 and self.cycle_step:
            self.write32(address, self.read32(address) + self.cycle_step)
        if address in self.pending_sr:
            self.write32(address, self.pending_sr.pop(address))

    def read32(self, address):
        return struct.unpack("<I", self.uc.mem_read(address, 4))[0]

    def write32(self, address, value):
        self.uc.mem_write(address, struct.pack("<I", value & 0xFFFFFFFF))

    def call(self, name, *args):
        """AAPCS register/stack arguments, isolated stack, real compiled body."""
        stack = RAM + 0x1E000
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            self.uc.reg_write(register, value & 0xFFFFFFFF)
        for i, value in enumerate(args[4:]):
            self.write32(stack + i * 4, value)
        self.uc.reg_write(UC_ARM_REG_SP, stack)
        self.uc.reg_write(UC_ARM_REG_LR, 0x080FF001)
        entry = self.symbols[name] if isinstance(name, str) else name
        self.uc.emu_start(entry | 1, 0x080FF000, count=2000000)
        for address, value in self.pending_sr.items():
            self.write32(address, value)
        self.pending_sr.clear()
        return self.uc.reg_read(UC_ARM_REG_R0)

    def status(self, axis):
        self.call("motion_status", axis, SCRATCH)
        return struct.unpack("<iiIII", self.uc.mem_read(SCRATCH, 20))

    def edge(self, axis, late=0):
        timer = TIM8 if axis < 4 else TIM5
        channel = axis if axis < 4 else axis - 4
        value = self.read32(timer + 0x34 + channel * 4)
        self.write32(timer + 0x24, (value + late) & 0xFFFF)
        self.write32(timer + 0x10, 2 << channel)
        self.call("TIM8_CC_IRQHandler" if axis < 4 else "TIM5_IRQHandler")
        # In hardware SR is rc_w0; memory-only emulation needs explicit clear.
        self.write32(timer + 0x10, 0)


class ArmProtocolTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmImage()

    def test_known_crc_vector(self):
        self.arm.uc.mem_write(SCRATCH, b"123456789")
        self.assertEqual(self.arm.call("protocol_crc", SCRATCH, 9), 0x29B1)

    def test_python_to_firmware_and_back(self):
        for sequence, body in [(1, "LED 1 0"), (0xFFFFFFFF, "MOVE 7 -9999 3000 2000")]:
            self.arm.uc.mem_write(SCRATCH, encode(sequence, body).rstrip(b"\n") + b"\0")
            self.assertEqual(self.arm.call("protocol_decode", SCRATCH, SCRATCH + 512), 1)
            self.assertEqual(self.arm.read32(SCRATCH + 512), sequence)
            self.arm.uc.mem_write(SCRATCH + 1024, b"OK cmd=LED led=1\0")
            self.assertEqual(self.arm.call("protocol_encode", sequence, SCRATCH + 1024, SCRATCH + 2048, 384), 1)
            frame = bytes(self.arm.uc.mem_read(SCRATCH + 2048, 384)).split(b"\0")[0]
            self.assertEqual(decode(frame).fields["led"], "1")

    def test_bad_crc_and_invalid_numbers(self):
        frame = encode(1, "LED 1 0").replace(b"LED 1", b"LED 0")
        self.arm.uc.mem_write(SCRATCH, frame.rstrip(b"\n") + b"\0")
        self.assertEqual(self.arm.call("protocol_decode", SCRATCH, SCRATCH + 512), 0)
        for value in [b"2147483648", b"1x", b"+2", b"", b"-3001"]:
            self.arm.uc.mem_write(SCRATCH, value + b"\0")
            self.assertEqual(self.arm.call("protocol_int", SCRATCH, -3000, 3000, SCRATCH + 512), 0)

    def test_actual_command_dispatch_replay_and_limits(self):
        self.arm.call("motion_init")
        self.arm.call("motion_lift_init")
        self.arm.call("motion_lift_direction", 1)
        self.arm.write32(0x4001101C, 1 << 7)  # TXE ready; no physical UART.

        def request(sequence, body):
            self.arm.transmitted.clear()
            self.arm.uc.mem_write(SCRATCH + 4096, encode(sequence, body).rstrip(b"\n") + b"\0")
            self.arm.call("process_line", SCRATCH + 4096)
            return decode(bytes(self.arm.transmitted))

        self.assertTrue(request(1, "PING").ok)
        self.assertTrue(request(2, "ARM 1").ok)
        self.assertEqual(request(3, "LED 1 1").fields["led"], "3")
        self.assertFalse(request(4, "MOVE 5 100 3001 2000").ok)
        self.assertTrue(request(5, "MOVE 5 10 3000 2000").ok)
        self.arm.edge(4)
        self.arm.edge(4)
        self.assertTrue(request(5, "MOVE 5 10 3000 2000").ok)
        self.assertEqual(self.arm.status(4)[2], 9)  # replay did not restart move
        self.assertEqual(request(5, "MOVE 5 20 3000 2000").fields["code"], "SEQUENCE_REUSE")
        self.assertTrue(request(6, "STOP").ok)
        self.assertEqual(self.arm.call("motion_armed"), 0)
        self.arm.write32(self.arm.symbols["xTickCount"], 1100)
        self.assertEqual(request(7, "ARM 1").fields["code"], "HEARTBEAT_REQUIRED")

    def test_vector_table_uses_real_rtos_and_timer_handlers(self):
        self.assertTrue(RAM < self.arm.read32(0x08000000) <= RAM + 0x20000)
        for slot, name in [(11, "SVC_Handler"), (14, "PendSV_Handler"), (15, "SysTick_Handler"),
                           (16 + 46, "TIM8_CC_IRQHandler"), (16 + 50, "TIM5_IRQHandler"),
                           (16 + 37, "USART1_IRQHandler"), (16 + 82, "UART7_IRQHandler")]:
            self.assertEqual(self.arm.read32(0x08000000 + 4 * slot), self.arm.symbols[name] | 1)


class ArmMotionTests(unittest.TestCase):
    def setUp(self):
        self.arm = ArmImage()
        self.arm.call("motion_init")
        self.arm.call("motion_lift_init")
        self.arm.call("motion_lift_direction", 1)

    def test_boot_disarmed_and_limits(self):
        self.assertEqual(self.arm.call("motion_armed"), 0)
        self.assertEqual(self.arm.call("motion_move", 0, 10, 1000, 2000, 1), 0)
        self.assertEqual(self.arm.call("motion_arm", 1), 1)
        self.assertEqual(self.arm.call("motion_move", 0, 10, 5001, 2000, 1), 0)
        self.assertEqual(self.arm.call("motion_move", 4, 10, 3001, 2000, 1), 0)
        self.assertEqual(self.arm.call("motion_move", 6, 0, 1000, 2000, 1), 0)

    def test_all_seven_exact_pulse_counts(self):
        self.arm.call("motion_arm", 1)
        for i in range(7):
            steps = (i + 1) * (1 if i % 2 or i == 4 else -1)
            self.assertEqual(self.arm.call("motion_move", i, steps, 5000 if i < 4 else 3000, 2000, i + 10), 1)
            for _ in range(2 * abs(steps)):
                self.arm.edge(i)
            pos, sps, remaining, move_id, active = self.arm.status(i)
            self.assertEqual((pos, remaining, active, move_id), (steps, 0, 0, i + 10))

    def test_signed_wheel_targets_and_reversal(self):
        self.arm.call("motion_arm", 1)
        self.arm.uc.mem_write(SCRATCH, struct.pack("<iiii", 5000, -4000, 3000, -2000))
        self.assertEqual(self.arm.call("motion_wheels", SCRATCH, 2000, 25), 1)
        for i in range(4):
            self.arm.edge(i)
            self.assertEqual(self.arm.status(i)[0], 1 if i % 2 == 0 else -1)
        self.arm.uc.mem_write(SCRATCH, struct.pack("<iiii", -1000, 1000, -1000, 1000))
        self.assertEqual(self.arm.call("motion_wheels", SCRATCH, 2000, 26), 0)
        self.assertEqual(self.arm.status(0)[3], 25)

    def test_wrap_and_one_sps(self):
        self.arm.write32(TIM8 + 0x24, 65530)
        self.arm.call("motion_arm", 1)
        self.assertEqual(self.arm.call("motion_move", 0, 2, 1, 100, 33), 1)
        for _ in range(4):
            self.arm.edge(0)
        self.assertEqual(self.arm.status(0), (2, 0, 0, 33, 0))
        self.assertEqual(self.arm.call("motion_faults"), 0)

    def test_stop_finishes_high_pulse_without_extra_step(self):
        self.arm.call("motion_arm", 1)
        self.arm.call("motion_move", 4, 100, 3000, 2000, 1)
        self.arm.edge(4)
        self.arm.call("motion_stop")
        self.assertEqual(self.arm.status(4)[4], 1)
        self.arm.edge(4)
        self.assertEqual(self.arm.status(4)[0], 1)
        self.assertEqual(self.arm.status(4)[4], 0)
        self.assertEqual(self.arm.call("motion_armed"), 0)

    def test_missed_deadline_latches_fault(self):
        self.arm.call("motion_arm", 1)
        self.arm.call("motion_move", 0, 100, 5000, 2000, 1)
        self.arm.edge(0, late=3000)
        self.assertEqual(self.arm.call("motion_faults"), 1)
        self.assertEqual(self.arm.call("motion_armed"), 0)
        self.assertEqual(self.arm.call("motion_arm", 1), 0)

    def test_fractional_rates_and_ramp(self):
        for sps in [1, 7, 123, 3000, 4999, 5000]:
            self.arm.write32(SCRATCH, 0)
            # One complete divider repeat produces exactly one second of ticks.
            total = sum(self.arm.call("motion_interval", sps, SCRATCH) for _ in range(2 * sps))
            self.assertEqual(total, 50000)
        self.assertEqual(self.arm.call("motion_ramp", 1000, 5000000, 2000), 3000)
        self.assertEqual(self.arm.call("motion_ramp", 1000, 0, 2000), 0)


class ArmServoTests(unittest.TestCase):
    """The pulse table is data and is checked exhaustively here; that PH9/PE4 really
    emits 50 Hz and that the horn reaches the commanded angle are hardware facts
    this emulator cannot see (OCxE drives nothing without a real timer clock)."""

    def setUp(self):
        self.arm = ArmImage()
        self.arm.call("servo_init")

    def servo(self, channel):
        result = self.arm.call("servo_status", channel, SCRATCH)
        return (result, *struct.unpack("<HHI", self.arm.uc.mem_read(SCRATCH, 8)))

    def expected_pulse(self, steps):
        return 500 + (2000 * steps + 270) // 540

    def test_timer_programmed_for_50hz_with_one_microsecond_ticks(self):
        for timer in (TIM15, TIM12):
            self.assertEqual(self.arm.read32(timer + 0x28), 199)      # PSC
            self.assertEqual(self.arm.read32(timer + 0x2C), 19999)    # ARR
            self.assertEqual(self.arm.read32(timer + 0x20), 4 if timer == TIM15 else 16)
            self.assertEqual(self.arm.read32(timer + 0x00) & 1, 1)    # CEN
        self.assertEqual(self.arm.read32(TIM15 + 0x44) & (1 << 15), 1 << 15)  # MOE

    def test_pe4_unchanged_and_180_degree_route_moves_to_ph9(self):
        """Check the compiled mux/clock setup; electrical timing needs a board."""
        gpioe, gpioh, rcc = 0x58021000, 0x58021C00, 0x58024400
        self.assertEqual((self.arm.read32(gpioe) >> 8) & 3, 2)
        self.assertEqual((self.arm.read32(gpioe + 0x20) >> 16) & 15, 4)
        self.assertEqual((self.arm.read32(gpioh) >> 18) & 3, 2)
        self.assertEqual((self.arm.read32(gpioh + 0x24) >> 4) & 15, 2)
        self.assertEqual((self.arm.read32(gpioh) >> 12) & 3, 0)  # PH6 not configured
        self.assertTrue(self.arm.read32(rcc + 0xE0) & (1 << 4))  # GPIOE
        self.assertTrue(self.arm.read32(rcc + 0xE8) & (1 << 6))  # TIM12
        self.assertTrue(self.arm.read32(rcc + 0xF0) & (1 << 16))  # TIM15

    def test_pe4_uses_ch1n_with_positive_servo_pulses(self):
        """STM32H743IITx AF4: PE4=TIM15_CH1N; PE5, not PE4, is CH1.

        RM0433 Rev 6 Table 337: with CC1E=0 and CC1NE=1, OC1N is
        OC1REF XOR CC1NP. CC1NP must be zero for short high PWM1 pulses. This tests register
        configuration, not measured voltage or a moving servo.
        """
        self.assertEqual(self.arm.read32(TIM15 + 0x20) & 15, 4)
        self.assertEqual(self.arm.read32(TIM15 + 0x44) & (1 << 15), 1 << 15)
        self.assertEqual(self.arm.read32(TIM12 + 0x20), 16)

    def test_diagnostic_reads_hardware_registers_without_changing_outputs(self):
        """Do not mistake cached target acknowledgements for live timer state."""
        for channel, timer, pin, output in ((1, TIM15, 'PE4', 'TIM15_CH1N'), (2, TIM12, 'PH9', 'TIM12_CH2')):
            self.arm.write32(timer, 0)
            self.arm.write32(timer + (0x34 if timer == TIM15 else 0x38), 1777)
            before = bytes(self.arm.uc.mem_read(timer, 0x50))
            self.arm.transmitted.clear()
            self.arm.uc.mem_write(SCRATCH + 4096, encode(200 + channel, f'SERVO DIAG {channel}').rstrip(b'\n') + b'\0')
            self.arm.call('process_line', SCRATCH + 4096)
            reply = decode(bytes(self.arm.transmitted))
            self.assertTrue(reply.ok, reply.body)
            self.assertEqual(reply.fields['pin'], pin)
            self.assertEqual(reply.fields['timer'], output)
            self.assertEqual(reply.fields['cr1'], '0')
            self.assertEqual(reply.fields['ccr'], '1777')
            self.assertEqual(reply.fields['mode'], '2')
            self.assertEqual(reply.fields['af'], '4' if channel == 1 else '2')
            self.assertEqual(bytes(self.arm.uc.mem_read(timer, 0x50)), before)

    def test_diagnostic_measures_pad_pulses_and_detects_stuck_levels(self):
        """Drive GPIO input independently of CCR, then execute the real sampler.

        This validates diagnostic logic, not the physical timer or connector.
        Include the observed inverted 500 us pulse and constant-low/high faults.
        """
        for channel, high_us, inverted, expected_high, levels in (
                (0, 500, False, 500, 3), (1, 2000, False, 2000, 3),
                (0, 500, True, 19500, 3), (1, 0, False, 0, 1), (1, 20000, False, 0, 2)):
            with self.subTest(high_us=high_us, inverted=inverted):
                arm = ArmImage()
                arm.call('servo_init')
                arm.write32(0xE0001000, 1)  # DWT cycle counter enabled
                arm.cycle_step = 40000     # 100 us per sampling iteration

                def pad(uc, access, address, size, value, user):
                    if address == 0xE0001004:
                        phase = (arm.read32(address) // 400) % 20000
                        level = int((phase < high_us) != inverted)
                        arm.write32(0x58021010 if channel == 0 else 0x58021C10, level << (4 if channel == 0 else 9))

                arm.uc.hook_add(UC_HOOK_MEM_READ, pad)
                before = bytes(arm.uc.mem_read(TIM15, 0x50))
                arm.call('servo_diagnostics', channel, SCRATCH, 384)
                body = bytes(arm.uc.mem_read(SCRATCH, 384)).split(b'\0')[0].decode()
                fields = dict(token.split('=', 1) for token in body.split() if '=' in token)
                self.assertEqual(fields['sample'], '1', body)
                self.assertEqual(int(fields['high_us']), expected_high, body)
                self.assertEqual(int(fields['period_us']), 20000 if levels == 3 else 0, body)
                self.assertEqual(int(fields['levels']), levels, body)
                self.assertEqual(fields['gap_us'], '100', body)
                self.assertIn('apb2_clk=', body)  # full report fits the protocol buffer
                self.assertEqual(bytes(arm.uc.mem_read(TIM15, 0x50)), before)

    def test_diagnostic_stalled_cycle_counter_returns_incomplete(self):
        self.arm.write32(0xE0001000, 1)
        self.arm.call('servo_diagnostics', 1, SCRATCH, 384)
        body = bytes(self.arm.uc.mem_read(SCRATCH, 384)).split(b'\0')[0].decode()
        self.assertIn('sample=0 ', body)

    def test_both_outputs_use_pwm1_not_inactive_on_match(self):
        """Check the compiled init's mode fields against STM32 PWM1 encoding.

        A valid CCR value alone cannot produce servo pulses: OCxM=0b0010
        only clears the output on a match; PWM1 requires OCxM=0b0110.
        This checks register programming, not physical output timing.
        """
        for timer in (TIM15, TIM12):
            with self.subTest(timer=timer):
                ccmr1 = self.arm.read32(timer + 0x18)
                ccer = self.arm.read32(timer + 0x20)
                shift = 0 if timer == TIM15 else 8
                self.assertEqual((ccmr1 >> shift) & 3, 0)  # CCxS: output
                mode = ((ccmr1 >> (shift + 4)) & 7) | (((ccmr1 >> (shift + 16)) & 1) << 3)
                self.assertEqual(mode, 6, "PWM1 requires OCxM=0b0110, not 0b0010")
                self.assertEqual((ccmr1 >> (shift + 3)) & 1, 1)  # CCR preload
                self.assertEqual(ccer, 4 if timer == TIM15 else 16)  # CH1N only, normal polarity / CH1 normal

    def test_both_channels_hold_the_home_pulse_from_the_first_period(self):
        # on=1 is truthful: the pulse train is live and each horn is holding at
        # home, even though no SERVO command has been accepted yet.
        for channel, angle, pulse in ((0, 900, 1167), (1, 900, 1500)):
            with self.subTest(channel=channel):
                self.assertEqual(self.servo(channel), (1, angle, pulse, 1))
                self.assertEqual(self.arm.read32((TIM15 + 0x34 if channel == 0 else TIM12 + 0x38)), pulse)

    def test_each_channel_uses_its_own_full_travel_table(self):
        """Exercise every compiled lookup entry and round trip, not a Python copy alone."""
        for channel, limit in ((0, 540), (1, 360)):
            for steps in range(limit + 1):
                pulse = 500 + (2000 * steps + limit // 2) // limit
                self.assertEqual(self.arm.call("servo_set_steps", channel, steps), 1)
                self.assertEqual(self.arm.read32((TIM15 + 0x34 if channel == 0 else TIM12 + 0x38)), pulse)
                self.assertEqual(self.servo(channel), (1, steps * 5, pulse, 1))
            self.assertEqual(self.arm.call("servo_set_steps", channel, limit + 1), 0)

    def test_s2_180_degree_mapping_preserves_s1_travel(self):
        """Verify the framed command path, boundaries and per-channel handshake."""
        def request(sequence, body):
            self.arm.transmitted.clear()
            self.arm.uc.mem_write(SCRATCH + 4096, encode(sequence, body).rstrip(b"\n") + b"\0")
            self.arm.call("process_line", SCRATCH + 4096)
            return decode(bytes(self.arm.transmitted))

        hello = request(80, "HELLO")
        self.assertEqual(hello.fields["servo1_travel"], "2700")
        self.assertEqual(hello.fields["servo2_travel"], "1800")
        self.assertEqual(hello.fields["servo1_pin"], "PE4")
        self.assertEqual(hello.fields["servo2_pin"], "PH9")
        reply = request(79, "SERVO 1 90")
        self.assertEqual(reply.fields["pin"], "PE4")
        self.assertEqual(reply.fields["timer"], "TIM15_CH1N")
        status = request(78, "SERVO STAT")
        self.assertIn("pin=PE4 timer=TIM15_CH1N", status.body)
        self.assertIn("pin=PH9 timer=TIM12_CH2", status.body)
        for seq, angle, pulse in ((81, 45, 1000), (82, 90, 1500), (83, 135, 2000), (84, 180, 2500)):
            reply = request(seq, f"SERVO 2 {angle}")
            self.assertTrue(reply.ok, reply.body)
            self.assertEqual(reply.fields["us"], str(pulse))
            self.assertEqual(reply.fields["angle"], f"{angle}.0")
        self.assertEqual(request(85, "SERVO 2 180.5").fields["code"], "ANGLE_RANGE")
        self.assertEqual(request(86, "SERVO 1 360").fields["code"], "ANGLE_RANGE")
        self.assertEqual(self.servo(0), (1, 900, 1167, 1))
        self.assertEqual(self.servo(1), (1, 1800, 2500, 1))
        self.assertEqual(request(87, "SERVO 2 270").fields["code"], "ANGLE_RANGE")
        self.assertEqual(request(88, "SERVO 2 360").fields["code"], "ANGLE_RANGE")
        self.assertTrue(request(89, "SERVO 1 270").ok)
        # User's two grasping setpoints on the corrected 0..180 degree scale.
        for seq, angle, pulse in ((90, 90, 1500), (91, 120, 1833)):
            reply = request(seq, f"SERVO 2 {angle}")
            self.assertEqual(reply.fields["us"], str(pulse))
            self.assertEqual(reply.fields["angle"], f"{angle}.0")
            self.assertEqual(self.arm.read32(TIM12 + 0x38), pulse)

    def test_every_step_maps_monotonically_inside_the_electrical_range(self):
        previous = 0
        for steps in range(541):
            pulse = self.expected_pulse(steps)
            if steps in (0, 90, 180, 270, 540):
                self.assertEqual(self.arm.call("servo_set_steps", 0, steps), 1)
                self.assertEqual(self.servo(0), (1, steps * 5, pulse, 1))
                self.assertEqual(self.arm.read32(TIM15 + 0x34), pulse)
            self.assertGreaterEqual(pulse, previous)
            self.assertGreaterEqual(pulse, 500)
            self.assertLessEqual(pulse, 2500)
            previous = pulse
        self.assertEqual(self.expected_pulse(0), 500)
        self.assertEqual(self.expected_pulse(540), 2500)

    def test_the_two_channels_are_independent(self):
        """Each horn has its own compare register; one must not drag the other."""
        self.assertEqual(self.arm.call("servo_set_steps", 0, 0), 1)
        self.assertEqual(self.arm.call("servo_set_steps", 1, 360), 1)
        self.assertEqual(self.servo(0), (1, 0, 500, 1))
        self.assertEqual(self.servo(1), (1, 1800, 2500, 1))
        self.assertEqual(self.arm.read32(TIM15 + 0x34), 500)   # S1 -> PE4/TIM15_CH1
        self.assertEqual(self.arm.read32(TIM12 + 0x38), 2500)  # S2 -> PH9/CH2
        # Refusing the second channel leaves the first one where it was.
        self.assertEqual(self.arm.call("servo_set_steps", 1, 361), 0)
        self.assertEqual(self.servo(1), (1, 1800, 2500, 1))

    def test_pulse_path_round_trips_and_picks_the_nearest_step(self):
        self.assertEqual(self.arm.call("servo_set_pulse_us", 0, 1500), 1)
        self.assertEqual(self.servo(0), (1, 1350, 1500, 1))  # exact table entry
        self.assertEqual(self.arm.call("servo_set_pulse_us", 0, 1166), 1)
        self.assertEqual(self.servo(0), (1, 900, 1166, 1))   # 1 us inside entry 180

    def test_out_of_range_or_unknown_channel_is_refused_not_clamped(self):
        self.arm.call("servo_set_steps", 0, 270)
        for steps in (541, 1000):
            self.assertEqual(self.arm.call("servo_set_steps", 0, steps), 0)
        self.assertEqual(self.arm.call("servo_set_steps", 2, 90), 0)
        for us in (499, 2501):
            self.assertEqual(self.arm.call("servo_set_pulse_us", 0, us), 0)
        self.assertEqual(self.servo(0), (1, 1350, 1500, 1))  # untouched by refusals
        self.assertEqual(self.arm.call("servo_status", 2, SCRATCH), 0)
        self.assertEqual(self.arm.call("servo_channels"), 2)

    def test_dispatch_reports_degrees_with_one_decimal(self):
        self.arm.call("motion_init")
        self.arm.write32(0x4001101C, 1 << 7)

        def request(sequence, body):
            self.arm.transmitted.clear()
            self.arm.uc.mem_write(SCRATCH + 4096, encode(sequence, body).rstrip(b"\n") + b"\0")
            self.arm.call("process_line", SCRATCH + 4096)
            return decode(bytes(self.arm.transmitted))

        reply = request(60, "SERVO 1 135")
        self.assertEqual((reply.fields["angle"], reply.fields["us"]), ("135.0", "1500"))
        self.assertEqual(request(61, "SERVO 1 90.0").fields["angle"], "90.0")
        self.assertEqual(request(62, "SERVO 2 45").fields["angle"], "45.0")
        self.assertEqual(request(63, "SERVO 2 180").fields["angle"], "180.0")
        self.assertEqual(request(64, "SERVO 1 270.5").fields["code"], "ANGLE_RANGE")
        self.assertEqual(request(65, "SERVO 1 900").fields["code"], "ANGLE_RANGE")
        # A trailing character must not be silently ignored: "90x" is not 90.
        self.assertEqual(request(66, "SERVO 1 90x").fields["code"], "SERVO_BAD_ARGUMENT")
        self.assertEqual(request(67, "SERVO 1 90.55").fields["code"], "SERVO_BAD_ARGUMENT")
        self.assertEqual(request(68, "SERVO 3 90").fields["code"], "SERVO_BAD_ARGUMENT")
        self.assertEqual(request(69, "SERVO 0 90").fields["code"], "SERVO_BAD_ARGUMENT")
        # STAT answers one frame per channel, separated by a space. The host's
        # field parser keeps the last duplicate key, so channel 2 wins the flat
        # view; both answers must be in the body and the page reads them by hand.
        stat = request(70, "SERVO STAT")
        self.assertEqual(stat.fields["ch"], "2")
        self.assertEqual(stat.fields["angle"], "180.0", "the last channel is the flat view")
        self.assertIn("ch=1", stat.body)
        self.assertIn("ch=2", stat.body)
        self.assertIn("angle=90.0", stat.body, "channel 1 was last set to 90.0 by request 61")
        self.assertIn("angle=180.0", stat.body)
        self.assertEqual(request(71, "SERVO 1").fields["code"], "SERVO_BAD_ARGUMENT")
        self.assertEqual(request(72, "SERVO").fields["code"], "SERVO_BAD_ARGUMENT")
        self.assertEqual(request(73, "SERVO 1 135").fields["angle"], "135.0")  # unchanged by refusals
        self.assertEqual(request(74, "SERVO 2 180").fields["ch"], "2")

    def test_hello_advertises_two_servos_after_the_second_one_was_wired(self):
        hello = self.arm.call("servo_channels")
        self.assertEqual(hello, 2)



if __name__ == "__main__":
    unittest.main()
