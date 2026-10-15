#!/usr/bin/env python3
"""Exercise the real pyserial code path without an STM32 attached.

Two processes are joined by a pseudo-terminal: this one emulates the board on
the PTY master, and a child runs host.agent against the matching /dev/pts/N
slave. Because the child sees an ordinary character device, the code under test
is serial.Serial itself -- open(), 115200 8N1, read timeouts, partial frames --
which SimulatedSerial never touches. It proves the box-side plumbing, not the
board: electrical levels, the CH340X and the real UART still need hardware.

    python3 tests/pty_link_check.py
"""
import argparse
import os
import pty
import subprocess
import sys
import termios
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.protocol import Framer, decode, encode          # noqa: E402
from host.simulator import SimulatedBoard                 # noqa: E402

PING_EVERY = .25          # mirrors transport.Connection's heartbeat


def run_board(master_fd, seconds):
    """Serve the demo board over the PTY master until the deadline."""
    board = SimulatedBoard()
    framer = Framer()
    deadline = time.monotonic() + seconds
    last_ping = 0.0
    reply_count = 0
    while time.monotonic() < deadline:
        try:
            chunk = os.read(master_fd, 4096)
        except OSError:
            break
        if not chunk:
            time.sleep(.01)
            continue
        for frame in framer.feed(chunk):
            answer = board.receive(frame)
            if answer:
                os.write(master_fd, answer)
                reply_count += 1
        now = time.monotonic()
        if now - last_ping >= PING_EVERY:
            os.write(master_fd, board.receive(encode(1, "PING")))
            last_ping = now
            reply_count += 1
    return board, reply_count


def run_checks(port, seconds):
    """Drive the agent against `port` in a child process, in three stages."""
    stages = (
        ("handshake + link rehearsal", ["-m", "host.verify_link", "--port", port,
                                        "--seconds", str(seconds)]),
        ("idempotent requests on a live link", ["-m", "host.agent", "--port", port,
                                                "--script", "-"]),
        ("lease watchdog", ["-m", "host.agent", "--port", port, "--json",
                            "--watchdog-seconds", str(seconds)]),
    )
    results = []
    for name, argv in stages:
        if argv[-1] == "-":                      # script mode reads a file
            handle = Path("/tmp/pty_cmds.txt")
            handle.write_text("HELLO\nPING\nINFO\nSTAT\nAXIS 5\nCHASSIS STAT\n",
                              encoding="ascii")
            argv = argv[:-1] + [str(handle)]
        proc = subprocess.run([sys.executable] + argv, cwd=str(ROOT),
                              capture_output=True, text=True, timeout=seconds + 60)
        results.append((name, proc.returncode, (proc.stdout + proc.stderr).strip()))
    return results


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=6.0,
                        help="how long each stage may run")
    args = parser.parse_args(argv)

    master_fd, slave_fd = pty.openpty()
    # Raw both ways: no echo, no CR/LF translation, no signal characters. The
    # kernel would otherwise echo our own writes back into the protocol stream.
    for fd in (master_fd, slave_fd):
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0                                        # iflag
        attrs[1] = 0                                        # oflag
        attrs[2] = termios.CREAD | termios.CS8              # lflag
        attrs[3] = 0                                        # cflag
        attrs[6] = [b"\x00"] * len(attrs[6])                # cc, all disabled
        attrs[6][termios.VMIN] = b"\x00"
        attrs[6][termios.VTIME] = b"\x00"
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    slave_name = os.ttyname(slave_fd)
    # Keep slave_fd open for the whole run: once the last slave descriptor
    # closes, the kernel hangs up the master and the channel stops carrying
    # bytes. Handing the name to the child is not enough on its own.
    print("PTY slave offered to the agent: {}".format(slave_name))

    pid = os.fork()
    if pid == 0:                                            # child: emulate board
        try:
            board, replies = run_board(master_fd, args.seconds * 4 + 30)
            os._exit(0)
        except BaseException:
            os._exit(1)

    summary = {}
    try:
        time.sleep(.3)                                      # let the child settle
        for name, code, output in run_checks(slave_name, args.seconds):
            print("\n=== {} ===\n[exit {}]".format(name, code))
            print("\n".join(output.splitlines()[-12:]))
            summary[name] = (code, output)
    finally:
        try:
            os.kill(pid, 15)
        except ProcessLookupError:
            pass
        os.waitpid(pid, 0)
        os.close(slave_fd)
        os.close(master_fd)

    failed = [name for name, (code, _) in summary.items() if code != 0]
    print("\n---")
    if failed:
        print("FAILED stages: {}".format(", ".join(failed)))
        return 1
    print("all {} stages passed over a real pty-backed serial device "
          "({})".format(len(summary), slave_name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
