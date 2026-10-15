#!/usr/bin/env python3
"""Hold the board lease while the box is deliberately overloaded.

The phase-1 risk is not an idle link: it is whether the 250 ms heartbeat still
gets scheduled when the Jetson is busy with vision work. The board stops the
axes by itself 1000 ms after the last PING, so a starved heartbeat thread shows
up as a non-zero `timeout` counter in STAT — that counter is the acceptance
signal, not our own request latency.

Load is made by `ops` CPU spinners, each with its own saturating integer loop,
so no external tool (stress-ng is absent on this box) is required.

    python3 tests/lease_stress.py --seconds 60                  # 6 workers on 6 cores
    python3 tests/lease_stress.py --seconds 60 --ops 12         # 2x oversubscribed
    python3 tests/lease_stress.py --seconds 30 --port /dev/ttyUSB0
    python3 tests/lease_stress.py --seconds 30 --no-load       # control run
"""
import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.agent import Agent, AgentError, LinkLost          # noqa: E402


def spin_forever():
    """Saturate one core until SIGTERM. Integer and float work, no syscalls."""
    total = 0
    x = 1.0001
    while True:
        for _ in range(20000):
            total += 1
            x = x * 1.0000001
            if x > 2.0:
                x = 1.0001
        if total < 0:                       # never true; keeps the work honest
            break


def spawn_load(ops):
    """Fork `ops` spinners. Returns the child pids."""
    pids = []
    for index in range(ops):
        pid = os.fork()
        if pid == 0:
            try:
                if hasattr(os, "sched_setaffinity"):
                    os.sched_setaffinity(0, {index % (os.cpu_count() or 1)})
                spin_forever()
            except BaseException:
                pass
            os._exit(0)
        pids.append(pid)
    return pids


def stop_load(pids):
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for pid in pids:
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass


def read_loadavg():
    try:
        with open("/proc/loadavg") as handle:
            return handle.read().split()[0]
    except OSError:
        return "?"


def gil_worker(stop_event):
    """Hold the interpreter lock in long pure-Python bursts.

    This is the failure mode CPU load alone does not produce, and the one real
    vision code can: a module that does sustained Python-level work between
    releasing the GIL lets the heartbeat thread starve even on an idle machine.
    """
    total = 0
    while not stop_event.is_set():
        for _ in range(3000000):            # roughly 100-200 ms per burst
            total += 1
        if total < 0:
            break


def cv_worker(stop_event, size=1080, stats=None):
    """Occupy the box the way a vision node does: C-level array work per frame.

    A pure-Python spinner releases the GIL often; OpenCV and numpy calls hold it
    for the whole C routine. If any single call exceeds the 1000 ms lease, the
    heartbeat cannot be sent at all no matter how it is scheduled -- that is the
    failure mode this worker exists to expose, so per-frame time is recorded.
    """
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        print("cv load unavailable: {}".format(exc), file=sys.stderr)
        return
    if stats is None:
        stats = {"frames": 0, "total_s": 0.0, "max_frame_s": 0.0}
    frame = np.zeros((size, size, 3), dtype=np.uint8)
    cv2.circle(frame, (size // 2, size // 2), size // 4, (255, 255, 255), -1)
    while not stop_event.is_set():
        started = time.monotonic()
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey, 100, 200)
        blur = cv2.GaussianBlur(edges, (21, 21), 0)
        _, mask = cv2.threshold(blur, 25, 255, cv2.THRESH_BINARY)
        cv2.countNonZero(mask)
        elapsed = time.monotonic() - started
        stats["frames"] += 1
        stats["total_s"] += elapsed
        stats["max_frame_s"] = max(stats["max_frame_s"], elapsed)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="serial device; omit to use the demo board")
    parser.add_argument("--demo", action="store_true", help="force the simulated board")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--ops", type=int, default=0,
                        help="CPU spinners to fork (default: one per core)")
    parser.add_argument("--no-load", action="store_true",
                        help="control run: watchdog only, no spinners")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="seconds to overload the box before the lease starts")
    parser.add_argument("--gil-threads", type=int, default=0,
                        help="in-process threads doing long pure-Python bursts; "
                             "this is what starves the heartbeat on a desktop-class CPU")
    parser.add_argument("--cv-threads", type=int, default=0,
                        help="in-process threads running OpenCV/numpy work per frame, "
                             "which holds the GIL for whole C routines")
    parser.add_argument("--cv-size", type=int, default=1080,
                        help="square frame size for --cv-threads (1080 = ~1080p)")
    parser.add_argument("--lower-priority", action="store_true",
                        help="run the agent (and spinners) at nice +10 so free cores "
                             "always beat it for CPU")
    args = parser.parse_args(argv)

    if not hasattr(os, "fork"):
        # Windows has no fork, and this test belongs on the Jetson anyway: the
        # question it answers is whether that box keeps its lease under load.
        print("this test needs fork(); run it on the Jetson, not on Windows")
        return 2

    port = "SIM" if (args.demo or not args.port) else args.port
    ops = 0 if args.no_load else (args.ops or (os.cpu_count() or 1))

    agent = Agent(port)
    try:
        agent.connect()
    except AgentError as exc:
        print("cannot connect: {}".format(exc))
        return 2

    pids = []
    gil_stop = threading.Event()
    gil_threads = []
    exit_code = 0
    try:
        if args.lower_priority:
            os.nice(10)
            print("agent and load run at nice +10")
        if ops:
            print("overloading {} core(s) with {} spinner(s)".format(
                os.cpu_count(), ops))
            pids = spawn_load(ops)
        for index in range(args.gil_threads):
            thread = threading.Thread(target=gil_worker, args=(gil_stop,),
                                      name="gil-hog-{}".format(index), daemon=True)
            thread.start()
            gil_threads.append(thread)
        cv_stats = {"frames": 0, "total_s": 0.0, "max_frame_s": 0.0}
        for index in range(args.cv_threads):
            thread = threading.Thread(target=cv_worker,
                                      args=(gil_stop, args.cv_size, cv_stats),
                                      name="cv-hog-{}".format(index), daemon=True)
            thread.start()
            gil_threads.append(thread)
        if gil_threads:
            print("{} in-process thread(s) contending for the GIL "
                  "({} python bursts, {} opencv)".format(
                      len(gil_threads), args.gil_threads, args.cv_threads))
        if ops or gil_threads:
            time.sleep(args.settle)

        # Measure the gaps that actually matter: the board's lease is refreshed
        # when a PING arrives on the wire, so the clock that counts is the
        # moment bytes leave the port, not the moment a reply is read back.
        writes = []
        serial_obj = getattr(agent.connection, "serial", None)
        if serial_obj is not None:
            original_write = serial_obj.write

            def timed_write(data, _write=original_write, _writes=writes):
                # Stamp before the write so the recorded time is never later
                # than the moment the bytes were handed to the driver.
                _writes.append((time.monotonic(), len(data)))
                return _write(data)

            serial_obj.write = timed_write

        print("loadavg before lease: {}".format(read_loadavg()))
        print("board STAT before:    {}".format(
            (agent.request_resilient("STAT") or "?").body.split("cmd=STAT")[-1].strip()[:120]))

        # The watchdog itself runs STAT polls and keeps the heartbeat alive, so
        # a starved scheduler hits both. LinkLost is what a dropped lease looks
        # like from here; the board's own counter is the authoritative record.
        summary = agent.run_watchdog(args.seconds, interval=1.0, report_every=10.0)
        summary["loadavg_after"] = read_loadavg()
        summary["ops"] = ops
        summary["gil_threads"] = len(gil_threads)
        if args.cv_threads:
            frames = cv_stats["frames"]
            summary["cv_frames"] = frames
            summary["cv_mean_frame_ms"] = (
                round(1000.0 * cv_stats["total_s"] / frames, 1) if frames else None)
            summary["cv_max_frame_ms"] = round(1000.0 * cv_stats["max_frame_s"], 1)
            summary["cv_fps_total"] = (
                round(frames / max(summary["seconds"], .001), 1) if frames else None)
        if len(writes) >= 2:
            stamps = [moment for moment, _ in writes]
            gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
            summary["tx_frames"] = len(writes)
            summary["tx_max_gap_s"] = round(max(gaps), 3)
            summary["tx_over_900ms"] = sum(1 for gap in gaps if gap > .9)
        else:
            summary["tx_frames"] = len(writes)
            summary["tx_max_gap_s"] = None
            summary["tx_over_900ms"] = None
        print("\n" + json.dumps(summary))

        timeouts = summary.get("board_timeouts")
        if timeouts not in (None, "0"):
            print("\nFAIL: the board recorded {} lease timeout(s) under load — "
                  "the heartbeat missed 1000 ms.".format(timeouts))
            exit_code = 1
        elif summary.get("stat_failures"):
            print("\nFAIL: {} STAT poll(s) got no answer.".format(summary["stat_failures"]))
            exit_code = 1
        else:
            print("\nPASS: {}s with {} spinner(s) and {} GIL thread(s), "
                  "board timeouts=0, reconnects={}, stat_failures=0, "
                  "tx max gap {}s over {} frames ({} above 900 ms)".format(
                      summary["seconds"], ops, len(gil_threads),
                      summary["reconnects"], summary["tx_max_gap_s"],
                      summary["tx_frames"], summary["tx_over_900ms"]))
    except LinkLost as exc:
        print("\nlink lost during the run: {}".format(exc))
        exit_code = 1
    finally:
        gil_stop.set()
        for thread in gil_threads:
            thread.join(timeout=2.0)
        if pids:
            stop_load(pids)
            print("load stopped ({} worker(s))".format(len(pids)))
        agent.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
