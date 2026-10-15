#!/usr/bin/env python3
"""Phase-1 rehearsal: drive the agent the way the vision code will, on any port.

Runs a longer session with continuous heartbeats and checks that the board
never recorded a lease timeout. Safe to point at the simulated board today and
at /dev/ttyUSB0 once the STM32 is wired up.

    python3 verify_link.py --demo
    python3 verify_link.py --port /dev/ttyUSB0
"""
import argparse
import json
import sys
import time

from host.agent import Agent, AgentError


def step(name, ok, detail=""):
    print("{:<34} {} {}".format(name, "PASS" if ok else "FAIL", detail))
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="how long to keep the session running")
    args = parser.parse_args(argv)

    port = "SIM" if args.demo else args.port
    if not port:
        parser.error("give --port or --demo")

    results = []
    agent = Agent(port)
    started = time.monotonic()
    try:
        agent.connect()
    except AgentError as exc:
        print("cannot connect: {}".format(exc))
        return 2

    try:
        hello = agent.hello
        results.append(step("handshake protocol=1",
                            hello is not None and hello.fields.get("protocol") == "1"))
        info = agent.fields("INFO")
        results.append(step("INFO readable", info is not None and info["_ok"] == "1",
                            "cpu={}".format(info.get("cpu_hz", "?")) if info else ""))

        # LED round trip: proves a command actually reaches the board and the
        # reply carries the board's own state, not an echo of the request.
        led_on = agent.request("LED 1 0")
        results.append(step("LED write acknowledged",
                            led_on is not None and led_on.ok and led_on.fields.get("led") == "1",
                            "" if led_on is None else led_on.body))
        led_off = agent.request("LED 0 0")
        results.append(step("LED off acknowledged",
                            led_off is not None and led_off.ok
                            and led_off.fields.get("led") == "0"))

        # Arm needs a fresh lease, so this also proves the heartbeat is live.
        arm = agent.request("ARM 1")
        results.append(step("ARM granted after heartbeat",
                            arm is not None and arm.ok, "" if arm is None else arm.body))

        # Continuous status polling for the whole window, with the heartbeat
        # running underneath. Any lease miss shows up in STAT timeout.
        polls = 0
        failures = 0
        deadline = started + args.seconds
        while time.monotonic() < deadline:
            stat = agent.fields("STAT")
            polls += 1
            if stat is None or stat["_ok"] != "1":
                failures += 1
            time.sleep(.25)

        final = agent.fields("STAT")
        results.append(step("continuous polling", failures == 0,
                            "{} polls, {} failures".format(polls, failures)))
        results.append(step("board recorded no lease timeout",
                            final is not None and final["timeout"] == "0",
                            "timeout=" + (final or {}).get("timeout", "?")))
        results.append(step("error counters clean",
                            final is not None and final["bad"] == "0"
                            and final["rx_err"] == "0",
                            "bad={} rx_err={}".format((final or {}).get("bad"),
                                                      (final or {}).get("rx_err"))))

        agent.request("STOP", expect_ok=True)
        stopped = agent.fields("STAT")
        results.append(step("STOP disarms", stopped is not None and stopped["armed"] == "0"))
    except AgentError as exc:
        results.append(step("session", False, str(exc)))
    finally:
        elapsed = time.monotonic() - started
        agent.close()

    passed = sum(1 for r in results if r)
    print("\n{}/{} checks passed in {:.1f}s on {}".format(passed, len(results), elapsed, port))
    print(json.dumps({"port": port, "passed": passed, "total": len(results)}))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
