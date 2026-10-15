"""Headless client for the H743 protocol: blocking calls, no GUI, no Tkinter.

This is the layer that runs on the Jetson. It reuses host/protocol.py and
host/transport.py unchanged, so the GUI and the on-board agent share exactly
one implementation of framing, retry and the 250 ms heartbeat. Nothing here
opens a port until connect() is called; the protocol version is verified the
same way transport.Connection does for the GUI.

    agent = Agent("COM5")            # or Agent("SIM") for the simulated board
    agent.connect()
    print(agent.request("STAT"))
    agent.submit("LED 1 0")
    agent.close()

Python 3.8 compatible: no str.removesuffix, no match statement.
"""
import argparse
import json
import queue
import sys
import time

from .protocol import Reply
from .transport import Connection

CONNECT_TIMEOUT = 20.0
REPLY_TIMEOUT = 1.5
VALID_EVENTS = ("connected", "disconnected", "error", "reply", "warning")


class AgentError(Exception):
    """Connection lost, handshake rejected, or the board stopped answering."""


class LinkLost(AgentError):
    """The serial worker died: port gone, or a reply never came twice.

    Raised instead of AgentError where the caller can recover by reconnecting,
    so a noisy UART does not have to end the session.
    """


# Commands whose repetition cannot change the outcome, so they are safe to
# resend on a fresh link. Motion commands are deliberately absent: replaying
# MOVE/WHEELS after a reconnect could duplicate a physical movement.
IDEMPOTENT = frozenset((
    "HELLO", "PING", "INFO", "STAT", "AXIS", "LED", "SERVO", "CHASSIS STAT",
))


def is_idempotent(body):
    """True when replaying this body on a new link cannot double an action."""
    head = body.split(" ", 1)[0].upper()
    if head == "CHASSIS":
        return body.strip().upper() == "CHASSIS STAT"
    return head in IDEMPOTENT


class Agent:
    """Synchronous facade over transport.Connection.

    Connection owns the serial port, the heartbeat and the retry rule. This
    class only drains its event queue and matches replies to requests, so the
    firmware sees the same traffic a GUI session would produce.
    """

    def __init__(self, port, connect_timeout=CONNECT_TIMEOUT, reply_timeout=REPLY_TIMEOUT,
                 echo=False):
        self.port = port
        self.connect_timeout = connect_timeout
        self.reply_timeout = reply_timeout
        self.echo = echo
        self.connection = None
        self.events = queue.Queue()
        self._replies = queue.Queue()
        self._pending = {}          # body -> [Reply, ...] not yet claimed
        self._counter = 0
        self.hello = None
        self.warnings = []
        self.link_error = None

    # ---- lifecycle ----------------------------------------------------

    def connect(self):
        """Start the worker and complete HELLO/PING/INFO. Raises on failure."""
        if self.connection is not None:
            raise AgentError("already connected")
        self.connection = Connection(self.port, self.events)
        self.connection.start()
        deadline = time.monotonic() + self.connect_timeout
        while time.monotonic() < deadline:
            event = self._next_event(deadline)
            if event is None:
                break
            kind, payload = event
            if kind == "reply":
                # Handshake replies arrive interleaved with the event this loop
                # is waiting for; store them instead of dropping them.
                self._store_reply(payload)
                continue
            if kind == "connected":
                # transport announces "connected" only after HELLO, PING and
                # INFO all returned OK, so HELLO is already stored above.
                self.hello = self._take("HELLO")
                if self.hello is None:
                    # Never report success without the capability handshake: the
                    # protocol version and dual-serial flags live in this reply.
                    self.close()
                    raise AgentError(
                        "handshake reply for HELLO was lost; board state unknown")
                return self.hello
            if kind in ("error", "disconnected"):
                raise AgentError("connect failed on {}: {}".format(self.port, payload))
        self.close()
        raise AgentError("connect timed out after {:.0f}s on {}".format(
            self.connect_timeout, self.port))

    def close(self, timeout=3.0):
        """Stop the heartbeat and close the port. Sends STOP as the GUI does."""
        if self.connection is None:
            return
        self.connection.close()
        self.connection.join(timeout)
        self.connection = None

    @property
    def is_alive(self):
        """True while the serial worker runs and no link error has arrived.

        Drains first: the worker reports death through the event queue, and a
        caller asking "are we still up?" must not get a stale yes.
        """
        if self.connection is None:
            return False
        self._drain()
        return self.connection.is_alive() and self.link_error is None

    def ensure_link(self, attempts=3, delay=1.0):
        """Reconnect if the worker died. Returns True when a link is usable.

        The board's 1000 ms lease means the axes are already stopped after a
        dropout, and a reconnect re-runs HELLO/PING/INFO, so this can never
        restart motion on its own. It only restores the ability to command.
        """
        if self.is_alive:
            return True
        last = self.link_error
        for attempt in range(1, attempts + 1):
            self.close()
            self._pending.clear()
            self.link_error = None
            self.hello = None
            try:
                self.connect()
                return True
            except AgentError as exc:
                last = str(exc)
                if attempt < attempts:
                    time.sleep(delay)
        self.link_error = last
        return False

    def reconnect(self):
        """Drop the current link and establish a new one. Returns self."""
        self.close()
        self._pending.clear()
        self.link_error = None
        self.hello = None
        self.connect()
        return self

    def run_watchdog(self, seconds=60.0, interval=1.0, report_every=10.0, log=None):
        """Hold the link for `seconds`, reconnecting whenever it drops.

        This is the phase-1 stability test with the STM32 attached: it keeps the
        1000 ms board lease alive through reconnects and reports the board's own
        timeout counter, which is the only honest measure that the Jetson never
        let the lease lapse. Returns a summary dict.
        """
        started = time.monotonic()
        deadline = started + seconds
        log = log or (lambda text: print(text, file=sys.stderr))
        reconnects = 0
        stat_failures = 0
        last_report = started
        final = None

        while time.monotonic() < deadline:
            if not self.is_alive:
                reconnects += 1
                if self.ensure_link():
                    log("link lost; reconnected ({})".format(reconnects))
                else:
                    log("link lost; reconnect failed: {}".format(self.link_error))
            try:
                status = self.fields("STAT")
            except LinkLost:
                status = None
            if status is None:
                stat_failures += 1
            else:
                final = status
            now = time.monotonic()
            if now - last_report >= report_every:
                elapsed = now - started
                log("{:5.1f}s  timeouts={}  reconnects={}  stat_failures={}".format(
                    elapsed,
                    "?" if final is None else final.get("timeout"),
                    reconnects, stat_failures))
                last_report = now
            time.sleep(interval)

        elapsed = time.monotonic() - started
        return {
            "seconds": round(elapsed, 2),
            "reconnects": reconnects,
            "stat_failures": stat_failures,
            "alive": self.is_alive,
            "board_timeouts": None if final is None else final.get("timeout"),
            "bad": None if final is None else final.get("bad"),
            "rx_err": None if final is None else final.get("rx_err"),
            "armed": None if final is None else final.get("armed"),
        }

    def request_resilient(self, body, attempts=2, timeout=None, expect_ok=False):
        """request() that survives a link loss for idempotent commands only.

        A non-idempotent body (motion) is attempted exactly once and raises
        LinkLost instead: replaying it on a fresh link could duplicate a
        physical movement, and the board's dedup cache is gone after the gap.
        """
        if attempts > 1 and not is_idempotent(body):
            raise ValueError("refusing to retry non-idempotent command {!r}".format(body))
        last = None
        for attempt in range(1, attempts + 1):
            try:
                return self.request(body, timeout=timeout, expect_ok=expect_ok)
            except LinkLost as exc:
                last = exc
                if attempt < attempts and self.ensure_link():
                    continue
                raise
        raise last

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ---- event plumbing ----------------------------------------------

    def _next_event(self, deadline):
        """Return the next event, or None once the deadline passes."""
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                return self.events.get(timeout=min(remaining, .1))
            except queue.Empty:
                continue

    def _store_reply(self, payload):
        """Record one ("reply") payload.

        Payloads are (body, reply) or (body, reply, tag). A tagged reply belongs
        to exactly one caller request; an untagged one was sent by the transport
        itself (handshake or heartbeat PING) or by submit().
        """
        body, reply = payload[0], payload[1]
        tag = payload[2] if len(payload) > 2 else None
        key = tag if tag is not None else body
        queued = self._pending.setdefault(key, [])
        queued.append(reply)
        # Untagged traffic is never claimed by request(), so bound its growth.
        if tag is None and len(queued) > 4:
            del queued[:-4]
        if self.echo:
            print("[rx] {} -> {}".format(body, reply.body), file=sys.stderr)

    def _next_tag(self):
        self._counter += 1
        return "{}#{}".format(self.port, self._counter)

    def _drain(self):
        """Move every available event into the pending-reply map."""
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                return
            if kind == "reply":
                self._store_reply(payload)
            elif kind == "warning":
                self.warnings.append(payload)
                print("[warn] {}".format(payload), file=sys.stderr)
            elif kind == "error":
                # Remembered rather than raised: request() and ensure_link()
                # decide whether to reconnect, and every other call site can
                # keep draining without an exception escaping mid-loop.
                self.link_error = str(payload)
            elif kind == "disconnected":
                self.link_error = self.link_error or "port closed"
            elif kind == "connected":
                self.hello = None

    def _take(self, key):
        """Pop the oldest unclaimed reply stored under this key, if any.

        Keys are request tags for caller commands and bare bodies for untagged
        traffic. A reply is only ever returned once, so a heartbeat answer can
        never stand in for a caller's request even when both are "PING".
        """
        queued = self._pending.get(key)
        while queued:
            reply = queued.pop(0)
            if isinstance(reply, Reply):
                if not queued:
                    del self._pending[key]
                return reply
        if key in self._pending:
            del self._pending[key]
        return None

    # ---- request / reply ---------------------------------------------

    def request(self, body, timeout=None, expect_ok=False):
        """Send one command and return its Reply, or None on timeout.

        The body is sent verbatim: no parsing, no validation, no retry of our
        own. Connection already retries once with identical bytes and ID. The
        reply is matched by a unique tag, never by the command text, so an
        answer to the transport's own heartbeat can never be mistaken for it.
        """
        if self.connection is None or not self.connection.is_alive():
            raise LinkLost("serial worker is not running on {}".format(self.port))
        self._drain()
        if self.link_error is not None:
            raise LinkLost("link to {} is down: {}".format(self.port, self.link_error))
        # An untagged reply for the same body belongs to an earlier submit();
        # drop it now so it cannot masquerade as this command's answer.
        self._take(body)
        tag = self._next_tag()
        if not self.connection.submit(body, tag=tag):
            raise AgentError("command queue full, dropped {!r}".format(body))
        limit = self.reply_timeout if timeout is None else timeout
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            self._drain()
            reply = self._take(tag)
            if reply is not None:
                if expect_ok and not reply.ok:
                    raise AgentError("{} rejected: {}".format(body, reply.body))
                return reply
            time.sleep(.005)
        if self.link_error is not None:
            raise LinkLost("link to {} dropped: {}".format(self.port, self.link_error))
        raise AgentError("no reply to {!r} within {:.2f}s".format(body, limit))

    def submit(self, body, tag=None):
        """Queue a command without waiting for its reply (fire and forget).

        Its reply arrives untagged under the command body and is collected by
        drain_for(). Without a tag there is no way to tell two submits of the
        same body apart, so use request() when the answer must be matched.
        """
        if self.connection is None or not self.connection.is_alive():
            raise LinkLost("serial worker is not running on {}".format(self.port))
        return self.connection.submit(body, tag=tag)

    def drain_for(self, seconds):
        """Collect replies for commands sent with submit(). Returns a count."""
        collected = 0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._drain()
            collected = sum(len(v) for v in self._pending.values())
            time.sleep(.01)
        return collected

    # ---- typed helpers ------------------------------------------------

    def fields(self, body, timeout=None):
        """Return the reply as a dict of key=value fields, or None if unanswered.

        A timeout yields None so polling loops can keep trying; a lost link
        still raises AgentError, because retrying that is pointless.
        """
        try:
            reply = self.request(body, timeout=timeout)
        except AgentError as exc:
            if "no reply to" in str(exc):
                return None
            raise
        result = reply.fields
        result["_ok"] = "1" if reply.ok else "0"
        result["_body"] = reply.body
        result["_sequence"] = str(reply.sequence)
        return result

    def poll_until(self, body, key, predicate, timeout=10.0, interval=.2):
        """Poll an idempotent query until predicate(fields[key]) is true.

        Returns the last status dict, or None on timeout. Query commands may be
        repeated safely; motion commands must never be retried this way.
        """
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            status = self.fields(body)
            if status is not None:
                last = status
                if key in status:
                    try:
                        value = int(status[key])
                    except ValueError:
                        value = status[key]
                    if predicate(value):
                        return status
            time.sleep(interval)
        return last


def format_reply(reply):
    if reply is None:
        return "TIMEOUT"
    return "seq={} {}".format(reply.sequence, reply.body)


def run_script(agent, path, as_json=False):
    """Run one command per line; '#' comments and blank lines are skipped."""
    results = []
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            body = raw.strip()
            if not body or body.startswith("#"):
                continue
            reply = agent.request(body)
            results.append({"command": body, "reply": None if reply is None else reply.body,
                            "sequence": None if reply is None else reply.sequence})
            if not as_json:
                print("{:<40} {}".format(body, format_reply(reply)))
    return results


def repl(agent, as_json=False):
    print("connected on {} — one command per line, 'quit' to exit".format(agent.port))
    print("examples: HELLO | PING | INFO | STAT | LED 1 0 | CHASSIS STAT")
    while True:
        try:
            body = input("h743> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if body in ("quit", "exit"):
            return
        if not body:
            continue
        try:
            reply = agent.request(body)
        except AgentError as exc:
            print("error: {}".format(exc))
            return
        if as_json:
            print(json.dumps({"command": body,
                              "reply": None if reply is None else reply.body}))
        else:
            print(format_reply(reply))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Headless H743 protocol client for the Jetson (no GUI).")
    parser.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--demo", action="store_true",
                        help="use the simulated board instead of a real port")
    parser.add_argument("--script", help="run commands from a file, then exit")
    parser.add_argument("--json", action="store_true", help="print replies as JSON")
    parser.add_argument("--check", action="store_true",
                        help="handshake and heartbeat check only, then exit")
    parser.add_argument("--heartbeat-seconds", type=float, default=0.0,
                        help="with --check, keep pinging this long to prove stability")
    parser.add_argument("--watchdog-seconds", type=float, default=0.0,
                        help="hold the link this long, reconnecting if it drops, "
                             "and report the board's timeout counter")
    parser.add_argument("--echo", action="store_true", help="log every reply to stderr")
    args = parser.parse_args(argv)

    port = "SIM" if args.demo else args.port
    if not port:
        parser.error("give --port or --demo")

    agent = Agent(port, echo=args.echo)
    try:
        agent.connect()
    except AgentError as exc:
        print("FAILED: {}".format(exc), file=sys.stderr)
        return 2

    hello = agent.hello
    summary = {"port": port, "connected": True,
               "hello": None if hello is None else hello.body}
    if not args.json:
        print("connected: {}".format(port))
        print("HELLO: {}".format(summary["hello"]))
    else:
        print(json.dumps(summary))

    status = 0
    try:
        if args.watchdog_seconds > 0:
            summary = agent.run_watchdog(args.watchdog_seconds)
            print(json.dumps(summary) if args.json else
                  "watchdog: {seconds}s, reconnects={reconnects}, "
                  "stat_failures={stat_failures}, board timeout={board_timeouts}, "
                  "bad={bad}, rx_err={rx_err}".format(**summary))
            if summary["board_timeouts"] not in (None, "0") or summary["stat_failures"]:
                status = 1
        elif args.heartbeat_seconds > 0:
            started = time.monotonic()
            pings = 0
            failures = 0
            while time.monotonic() - started < args.heartbeat_seconds:
                try:
                    reply = agent.request("PING")
                    if not reply.ok:
                        failures += 1
                except AgentError as exc:
                    if "no reply to" not in str(exc):
                        raise
                    failures += 1
                pings += 1
                time.sleep(.25)
            elapsed = time.monotonic() - started
            stat = agent.fields("STAT")
            report = {"pings": pings, "failures": failures,
                      "seconds": round(elapsed, 2),
                      "board_timeouts": None if stat is None else stat.get("timeout"),
                      "armed": None if stat is None else stat.get("armed")}
            print(json.dumps(report) if args.json else
                  "heartbeat: {} pings, {} failures, {:.1f}s, board timeout={} armed={}".format(
                      pings, failures, elapsed, report["board_timeouts"], report["armed"]))
            if failures:
                status = 1
        elif args.script:
            json_results = run_script(agent, args.script, args.json)
            if args.json:
                print(json.dumps(json_results, indent=2))
        elif args.check:
            pass
        else:
            repl(agent, args.json)
    except AgentError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        status = 3
    finally:
        agent.close()
    return status


if __name__ == "__main__":
    sys.exit(main())
