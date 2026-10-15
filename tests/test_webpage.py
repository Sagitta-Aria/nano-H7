"""Static contract checks for the web console's front end.

There is no headless browser in this environment, so the page is verified by
contract instead of by pixels: every element the script reaches for must exist in
the markup, every CSS class the markup asks for must be styled, and every API
route the script calls must exist on the server. That catches the failure mode of
a renamed id or a typo'd selector turning a control silently dead — which no
Python test would otherwise notice.
"""
import re
import unittest
from pathlib import Path

from host import webserver
from tests.chassis_fixture import MATRIX as FIRMWARE_MATRIX

HOST = Path(__file__).resolve().parents[1] / "host"
INDEX = (HOST / "static" / "index.html").read_text(encoding="utf-8")
SCRIPT = (HOST / "static" / "app.js").read_text(encoding="utf-8")
STYLE = (HOST / "static" / "styles.css").read_text(encoding="utf-8")
SERVER = (HOST / "webserver.py").read_text(encoding="utf-8")


def html_ids():
    return set(re.findall(r'id="([^"]+)"', INDEX))


def classes_in(markup):
    found = set()
    for attribute in re.findall(r'class="([^"]+)"', markup):
        found.update(attribute.split())
    return found


class MarkupContract(unittest.TestCase):
    def test_every_looked_up_id_exists(self):
        wanted = set(re.findall(r'\$\("([^"]+)"\)', SCRIPT))
        missing = sorted(wanted - html_ids())
        self.assertEqual(missing, [], f"app.js looks up ids that index.html does not define: {missing}")

    def test_one_gauge_card_per_wired_servo(self):
        """Two servos are wired, so the page must offer two independent panels."""
        channels = re.findall(r'class="card servo-card" data-ch="(\d)"', INDEX)
        self.assertEqual(channels, ["1", "2"])
        # Per-card controls are addressed by class, never by a global id, or the
        # second servo would silently drive the first one's input.
        for marker in (".angle-input", ".servo-send", ".presets", ".gauge", ".arc-live"):
            self.assertIn(f'querySelector("{marker}")', SCRIPT, marker)
        for stale in ('id="servo-input"', 'id="servo-range"', 'id="servo-send"',
                      'id="servo-angle"', 'id="arc-live"', 'id="needle"'):
            self.assertNotIn(stale, INDEX, f"{stale} would serve only one servo")

    def test_servo_status_parser_reads_every_channel(self):
        """The firmware packs one frame per channel into a single reply body."""
        pattern = re.search(r"const SERVO_FRAME = (/[^/]+/g);", SCRIPT)
        self.assertIsNotNone(pattern)
        grammar = re.compile(pattern.group(1)[1:-2])
        body = ("OK cmd=SERVO_STAT ch=1 angle=135.0 us=1500 on=1 "
                "OK cmd=SERVO_STAT ch=2 angle=270.0 us=2500 on=1")
        found = {int(m.group(1)): (m.group(2), m.group(3), m.group(4)) for m in grammar.finditer(body)}
        self.assertEqual(found, {1: ("135.0", "1500", "1"), 2: ("270.0", "2500", "1")})
        # A single-channel firmware reply must still parse.
        one = re.search(grammar, "OK cmd=SERVO_STAT ch=1 angle=90.0 us=1167 on=1")
        self.assertEqual(one.group(2), "90.0")

    def test_every_static_asset_is_served_or_referenced(self):
        for name in ("index.html", "styles.css", "app.js", "camera.js"):
            self.assertTrue((HOST / "static" / name).is_file(), name)
        self.assertIn('href="/styles.css"', INDEX)
        self.assertIn('src="/app.js"', INDEX)
        self.assertIn('from "/camera.js"', SCRIPT)

    def test_camera_page_matches_module_controls(self):
        """The capture module binds these controls before the first tab visit."""
        for name in ("source", "device", "refresh", "start", "stop", "resolution", "apply",
                     "snapshot", "mirror", "actual", "state", "message", "video", "image", "placeholder", "bind", "identity"):
            self.assertIn(f"camera-{name}", html_ids())
            self.assertIn(f"camera2-{name}", html_ids())
        self.assertIn('data-tab="camera"', INDEX)
        self.assertIn('id="tab-camera"', INDEX)
        self.assertIn('id="tab-camera2"', INDEX)
        self.assertEqual(len(re.findall(r'id="([^"]+)"', INDEX)), len(html_ids()))
        self.assertIn('autoplay muted playsinline', INDEX)

    def test_markup_classes_are_styled(self):
        """A class used in markup but absent from the sheet means unstyled UI."""
        used = classes_in(INDEX)
        known = set(re.findall(r"\.([a-zA-Z][\w-]*)", STYLE))
        # Styled through a descendant selector, driven by JS, or a layout helper.
        intentional = {"grow", "hidden", "inv", "fixed", "servo-card"}
        missing = sorted(used - known - intentional)
        self.assertEqual(missing, [], f"classes with no rule in styles.css: {missing}")
        for name in ("inv", "fixed", "servo-card"):
            self.assertIn(name, used, f"{name} is unused; drop it from the allow-list")

    def test_no_colour_is_hardcoded_outside_the_palettes(self):
        """Every surface colour must come from a variable, or one theme breaks."""
        components = STYLE.split("* { box-sizing: border-box; }", 1)[1]
        offenders = [line.strip() for line in components.splitlines()
                     if re.search(r"#[0-9a-fA-F]{3,8}\b", line)]
        self.assertEqual(offenders, [], f"hardcoded colours would not follow the theme: {offenders}")

    def test_both_themes_define_the_same_variables(self):
        """A variable present in one palette and missing in the other renders as
        an invalid value, which silently drops the declaration."""
        palettes = re.findall(r":root\[data-theme=\"(\w+)\"\]\s*\{(.*?)\n\}", STYLE, re.S)
        self.assertEqual(sorted(name for name, _ in palettes), ["dark", "light"])
        sets = {name: set(re.findall(r"(--[\w-]+):", block)) for name, block in palettes}
        self.assertEqual(sorted(sets["light"] - sets["dark"]),
                         [], "light-only variables")
        self.assertEqual(sorted(sets["dark"] - sets["light"]),
                         [], "dark-only variables")
        self.assertIn("--bg", sets["light"])

    def test_light_is_the_default_and_the_choice_persists(self):
        self.assertIn('data-theme="light"', STYLE)
        self.assertIn("h743-theme", INDEX, "the saved theme must be read before paint")
        self.assertIn("h743-theme", SCRIPT, "the theme switch must remember the choice")
        # The inline script must be synchronous and in <head>: it has to set the
        # attribute before the first paint, or the page flashes the other theme.
        head = INDEX.split("</head>", 1)[0]
        self.assertIn("h743-theme", head, "the theme must be applied from <head>")
        self.assertNotIn("defer", head.split("h743-theme")[0].rsplit("<script", 1)[-1])

    def test_chassis_directions_match_the_firmware_matrix(self):
        """The wheel schematic must show the same signs the board actually drives.

        The expected values come from the ARM-side test matrix, so the page, the
        simulator and the firmware cannot drift apart silently.
        """
        found = dict(re.findall(r"(\w+):\s*\{[^}]*?wheels:\s*\[([^\]]+)\]", SCRIPT))
        self.assertEqual(sorted(found), sorted(FIRMWARE_MATRIX), "every direction needs a schematic")
        for name, signs in FIRMWARE_MATRIX.items():
            parsed = tuple(int(part.strip()) for part in found[name].split(","))
            self.assertEqual(parsed, signs, name)

    def test_direction_buttons_cover_every_direction(self):
        buttons = set(re.findall(r'data-dir="([^"]+)"', INDEX))
        self.assertEqual(buttons, {"FWD", "BACK", "LEFT", "RIGHT", "CCW", "CW", "HALT"})

    def test_angle_input_matches_the_firmware_grammar(self):
        """The page must accept exactly what parse_degrees() accepts."""
        pattern = re.search(r"const ANGLE_RE = (/[^/]+/);", SCRIPT)
        self.assertIsNotNone(pattern)
        self.assertEqual(pattern.group(1), r"/^\d{1,3}(\.\d)?$/")
        grammar = re.compile(pattern.group(1)[1:-1])
        for accepted in ("0", "90", "135", "270", "13.5", "270.0", "0.5"):
            self.assertIsNotNone(grammar.match(accepted), accepted)
        for refused in ("", "270.5.1", "1234", "90x", "9 0", "90,5", "-45", "1e2"):
            self.assertIsNone(grammar.match(refused), refused)


class RouteContract(unittest.TestCase):
    def page_routes(self):
        called = set(re.findall(r'(?:post|fetch)\("(/api/[a-z]+)"', SCRIPT))
        # The stream is opened with a token query, so it never matches the above.
        called |= set(re.findall(r'EventSource\(`(/api/[a-z]+)', SCRIPT))
        return called

    def test_script_routes_all_exist_in_the_server(self):
        routes = self.page_routes()
        self.assertTrue(routes)
        for route in routes:
            self.assertIn(f'"{route}"', SERVER, f"{route} is called by the page but not routed")

    def test_server_routes_are_all_used(self):
        routed = set(re.findall(r'path == "(/api/[a-z]+)"', SERVER))
        # /api/state is the documented read-only snapshot endpoint for tools and
        # for a reconnect that wants data before the stream is up; it is not a
        # dead route even though the page itself never calls it.
        routed.discard("/api/state")
        self.assertEqual(sorted(routed - self.page_routes()), [], "dead API routes")

    def test_handler_offers_the_pieces_the_session_needs(self):
        for attribute in ("join", "leave", "is_controller", "broadcast", "run"):
            self.assertTrue(hasattr(webserver.Session, attribute), attribute)
        self.assertTrue(hasattr(webserver.Console, "hello"))
        self.assertTrue(hasattr(webserver.Console, "drain"))
        # stopping is per-instance state, not a class attribute.
        session = webserver.Session(webserver.Console(), exit_when_idle=False)
        self.assertFalse(session.stopping.is_set())
        self.assertIsNone(session.controller)

    def test_loopback_is_the_default_bind(self):
        parser_default = re.search(r'"--host",\s*default="([^"]+)"', SERVER)
        self.assertEqual(parser_default.group(1), "127.0.0.1",
                         "motor control must not be exposed to the network by default")


if __name__ == "__main__":
    unittest.main()
