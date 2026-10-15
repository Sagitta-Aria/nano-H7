"""Exercise real Tk widgets against demo transport, without opening a COM port."""
import io
import sys
import time
import tempfile
from pathlib import Path
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch
from host.gui import Console
from host.navigation import MissionConfig
from host.auxiliary import AuxiliaryConfig

# This console is a GBK code page on some machines; µ and ° must not crash it.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
else:  # pragma: no cover - very old Python
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")


def wait(root, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        root.update()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("GUI state did not arrive before timeout")


def main():
    root = tk.Tk()
    root.withdraw()
    app = Console(root)
    app.toggle_connection()
    wait(root, lambda: app.connected and app.values["cpu"].get() == "400 MHz")
    app.led_buttons[0].invoke()
    wait(root, lambda: app.led_canvas.itemcget(app.led_shapes[1], "fill") == "#20ad78")
    app.lift.direction.set("正步数向下")
    app.lift.configure_direction()
    wait(root, lambda: app.lift.fields.get("down_sign") == "1")
    app.arm()
    wait(root, lambda: app.values["armed"].get() == "已使能")
    app.move_fields["steps"].set("10")
    app.move_axis()
    wait(root, lambda: app.connection.serial.board.axes[4]["pos"] == 10)
    wait(root, lambda: app.lift.fields.get("pos") == "10" and app.lift.fields.get("active") == "0")
    app.lift.buttons["返回最高点"].invoke()
    wait(root, lambda: app.lift.fields.get("pos") == "0" and app.lift.fields.get("active") == "0")
    app.lift.steps.set("20")
    app.lift.buttons["向下"].invoke()
    wait(root, lambda: app.lift.fields.get("pos") == "20" and app.lift.fields.get("active") == "0")
    with patch("host.lift_panel.messagebox.showerror") as lift_error:
        app.lift.steps.set("21")
        app.lift.move("UP")
        assert lift_error.called
        assert app.connection.serial.board.axes[4]["pos"] == 20
    app.lift.steps.set("20")
    app.lift.buttons["向上"].invoke()
    wait(root, lambda: app.lift.fields.get("pos") == "0" and app.lift.fields.get("active") == "0")
    app.send("STOP")
    wait(root, lambda: app.values["armed"].get() == "已关闭")
    # Dedicated rotation setup/presets use a temporary settings file.
    with tempfile.TemporaryDirectory() as directory:
        store = AuxiliaryConfig(Path(directory)/"auxiliary.json")
        app.lift.settings = app.rotation.settings = store
        rotation = app.rotation
        rotation.variables["min"].set("-100")
        rotation.variables["max"].set("100")
        rotation.variables["step"].set("10")
        rotation.presets[0].set("-20"); rotation.presets[1].set("40")
        rotation.zero_button.invoke()
        wait(root, lambda: rotation.telemetry.get("referenced") == "1")
        rotation.apply_range()
        wait(root, lambda: rotation.telemetry.get("configured") == "1")
        assert AuxiliaryConfig(store.path).snapshot()["6"]["presets"] == [-20, 40]
        app.arm()
        wait(root, lambda: app.values["armed"].get() == "已使能")
        rotation.move("DOWN")
        wait(root, lambda: rotation.telemetry.get("pos") == "10" and rotation.telemetry.get("active") == "0")
        for index, target in ((0, "-20"), (1, "40")):
            rotation.goto(index)
            wait(root, lambda: rotation.telemetry.get("pos") == target and rotation.telemetry.get("active") == "0")
        rotation.record(0)
        assert rotation.presets[0].get() == "40"
        with patch("host.aux_panel.messagebox.showerror") as complaint:
            rotation.variables["step"].set("61")
            rotation.move("DOWN")
            assert complaint.called
        app.send("AUX GOTO 6 0 300 1000")
        wait(root, lambda: rotation.telemetry.get("pos") == "0" and rotation.telemetry.get("active") == "0")
        app.send("STOP")
        wait(root, lambda: app.values["armed"].get() == "已关闭")
    # Body-relative movement and actual keyboard handlers, using demo I/O.
    panel = app.chassis
    app.notebook.select(panel.parent)
    root.update()
    app.arm()
    wait(root, lambda: app.values["armed"].get() == "已使能")
    panel.steps.set("10")
    panel.direction.set("左移")
    panel.move()
    wait(root, lambda: app.connection.serial.board.chassis_state == "done")
    assert [a["pos"] for a in app.connection.serial.board.axes[:4]] == [-10, -10, 10, 10]
    panel.keyboard.set(True)
    panel.key_down(SimpleNamespace(keysym="Up"))
    wait(root, lambda: app.connection.serial.board.chassis_state == "jogging")
    for _ in range(10):
        panel.key_down(SimpleNamespace(keysym="Up"))  # auto-repeat must not enqueue
    panel.key_up(SimpleNamespace(keysym="Up"))
    wait(root, lambda: app.connection.serial.board.chassis_state == "stopped")
    assert app.connection.current_jog() is None
    panel.begin("CCW", "mouse")
    wait(root, lambda: app.connection.serial.board.chassis_state == "jogging")
    panel.mouse_up(None)
    wait(root, lambda: app.connection.serial.board.chassis_state == "stopped")
    # Supply OS focus loss deterministically without stealing desktop focus.
    panel.begin("FWD", "mouse")
    wait(root, lambda: app.connection.serial.board.chassis_state == "jogging")
    with patch.object(root, "focus_displayof", return_value=None):
        panel.focus_out(None)
        root.update_idletasks()
    wait(root, lambda: app.connection.serial.board.chassis_state == "stopped")
    entry = tk.Entry(panel.content)
    with patch.object(root, "focus_get", return_value=entry):
        panel.key_down(SimpleNamespace(keysym="Up"))
        assert panel.intent is None  # Editing a value cannot start the wheels.
    entry.destroy()
    panel.begin("RIGHT", "mouse")
    wait(root, lambda: app.connection.serial.board.chassis_state == "jogging")
    app.notebook.select(0)
    wait(root, lambda: app.connection.serial.board.chassis_state == "stopped")
    # Servo page talks to the same simulator; the demo board advertises 270 deg.
    # A hidden window makes tkinter message boxes modal and invisible, so patch
    # the panels that can raise one instead of hanging the test on a dialog.
    with patch("host.servo_panel.messagebox.showerror") as complaint:
        app.servo.channel.set("2")
        app.servo.select_channel()
        wait(root, lambda: app.servo.travel == 270.0)
        app.servo.send("135")
        wait(root, lambda: app.servo.frames.get(2, {}).get("angle") == "135.0")
        assert app.connection.serial.board.servo_steps[1] == 180
        app.servo.send("270.5")          # out of travel: refused before any command
        assert complaint.called, "the panel must refuse an angle past its travel"
        assert app.connection.serial.board.servo_steps[1] == 180
        app.send("SERVO 2 180.5")        # the board refuses it as well
        wait(root, lambda: "ANGLE_RANGE" in app.status.get())
        assert app.connection.serial.board.servo_steps[1] == 180
        app.servo.pulse.set("2500")      # calibration path converts back to an angle
        app.servo.send_pulse()
        wait(root, lambda: app.connection.serial.board.servo_steps[1] == 360)
        # Selecting S1/PH6 changes its limit without moving it.
        app.servo.channel.set("1")
        app.servo.select_channel()
        assert app.servo.travel == 180.0
        app.servo.send("180")
        wait(root, lambda: app.connection.serial.board.servo_steps[0] == 540)
        assert app.connection.serial.board.servo_steps[1] == 360
        complaint.reset_mock()
        app.servo.send("180.5")
        assert complaint.called
        app.servo.pulse.set("1500")
        app.servo.send_pulse()
        wait(root, lambda: app.connection.serial.board.servo_steps[0] == 270)
        app.send("SERVO STAT")
        # Device state can update before the worker delivers its reply to Tk.
        wait(root, lambda: app.servo.frames.get(2, {}).get("angle") == "270.0"
                          and app.servo.frames.get(1, {}).get("angle") == "90.0")
        app.servo.preset_buttons[120].invoke()
        wait(root, lambda: app.servo.frames.get(1, {}).get("angle") == "120.0")
        assert app.connection.serial.board.servo_steps[0] == 360
        app.servo.preset_buttons[90].invoke()
        wait(root, lambda: app.servo.frames.get(1, {}).get("angle") == "90.0")
        app.servo.channel.set("2")
        app.servo.select_channel()
        assert app.servo.travel == 270.0
        assert float(app.servo.angle.get()) == 270.0
    # Exercise the new Tk mission editor without altering user parameters.
    app.send("STOP")
    wait(root, lambda: app.values["armed"].get() == "已关闭")
    with tempfile.TemporaryDirectory() as directory:
        navigation = app.navigation
        navigation.config = MissionConfig(Path(directory) / "navigation.json")
        for name, point in {"O": (0, 0), "A": (0, 14), "B": (14, 0),
                            "C": (14, 28), "D": (28, 14)}.items():
            for variable, value in zip(navigation.fields[name], point):
                variable.set(str(value))
        navigation.fields["dwell_ms"].set("20")
        for variables in navigation.grid_fields.values():
            for variable, value in zip(variables, (0,14,28)): variable.set(str(value))
        navigation.origin()
        wait(root, lambda: navigation.telemetry.get("ready") == "1")
        app.arm()
        wait(root, lambda: app.values["armed"].get() == "已使能")
        navigation.start()
        wait(root, lambda: navigation.telemetry.get("state") == "done", timeout=30)
        assert navigation.telemetry["x"] == "0" and navigation.telemetry["y"] == "14"
        assert not navigation.uploading
        app.send("NAV STOP")
    app.toggle_connection()
    wait(root, lambda: app.connection is None)
    root.destroy()
    print("GUI smoke OK: LED, axes, lift down/up/top and limit, chassis finite move, key hold/repeat/release, mouse release, "
          "rotation limits/presets/record, focus loss, entry guard, tab change, servo angle and pulse calibration, Tk navigation mission, disconnect")


if __name__ == "__main__":
    main()
