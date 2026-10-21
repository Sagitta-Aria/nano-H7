"""Push the Jetson host package to the Jetson and verify what landed.

The Jetson gets the headless control layer and the dependency-free Web console;
the Tk GUI modules stay on the development PC. Files are compared by SHA-256
and only differing ones are copied, so repeated runs are cheap and the box
cannot silently drift behind the working tree.

    python tools/deploy_host.py --check      # report differences only
    python tools/deploy_host.py              # sync, then run the box's tests
    python tools/deploy_host.py --no-tests   # sync only
"""
import argparse
import hashlib
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
REMOTE = "nx"
REMOTE_DIR = "h743"

HOST_FILES = ("__init__.py", "protocol.py", "transport.py", "simulator.py", "servo_config.py", "mission.py", "mission_vision.py", "navigation.py", "navigation_runner.py", "navigation_grid.py", "navigation_sim.py", "auxiliary.py",
              "turn.py", "turn_sim.py", "straight.py", "straight_sim.py", "optical_flow.py",
              "agent.py", "control.py", "verify_link.py", "webapp.py", "imu.py", "imu_protocol.py", "autolink.py", "camera.py",
              "webserver.py", "camera_vision.py", "camera_alignment.py", "camera_monitor.py", "vision_alignment.py", "ring_sequence.py", "ring_perception.py", "static/index.html", "static/styles.css",
              "static/app.js", "static/imu.js", "static/camera.js", "static/camera_alignment.js")
TEST_FILES = ("__init__.py", "test_agent.py", "test_control.py", "test_webapp.py", "test_servo_connection.py", "test_host_restart.py", "test_ring_task.py",
              "test_imu.py", "test_autolink.py", "test_camera.py", "test_camera_alignment.py", "test_camera_servo.py", "test_visual_pid.py", "test_ring_sequence.py", "test_webpage.py", "test_stations.py", "test_mission.py", "chassis_fixture.py", "imu_fixture_server.py",
              "pty_link_check.py", "lease_stress.py", "test_flow_host.py", "test_path_upload.py", "test_control_fixes_host.py", "test_heading_tuning_host.py", "test_straight_host.py", "test_center_navigation.py")
HOST_FILES += ("vision_tuning.py", "static/vision_tuning.js", 'camera_qr.py', 'task_code.py', 'task_camera.py', 'qr_search.py')
HOST_FILES += ('pick_place.py', 'static/pick_place.js')
HOST_FILES += ('material_flow.py', 'static/material_flow.js')
HOST_FILES += ('b_pickup.py', 'static/b_pickup.js')
TEST_FILES += ('test_b_pickup.py',)
HOST_FILES += ('competition.py', 'control_settings.py', 'static/competition.html', 'static/competition.js', 'static/competition.css')
TEST_FILES += ('test_competition.py',)
TEST_FILES += ('test_control_latency.py',)
TEST_FILES += ('test_auxiliary_restore.py',)
TEST_FILES += ('test_camera_qr.py', 'test_task_code.py', 'test_vision_tuning.py', 'test_vision_tuning_http.py')
TEST_FILES += ('test_ring_performance.py',)
TEST_FILES += ('test_camera_monitor_modes.py',)
HOST_FILES += ('ring_detector.py', 'stack_ring.py')
HOST_FILES += ('imu_pause.py',)
TEST_FILES += ('test_imu_pause_host.py',)
TEST_FILES += ('test_pick_place.py',)
TEST_FILES += ('test_material_flow.py', 'test_material_settings.py', 'test_stack_ring.py')
VISION_FILES = ('二维码识别/qr_task.py', '色块识别/color_block_center.py', '黑色圆环识别与数字识别/black_ring.py',
                '黑色圆环识别与数字识别/ring_digit.py', '黑色圆环识别与数字识别/digit_core.py')
VISION_FILES += tuple('色块识别/'+name+suffix for name in
                      ('red','green','blue','yellow','black','light_blue')
                      for suffix in ('_block.py','_thresholds.json'))
VISION_FILES += ('黑色圆环识别与数字识别/ring_thresholds.py',)
MONITOR_FILES = ('nx_vision_live.py', 'nx_vision_ui.py', 'nx_vision_roi.py', 'nx_vision_color.py', 'nx_vision_fast.py', 'nx_vision_trt.py')
FILE_GROUPS = (("host", HOST_FILES), ("tests", TEST_FILES), ("工创赛视觉识别", VISION_FILES), ('tools', MONITOR_FILES))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def remote_digests():
    """One ssh round trip: sha256 of every remote file, or MISSING."""
    listing = " ".join(group + "/" + name for group, names in FILE_GROUPS for name in names)
    script = 'cd ~/{0} || exit 1; for f in {1}; do ' \
             'if [ -f "$f" ]; then echo "$(sha256sum "$f" | cut -d" " -f1) $f"; ' \
             'else echo "MISSING $f"; fi; done'.format(REMOTE_DIR, listing)
    # SSH returns UTF-8 paths, including the reused Chinese detector folders.
    result = subprocess.run(["ssh", REMOTE, script], capture_output=True,
                            text=True, encoding="utf-8")
    if result.returncode != 0:
        raise SystemExit("ssh failed: " + (result.stderr or result.stdout).strip())
    found = {}
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] == "MISSING":
            found[parts[1]] = None
        elif len(parts) == 2:
            found[parts[1]] = parts[0]
    return found


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="report, copy nothing")
    parser.add_argument("--no-tests", action="store_true", help="skip the remote test run")
    args = parser.parse_args(argv)

    remote = remote_digests()
    planned = []
    for group, names in FILE_GROUPS:
        for name in names:
            local = ROOT / group / name
            if not local.is_file():
                print("missing locally, skipped: {}/{}".format(group, name))
                continue
            key = "{}/{}".format(group, name)
            want = digest(local)
            have = remote.get(key)
            if want == have:
                print("same     {:<22} {}".format(key, want[:16]))
            else:
                state = "absent  " if have is None else "outdated"
                print("{} {:<22} local {} remote {}".format(
                    state, key, want[:16], (have or "-")[:16]))
                planned.append((group, name))

    if not planned:
        print("\nremote copy is up to date ({} files checked)".format(
            sum(len(names) for _, names in FILE_GROUPS)))
    elif args.check:
        print("\n{} file(s) would be copied; rerun without --check".format(len(planned)))
        return 1
    else:
        for group, name in planned:
            parent = pathlib.PurePosixPath(name).parent
            remote_parent = "~/{}/{}/{}".format(REMOTE_DIR, group, parent)
            subprocess.run(["ssh", REMOTE, "mkdir -p {}".format(remote_parent)], check=True)
            subprocess.run(["scp", "-q", str(ROOT / group / name),
                            "{}:{}".format(REMOTE, remote_parent)], check=True)
        print("\ncopied {} file(s)".format(len(planned)))
        # Trust nothing: prove the bytes that landed match the working tree.
        after = remote_digests()
        bad = [key for key in ("{}/{}".format(g, n) for g, n in planned)
               if after.get(key) != digest(ROOT / key)]
        if bad:
            print("VERIFY FAILED for: " + ", ".join(bad))
            return 2
        print("verified: every copied file matches its local SHA-256")

    if not args.no_tests and not args.check:
        print("\n--- remote agent tests (python3 on the box) ---")
        result = subprocess.run(
            ["ssh", REMOTE, "cd ~/{} && python3 -m unittest tests.test_agent tests.test_control tests.test_webapp tests.test_servo_connection tests.test_host_restart tests.test_imu tests.test_autolink tests.test_camera tests.test_camera_qr tests.test_task_code tests.test_pick_place tests.test_camera_alignment tests.test_camera_servo tests.test_visual_pid tests.test_ring_sequence tests.test_ring_task tests.test_webpage tests.test_flow_host tests.test_path_upload tests.test_control_fixes_host tests.test_heading_tuning_host tests.test_straight_host tests.test_mission tests.test_stations tests.test_center_navigation -q"
             .format(REMOTE_DIR)], text=True)
        if result.returncode != 0:
            return result.returncode
        # Then the real pyserial path over a pseudo-terminal: the closest thing
        # to the STM32 that exists before the board is wired up.
        print("\n--- real serial path over a pty (pyserial, no hardware) ---")
        pty_check = subprocess.run(
            ["ssh", REMOTE, "cd ~/{} && python3 tests/pty_link_check.py --seconds 4"
             .format(REMOTE_DIR)], text=True)
        return pty_check.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
