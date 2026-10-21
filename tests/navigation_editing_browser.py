"""Use a temporary-config SIM server; never access real devices."""
import os
from pathlib import Path
from tests import field_camera_browser

if __name__ == '__main__':
    os.environ['CAMERA_BROWSER_TEST'] = str(Path(__file__).with_suffix('.cjs'))
    raise SystemExit(field_camera_browser.main())
