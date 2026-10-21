"""Run X departure UI checks on the existing isolated, config-guarded SIM server."""
from pathlib import Path
from tests import stations_browser

if __name__ == '__main__':
    stations_browser.CHECK = Path(__file__).with_suffix('.cjs')
    raise SystemExit(stations_browser.main())
