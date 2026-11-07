"""PICO real-arm companion entry. Original launch_rm65_box_real.py is unchanged."""
from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parent.parent / "RM65_PICO" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
from launch_vdo_pico_real import main

if __name__ == "__main__":
    raise SystemExit(main())
