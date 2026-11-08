"""ASCII-only batch entry point for the collector in its original directory."""
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).resolve().parent / "\u91c7\u96c6\u5de5\u5177" / "collect.py"), run_name="__main__")
