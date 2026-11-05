"""Dry-run entry point without ROS: python calibrate.py. See sim2real/calibrate.py for the commands."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim2real.calibrate import main

if __name__ == "__main__":
    main()
