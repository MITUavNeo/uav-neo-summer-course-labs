"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Shape flight: takeoff, trace a repeating shape, land.

Same code runs in the simulator and on the real drone:
    drone sim course/flights/shape_flight/main.py   # simulator (from your computer)
    python3 main.py                                  # real drone (on the Pi)

Choose the shape and mode with the constants at the top of step_shape.py.
"""

# -- Course setup: makes the shared `neo_lab` helper and this folder importable. --
import os as _os, sys as _sys
_here = _os.path.dirname(_os.path.realpath(__file__))
if _here not in _sys.path:
    _sys.path.insert(0, _here)
_d = _here
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

import step_shape

LAUNCH_HEIGHT_M = 1.0

neo_lab.run_module(
    "Shape Flight",
    [("Trace shape", step_shape)],
    launch_height=LAUNCH_HEIGHT_M,
    autostart=True,
)
