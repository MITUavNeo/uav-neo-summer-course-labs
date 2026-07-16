"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 5 — Distance Estimation — Main orchestrator

Runs every step in sequence against the simulator:
    drone sim module5_distance_estimation/main.py
Run a single step directly instead:
    drone sim tasks/<step_file>.py
"""

# -- Course setup: makes the shared `neo_lab` helper importable (don't edit). --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

from tasks import (
    step1_measure_width,
    step2_estimate_range,
)

neo_lab.run_module("Week 2 · Module 5 — Distance Estimation", [
    ("Step 1: Measure a Gate Tag's Apparent Size", step1_measure_width),
    ("Step 2: Estimate Range and Approach", step2_estimate_range),
])
