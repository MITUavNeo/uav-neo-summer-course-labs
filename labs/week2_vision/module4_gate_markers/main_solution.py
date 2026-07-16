"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 4 — Gate Markers (Seek a Gate) (SOLUTION) — Main orchestrator

Runs every step in sequence against the simulator:
    drone sim module4_gate_markers/main_solution.py
Run a single step directly instead:
    drone sim solutions/<step_file>.py
"""

# -- Course setup: makes the shared `neo_lab` helper importable (don't edit). --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

from solutions import (
    step1_detect_markers,
    step2_locate_gate,
    step3_seek_gate,
)

neo_lab.run_module("Week 2 · Module 4 — Gate Markers (Seek a Gate) (SOLUTION)", [
    ("Step 1: Detect Gate Markers", step1_detect_markers),
    ("Step 2: Locate the Gate", step2_locate_gate),
    ("Step 3: Seek the Gate", step3_seek_gate),
])
