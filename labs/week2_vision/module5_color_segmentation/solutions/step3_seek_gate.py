"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Seek the Gate  (SOLUTION)
Yaw to center the largest cyan gate, then fly forward toward it.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

# -- Course setup: makes the shared `neo_lab` helper importable.
#    You don't need to read or change this block. --
import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
MIN_AREA   = 400
COL_CENTER = 320
MAX_YAW    = 0.3        # yaw authority for centering
APPROACH_PITCH = 0.2    # forward speed once centered
CENTER_TOL = 90         # px error to count as centered
SEARCH_YAW = 0.2        # spin slowly when no gate is seen
TARGET_WIDTH = 170      # gate this wide (px) => close enough

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done
    _done = False


def update(drone):
    global _done
    if _done:
        return True
    image = drone.camera.get_color_image()
    best = neo_lab.largest_cyan_gate(image, MIN_AREA)
    if best is None:
        drone.flight.send_pcmd(0, 0, SEARCH_YAW, 0)   # scan for a gate
        return False
    x, y, w, h = cv2.boundingRect(best)
    gate_col = x + w / 2.0
    err = (gate_col - COL_CENTER) / COL_CENTER        # -1 (left) .. +1 (right)
    yaw = uav_utils.clamp(err * MAX_YAW, -MAX_YAW, MAX_YAW)
    pitch = APPROACH_PITCH if abs(gate_col - COL_CENTER) < CENTER_TOL else 0.0
    drone.flight.send_pcmd(pitch, 0, yaw, 0)
    if w >= TARGET_WIDTH:
        drone.flight.stop()
        print(f"[Step 3] Reached the gate (width={w}px)")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Seek the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
