"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Center Over the Gate  (SOLUTION)
Visual-servo the drone to hover directly above the gate frame.
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
V_MIN = 200
MIN_AREA = 300
MAX_TILT = 0.18      # pitch/roll authority
CENTER_TOL = 60      # pixels considered 'centered'
HOLD_TIME = 2.0      # seconds to stay centered before done
ROW_CENTER = 240
COL_CENTER = 320

# -- Module-level state -----------------------------------------------------
_hold = 0.0
_done = False

def reset():
    global _hold, _done
    _hold = 0.0
    _done = False


def update(drone):
    global _hold, _done
    if _done:
        return True
    image = drone.camera.get_downward_image()
    best = neo_lab.largest_bright_contour(image, V_MIN, MIN_AREA)
    if best is None:
        drone.flight.stop()
        _hold = 0.0
        return False
    row, col = uav_utils.get_contour_center(best)
    err_col = col - COL_CENTER               # +ve => gate is to the right
    err_row = row - ROW_CENTER               # +ve => gate is behind (toward image bottom)
    roll = uav_utils.clamp(err_col / COL_CENTER * MAX_TILT, -MAX_TILT, MAX_TILT)
    pitch = uav_utils.clamp(-err_row / ROW_CENTER * MAX_TILT, -MAX_TILT, MAX_TILT)
    drone.flight.send_pcmd(pitch, roll, 0, 0)
    if abs(err_col) < CENTER_TOL and abs(err_row) < CENTER_TOL:
        _hold += drone.get_delta_time()
    else:
        _hold = 0.0
    if _hold >= HOLD_TIME:
        drone.flight.stop()
        print("[Step 3] Centered over the gate")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Center Over the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
