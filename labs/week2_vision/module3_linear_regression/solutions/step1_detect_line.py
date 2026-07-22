"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 1: Detect the Line Pixels  (SOLUTION)
Find the colored line pixels in the downward camera.
(The line is recolored each run, so detect it by saturation, not a fixed color.)
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
S_MIN         = 100
ADVANCE_PITCH = 0.15      # fly forward off the spawn pad to reach the line
ADVANCE_TIME  = 8.0       # seconds of forward flight before reporting

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False

def reset():
    global _timer, _done
    _timer = 0.0
    _done  = False


def update(drone):
    global _timer, _done
    if _done:
        return True
    _timer += drone.get_delta_time()
    if _timer < ADVANCE_TIME:
        drone.flight.send_pcmd(ADVANCE_PITCH, 0, 0, 0)   # fly off the spawn pad to the line
        return False
    drone.flight.stop()
    image = drone.camera.get_downward_image()
    line_mask = neo_lab.saturated_mask(image, S_MIN) > 0
    pixel_count = np.count_nonzero(line_mask)
    print(f"[Step 1] Found {pixel_count} colored line pixels")
    _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Detect the Line Pixels")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
