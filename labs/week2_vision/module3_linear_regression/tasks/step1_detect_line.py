"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 1: Detect the Bright Edge Pixels
Find the glowing gate-edge pixels in the downward camera.
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
V_MIN      = 200
HOVER_TIME = 3.0

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
    drone.flight.stop()   # hover in place
    ##################################
    #### START PUT CODE HERE #########
    _timer += drone.get_delta_time()
    image = drone.camera.get_downward_image()
    edge_mask = neo_lab.bright_mask(image, V_MIN) > 0
    pixel_count = np.count_nonzero(edge_mask)
    if _timer >= HOVER_TIME:
        print(f"[Step 1] Found {pixel_count} bright edge pixels")
        _done = True

    # Gate edges glow bright, so threshold by brightness (HSV Value): neo_lab.bright_mask(
    # image, V_MIN) gives a mask of the bright pixels. Count them, and after HOVER_TIME
    # print the count and set _done. See the README (Key terms).

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Detect the Bright Edge Pixels")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
