"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Bounding Box
Find the largest cyan gate structure and its bounding box.
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
LOWER = neo_lab.CYAN_LOWER
UPPER = neo_lab.CYAN_UPPER
MIN_AREA = 400
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
    image = drone.camera.get_color_image()
    best = neo_lab.largest_cyan_gate(image, MIN_AREA)  
    if best is None:
        return False
    x, y, w, h = cv2.boundingRect(best)
    if _timer >= HOVER_TIME:
        print(f"Gate bound box: x={x}, y={y}, w={w}, h={h}")
        _done = True
    # The long glowing boundary lines are also cyan, so use neo_lab.largest_cyan_gate(
    # image, MIN_AREA), which keeps only square-ish (gate-shaped) contours; it returns None
    # when there is no gate -> return False. Otherwise find the contour's bounding box and
    # print it. Advance _timer and finish at HOVER_TIME.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Bounding Box")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
