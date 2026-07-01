"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Blur & Edge Detection
Averaging blur then a Sobel edge-magnitude image.
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
KERNEL_SIZE = 5
HOVER_TIME  = 3.0

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

    # GOAL: report the average edge strength in the downward image.
    #
    # Tools: drone.camera.get_downward_image(); cv2.cvtColor(.., COLOR_BGR2GRAY);
    #        cv2.blur(gray, (KERNEL_SIZE, KERNEL_SIZE)); cv2.Sobel(.., cv2.CV_64F, ..).
    #
    # A Sobel filter measures how fast brightness changes in one direction. Run it
    # once across (x) and once down (y), then combine into one edge magnitude with
    # sqrt(sx**2 + sy**2). Blur first so single-pixel noise does not dominate.
    # _timer += drone.get_delta_time(); print the mean magnitude and finish at HOVER_TIME.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Blur & Edge Detection")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
