"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 1: Grayscale Thresholding
Grayscale + binary threshold of the live downward camera feed.
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
THRESHOLD_VALUE = 127     # grayscale cutoff (0-255)
HOVER_TIME      = 3.0     # seconds to observe

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

    downward_image = drone.camera.get_downward_image()
    downward_gray = cv2.cvtColor(downward_image, cv2.COLOR_BGR2GRAY)

    _, binary_mask = cv2.threshold(downward_gray, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)
    
    white_pixel_count = np.sum(binary_mask == 255)
    total_pixel_count = binary_mask.size
    fraction_white = white_pixel_count / total_pixel_count
    
    print(f"Fraction of white pixels: {fraction_white:.4f}")
   
    _timer += drone.get_delta_time()
    
    if _timer >= HOVER_TIME:
        _done = True  

    # Grab the downward image (drone.camera.get_downward_image()), convert it to
    # grayscale, and threshold at THRESHOLD_VALUE to make a binary mask. Report the
    # fraction of white pixels. Advance _timer and finish (_done) once it reaches
    # HOVER_TIME. See the README (Key terms) for thresholding.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Grayscale Thresholding")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
