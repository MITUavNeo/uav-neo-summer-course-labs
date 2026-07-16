"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Fit a Line (Least Squares)
Fit y = m*x + b to the bright edge pixels with linear regression.
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
MIN_PIXELS = 200
HOVER_TIME = 3.0

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done  = False

def fit_line(points):
    """Least-squares fit of y = m*x + b. points is the (row, col) array from
    np.argwhere, so column = x and row = y. See the README (Key terms) for the fit."""
    ##################################
    #### START PUT CODE HERE #########
    m, b = 0.0, 0.0
    ###### END PUT CODE HERE #########
    ##################################
    return m, b

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
    mask = neo_lab.bright_mask(image, V_MIN) > 0
    points = np.argwhere(mask)

    if len(points) < MIN_PIXELS:
        return False

    m, b = fit_line(points)

    if _timer >= HOVER_TIME:
        print(f"m={m:.3f}, b={b:.1f}")
        _done = True

    # Build the bright-edge mask like Step 1 and collect the (row, col) of every bright
    # pixel. If there are fewer than MIN_PIXELS, there is not enough edge to fit -> return
    # False. Otherwise call fit_line() and print m, b. Advance _timer and finish at
    # HOVER_TIME.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Fit a Line (Least Squares)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
