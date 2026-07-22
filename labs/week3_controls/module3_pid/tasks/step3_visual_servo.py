"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Visual Servoing (Vision + PID)
Use a PID loop on the camera pixel error to keep a gate centered by yawing.
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
COL_CENTER = 320
KP = 0.35
KI = 0.0
KD = 0.2
MAX_YAW = 0.25
SEARCH_YAW = 0.15    # slow yaw while searching
SEARCH_PITCH = 0.1   # creep forward while searching; ArUco tags decode only up close
CENTER_TOL = 0.15    # normalized error considered centered
HOLD_TIME = 1.0
SEARCH_TIMEOUT = 15.0  # land instead of scanning forever if no gate is ever seen

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_hold = 0.0
_search_t = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = 0.0
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _err_int, _prev_err, _hold, _search_t, _done
    _err_int = 0.0
    _prev_err = 0.0
    _hold = 0.0
    _search_t = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _hold, _search_t, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: yaw with a PID loop so a gate stays centered in the forward camera;
    # finish once it is centered (abs(error) < CENTER_TOL) for HOLD_TIME.
    #
    # Available helpers: drone.camera.get_color_image(); drone.get_delta_time();
    #   neo_lab.detect_gate(image) -> Gate or None (a Gate has .cx, the gate's center
    #   column); uav_utils.clamp; your pid_control() above.
    #
    # Turn the gate's horizontal offset from the image center (.cx vs COL_CENTER) into a
    # normalized error, PID it to a yaw command clamped to MAX_YAW, and when no gate is in view
    # creep forward (SEARCH_PITCH) while sweeping (SEARCH_YAW) so the tags get close enough to
    # decode. Count time without a gate and land once it passes SEARCH_TIMEOUT,
    # so a missing gate ends the lab instead of spinning forever. See the README (Key terms) and
    # Week 2 Module 5 for detect_gate.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Visual Servoing (Vision + PID)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
