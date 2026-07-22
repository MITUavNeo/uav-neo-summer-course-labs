"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Follow the Line
Steer the drone to keep the colored line centered while flying forward.
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
MIN_PIXELS    = 200
FORWARD_PITCH = 0.10     # slow forward speed so roll can keep up on curves
MAX_ROLL      = 0.20     # lateral centering on the near part of the line
MAX_YAW       = 0.40     # turn to align heading with the line ahead (primary curve steering)
IMAGE_CENTER  = 320      # 640-wide image -> center column
LOOKAHEAD_FRAC = 0.4     # split the line into a far (yaw) part and a near (roll) part
LOST_TIME     = 3.0      # land after the line has been out of view this long (its end)
MAX_FOLLOW    = 60.0     # safety cap on total follow time

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_lost  = 0.0
_done  = False

def reset():
    global _timer, _lost, _done
    _timer = 0.0
    _lost  = 0.0
    _done  = False


def update(drone):
    global _timer, _lost, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: follow the line by turning to face along it. Fly forward at FORWARD_PITCH;
    # split the line pixels into a FAR part (top of the view, where you are heading) and a
    # NEAR part (bottom, under you). Yaw toward the far part to steer through curves; roll
    # toward the near part for fine lateral centering.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.saturated_mask(image, S_MIN);
    #        np.argwhere(mask) -> line pixel (row, col); uav_utils.clamp(...);
    #        drone.flight.send_pcmd(pitch, roll, yaw, throttle).
    #
    # Split rows at rows.min() + (rows.max()-rows.min())*LOOKAHEAD_FRAC. The far part's mean
    # column vs IMAGE_CENTER gives a yaw error (clamp to MAX_YAW); the near part's gives a
    # roll error (clamp to MAX_ROLL). With too few line pixels, hover and count the time as
    # lost. Finish when the line has been lost for LOST_TIME (its end) or _timer hits MAX_FOLLOW.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Follow the Line")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
