"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Follow the Line  (SOLUTION)
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
    dt = drone.get_delta_time()
    _timer += dt
    image = drone.camera.get_downward_image()
    mask = neo_lab.saturated_mask(image, S_MIN) > 0
    points = np.argwhere(mask)
    if len(points) < MIN_PIXELS:
        drone.flight.stop()                 # line out of view -> hover
        _lost += dt
    else:
        _lost = 0.0
        rows = points[:, 0]
        far_row = rows.min() + (rows.max() - rows.min()) * LOOKAHEAD_FRAC
        ahead = points[rows <= far_row]     # the line ahead of the drone (top of the view)
        near = points[rows > far_row]       # the line under the drone (bottom of the view)
        ahead_col = (ahead if len(ahead) > 0 else points)[:, 1].mean()
        near_col = (near if len(near) > 0 else points)[:, 1].mean()
        yaw = uav_utils.clamp((ahead_col - IMAGE_CENTER) / IMAGE_CENTER * MAX_YAW, -MAX_YAW, MAX_YAW)
        roll = uav_utils.clamp((near_col - IMAGE_CENTER) / IMAGE_CENTER * MAX_ROLL, -MAX_ROLL, MAX_ROLL)
        drone.flight.send_pcmd(FORWARD_PITCH, roll, yaw, 0)
    if _lost >= LOST_TIME or _timer >= MAX_FOLLOW:
        drone.flight.stop()
        print("[Step 3] Finished following the line")
        _done = True
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
