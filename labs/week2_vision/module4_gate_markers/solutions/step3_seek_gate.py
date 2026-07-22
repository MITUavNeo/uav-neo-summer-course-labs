"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Seek the Gate  (SOLUTION)
Yaw to center a gate by its ArUco corner tags, then fly forward toward it.
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
COL_CENTER     = 320
MAX_YAW        = 0.3        # yaw authority for centering
APPROACH_PITCH = 0.2        # forward speed once centered
CENTER_TOL     = 60         # px error to count as centered
SEARCH_YAW     = 0.15       # scan slowly when no gate is seen
SEARCH_PITCH   = 0.1        # creep forward while searching; ArUco tags only resolve up close
TARGET_TAG_PX  = 40         # corner tags this big (px) => close enough

# -- Module-level state -----------------------------------------------------
_done = False

def reset():
    global _done
    _done = False


def update(drone):
    global _done
    if _done:
        return True
    image = drone.camera.get_color_image()
    gate = neo_lab.detect_gate(image)
    if gate is None:
        drone.flight.send_pcmd(SEARCH_PITCH, 0, SEARCH_YAW, 0)   # creep + scan until tags resolve
        return False
    err = (gate.cx - COL_CENTER) / COL_CENTER         # -1 (left) .. +1 (right)
    yaw = uav_utils.clamp(err * MAX_YAW, -MAX_YAW, MAX_YAW)
    centered = abs(gate.cx - COL_CENTER) < CENTER_TOL
    pitch = APPROACH_PITCH if centered else 0.0       # only fly forward once centered
    drone.flight.send_pcmd(pitch, 0, yaw, 0)
    if centered and gate.tag_px >= TARGET_TAG_PX:     # close AND centered before finishing
        drone.flight.stop()
        print(f"[Step 3] Reached the gate (tag={gate.tag_px:.0f}px, tags={gate.count})")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Seek the Gate")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
