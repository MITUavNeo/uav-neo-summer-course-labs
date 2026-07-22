"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 3 · Module 4 — Step 2: Hold a Heading  (SOLUTION)
"""

import drone_core
import drone_utils as uav_utils

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
TARGET_HEADING = 90.0
KP_YAW = 0.01
MAX_YAW = 0.3
TOL = 5.0
HOLD_TIME = 2.0

# -- Module-level state -----------------------------------------------------
_hold = 0.0
_done = False

def heading_error(target, current):
    return ((target - current + 180.0) % 360.0) - 180.0

def reset():
    global _hold, _done
    _hold = 0.0
    _done = False


def update(drone):
    global _hold, _done
    if _done:
        return True
    pitch, roll, yaw = drone.physics.get_attitude()
    error = heading_error(TARGET_HEADING, yaw)
    cmd = uav_utils.clamp(KP_YAW * error, -MAX_YAW, MAX_YAW)
    drone.flight.send_pcmd(0, 0, cmd, 0)
    if abs(error) < TOL:
        _hold += drone.get_delta_time()
    else:
        _hold = 0.0
    if _hold >= HOLD_TIME:
        drone.flight.stop()
        print(f"[Step 2] Holding heading {TARGET_HEADING:.0f} deg (yaw={yaw:.1f})")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Hold a Heading")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
