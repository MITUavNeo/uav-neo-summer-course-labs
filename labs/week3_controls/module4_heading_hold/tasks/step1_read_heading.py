"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 3 · Module 4 — Step 1: Read the Heading
The drone's attitude (pitch, roll, yaw) comes from its IMU, not the camera. Yaw is
the compass heading: which way the nose points. Read it here so Step 2 can hold it.
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
PROBE_YAW = 0.15      # turn slowly so the heading visibly changes
HOVER_TIME = 4.0

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done = False

def reset():
    global _timer, _done
    _timer = 0.0
    _done = False


def update(drone):
    global _timer, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # Read the drone's attitude from drone.physics.get_attitude() (pitch, roll, yaw in
    # degrees; yaw is the compass heading). Turn slowly with PROBE_YAW so the heading
    # visibly changes, and after HOVER_TIME stop, print the final yaw, and set _done.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Read the Heading")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
