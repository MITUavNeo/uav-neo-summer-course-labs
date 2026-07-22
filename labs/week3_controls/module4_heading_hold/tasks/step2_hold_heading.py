"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 3 · Module 4 — Step 2: Hold a Heading
Turn to a target compass heading and hold it with a proportional yaw controller.
The catch is angle wrap-around: the error between 350 deg and 10 deg is 20 deg, not
340 deg, so you must wrap the error into the range -180..180.
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
TARGET_HEADING = 90.0    # degrees to point the nose at
KP_YAW = 0.01            # yaw command per degree of error
MAX_YAW = 0.3
TOL = 5.0                # degrees counted as on-heading
HOLD_TIME = 2.0

# -- Module-level state -----------------------------------------------------
_hold = 0.0
_done = False

def heading_error(target, current):
    """Smallest signed angle (deg) from current heading to target, in -180..180."""
    ##################################
    #### START PUT CODE HERE #########
    error = 0.0
    ###### END PUT CODE HERE #########
    ##################################
    return error

def reset():
    global _hold, _done
    _hold = 0.0
    _done = False


def update(drone):
    global _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: rotate to TARGET_HEADING and hold it within TOL for HOLD_TIME.
    #
    # Tools: drone.physics.get_attitude() -> (pitch, roll, yaw) deg; heading_error()
    #        above; uav_utils.clamp(...); drone.flight.send_pcmd(0, 0, yaw, 0).
    #
    # Read yaw, compute the wrapped error with heading_error(), and command a yaw rate
    # proportional to it (gain KP_YAW, clamped to MAX_YAW). Watch the sign so it turns the
    # short way. Count HOLD_TIME of being within TOL before finishing.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Hold a Heading")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
