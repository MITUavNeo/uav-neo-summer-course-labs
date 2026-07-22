"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 1 — Step 2: Go To a Waypoint
Fly to a target point given as (right, up, forward) meters from the start. This is
your first controller that drives three axes at once, commanding a body velocity on
each: right, forward, and up.
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
TARGET_RIGHT = 2.0
TARGET_FWD = 4.0
TARGET_HEIGHT = 1.0
KP_POS = 0.6           # target speed (m/s) per meter of position error
POS_TOL = 0.5          # meters from target counted as arrived
SETTLE_SPEED = 0.25    # must slow below this to finish
HOLD_TIME = 1.5

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_hold = 0.0
_done = False

def reset():
    global _x, _z, _hold, _done
    _x = 0.0
    _z = 0.0
    _hold = 0.0
    _done = False


def update(drone):
    global _x, _z, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: fly to (TARGET_RIGHT, TARGET_HEIGHT, TARGET_FWD) and hold there.
    #
    # Tools: drone.physics.get_linear_velocity() -> (vx, vy, vz); drone.get_delta_time();
    #        neo_lab.altitude_hold_velocity(drone, TARGET_HEIGHT);
    #        neo_lab.send_velocity(drone, v_right, v_up, v_forward).
    #
    # Track right/forward position by integrating vx, vz like Step 1. Turn each position
    # error into a target speed (gain KP_POS); send_velocity turns it into motion and applies
    # the real-drone speed cap for you, so you don't clamp it yourself. Hold height with
    # neo_lab.altitude_hold_velocity. Command all three with send_velocity. Finish when both
    # horizontal errors are under POS_TOL and speed is under SETTLE_SPEED for HOLD_TIME.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Go To a Waypoint")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
