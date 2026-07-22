"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 2 — Step 1: Fly a Square
A flight path is a list of waypoints visited in order. Here the list forms a square.
Reuse the waypoint controller from Module 1, one corner at a time, advancing to the
next corner when you reach the current one.
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
SIDE = 3.0
# Corners as (right, forward) meters from the start, traced as a square.
WAYPOINTS = [(0.0, SIDE), (SIDE, SIDE), (SIDE, 0.0), (0.0, 0.0)]
TARGET_HEIGHT = 1.0
KP_POS = 0.6           # target speed (m/s) per meter of position error
WP_TOL = 0.6           # meters from a corner counted as reached

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
_wp = 0
_done = False

def reset():
    global _x, _z, _wp, _done
    _x = 0.0
    _z = 0.0
    _wp = 0
    _done = False


def update(drone):
    global _x, _z, _wp, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: visit each corner in WAYPOINTS in order, then finish.
    #
    # Tools: drone.physics.get_linear_velocity(); drone.get_delta_time();
    #        neo_lab.altitude_hold_velocity(drone, TARGET_HEIGHT);
    #        neo_lab.send_velocity(drone, v_right, v_up, v_forward).
    #
    # Integrate vx, vz into (_x, _z) like Module 1. If _wp has passed the last waypoint,
    # stop and finish. Otherwise steer toward WAYPOINTS[_wp] the same way as Module 1:
    # each position error becomes a target speed (gain KP_POS), height held by
    # neo_lab.altitude_hold_velocity, all three sent with send_velocity. When within WP_TOL
    # of the corner on both axes, advance _wp += 1.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Fly a Square")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
