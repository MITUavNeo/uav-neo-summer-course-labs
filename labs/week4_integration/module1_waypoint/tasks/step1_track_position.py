"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 1 — Step 1: Track Position
The sim has no GPS, so estimate position by integrating velocity over time (dead
reckoning) on every axis at once: right (x), up (y), forward (z). Step 2 uses this
estimate to fly to a target point.
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
PROBE_PITCH = 0.10     # gentle forward nudge
PROBE_ROLL = 0.10      # gentle rightward nudge
REPORT_TIME = 4.0

# -- Module-level state -----------------------------------------------------
_x = 0.0   # meters right of start
_z = 0.0   # meters forward of start
_timer = 0.0
_done = False

def reset():
    global _x, _z, _timer, _done
    _x = 0.0
    _z = 0.0
    _timer = 0.0
    _done = False


def update(drone):
    global _x, _z, _timer, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # Dead reckoning: read drone.physics.get_linear_velocity() (vx=right, vy=up,
    # vz=forward) and integrate the horizontal components into (_x, _z) each frame. Nudge
    # diagonally with PROBE_PITCH/PROBE_ROLL so the drone moves. After REPORT_TIME, stop,
    # print the position (use neo_lab.height(drone) for the up axis), and set _done.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Track Position")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
