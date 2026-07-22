"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 1 — Step 2: Go To a Waypoint  (SOLUTION)
Position is dead-reckoned from velocity, so it drifts; POS_TOL is generous to match.
"""

import drone_core

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
POS_TOL = 0.5
SETTLE_SPEED = 0.25
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
    dt = drone.get_delta_time()
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    err_right = TARGET_RIGHT - _x
    err_fwd = TARGET_FWD - _z
    v_right = KP_POS * err_right
    v_forward = KP_POS * err_fwd
    v_up = neo_lab.altitude_hold_velocity(drone, TARGET_HEIGHT)
    neo_lab.send_velocity(drone, v_right, v_up, v_forward)
    speed = (vx ** 2 + vz ** 2) ** 0.5
    if abs(err_right) < POS_TOL and abs(err_fwd) < POS_TOL and speed < SETTLE_SPEED:
        _hold += dt
    else:
        _hold = 0.0
    if _hold >= HOLD_TIME:
        drone.flight.stop()
        print(f"[Step 2] Arrived: right={_x:.2f} forward={_z:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Go To a Waypoint")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
