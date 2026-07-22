"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 2 — Step 1: Fly a Square  (SOLUTION)
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
SIDE = 3.0
WAYPOINTS = [(0.0, SIDE), (SIDE, SIDE), (SIDE, 0.0), (0.0, 0.0)]
TARGET_HEIGHT = 1.0
KP_POS = 0.6           # target speed (m/s) per meter of position error
WP_TOL = 0.6

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
    dt = drone.get_delta_time()
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    if _wp >= len(WAYPOINTS):
        drone.flight.stop()
        print("[Step 1] Square complete")
        _done = True
        return True
    target_right, target_fwd = WAYPOINTS[_wp]
    v_right = KP_POS * (target_right - _x)
    v_forward = KP_POS * (target_fwd - _z)
    v_up = neo_lab.altitude_hold_velocity(drone, TARGET_HEIGHT)
    neo_lab.send_velocity(drone, v_right, v_up, v_forward)
    if abs(target_right - _x) < WP_TOL and abs(target_fwd - _z) < WP_TOL:
        print(f"[Step 1] reached corner {_wp}: ({target_right:.1f}, {target_fwd:.1f})")
        _wp += 1
    return False


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Fly a Square")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
