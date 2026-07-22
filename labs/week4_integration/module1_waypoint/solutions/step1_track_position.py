"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 1 — Step 1: Track Position  (SOLUTION)
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
PROBE_PITCH = 0.10
PROBE_ROLL = 0.10
REPORT_TIME = 4.0

# -- Module-level state -----------------------------------------------------
_x = 0.0
_z = 0.0
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
    dt = drone.get_delta_time()
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    drone.flight.send_pcmd(PROBE_PITCH, PROBE_ROLL, 0, 0)
    _timer += dt
    if _timer >= REPORT_TIME:
        drone.flight.stop()
        print(f"[Step 1] position right={_x:.2f}  up={neo_lab.height(drone):.2f}  "
              f"forward={_z:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Track Position")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
