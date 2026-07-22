"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Altitude Setpoint Sequence  (SOLUTION)
Chase a sequence of target heights (a step response).
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
SETPOINTS = [0.5, 1.0, 0.3]   # meters above ground, in order
KP = 0.2
THROTTLE_LIMIT = 0.5
TOL = 0.2
HOLD_TIME = 2.0

# -- Module-level state -----------------------------------------------------
_index = 0
_hold = 0.0
_done = False

def reset():
    global _index, _hold, _done
    _index = 0
    _hold = 0.0
    _done = False


def update(drone):
    global _index, _hold, _done
    if _done:
        return True
    if _index >= len(SETPOINTS):
        drone.flight.stop()
        print("[Step 2] Visited all setpoints")
        _done = True
        return _done
    target = SETPOINTS[_index]
    height = neo_lab.height(drone)
    error = target - height
    throttle = uav_utils.clamp(KP * error, -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(0, 0, 0, throttle)
    if abs(error) < TOL:
        _hold += drone.get_delta_time()
    else:
        _hold = 0.0
    if _hold >= HOLD_TIME:
        print(f"[Step 2] Reached setpoint {target}m (height {height:.2f}m)")
        _index += 1
        _hold = 0.0
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Altitude Setpoint Sequence")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
