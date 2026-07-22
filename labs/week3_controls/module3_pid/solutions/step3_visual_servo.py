"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Visual Servoing (Vision + PID)  (SOLUTION)
Capstone: use a PID loop on the camera pixel error to keep a gate
centered by yawing. Combines Week 2 vision with Week 3 control.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

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
COL_CENTER = 320
KP = 0.35
KI = 0.0
KD = 0.2
MAX_YAW = 0.25
SEARCH_YAW = 0.15    # slow yaw while searching
SEARCH_PITCH = 0.1   # creep forward while searching; ArUco tags decode only up close
CENTER_TOL = 0.15    # normalized error considered centered
HOLD_TIME = 1.0
SEARCH_TIMEOUT = 15.0  # land instead of scanning forever if no gate is ever seen

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_hold = 0.0
_search_t = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Standard PID law: output = kp*err + ki*err_int + kd*err_dot."""
    return kp * err + ki * err_int + kd * err_dot

def reset():
    global _err_int, _prev_err, _hold, _search_t, _done
    _err_int = 0.0
    _prev_err = 0.0
    _hold = 0.0
    _search_t = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _hold, _search_t, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    image = drone.camera.get_color_image()
    gate = neo_lab.detect_gate(image)            # gate located from its ArUco corner tags
    if gate is None:
        _search_t += dt
        if _search_t >= SEARCH_TIMEOUT:          # give up rather than spin forever
            drone.flight.stop()
            print("[Step 3] No gate seen; landing. Start facing a gate, up close.")
            _done = True
            return True
        drone.flight.send_pcmd(SEARCH_PITCH, 0, SEARCH_YAW, 0)   # creep forward + scan; tags decode only up close
        _err_int = 0.0                           # reset integral when target is lost
        _hold = 0.0
        return False
    _search_t = 0.0
    error = (gate.cx - COL_CENTER) / COL_CENTER  # normalized -1..+1
    _err_int = uav_utils.clamp(_err_int + error * dt, -1.0, 1.0)
    err_dot = (error - _prev_err) / dt if dt > 0 else 0.0
    _prev_err = error
    yaw = uav_utils.clamp(pid_control(error, _err_int, err_dot, KP, KI, KD), -MAX_YAW, MAX_YAW)
    drone.flight.send_pcmd(0, 0, yaw, 0)
    if abs(error) < CENTER_TOL:
        _hold += dt
    elif abs(error) > 2.0 * CENTER_TOL:
        _hold = 0.0          # only reset on a big miss; tolerate small flicker
    if _hold >= HOLD_TIME:
        drone.flight.stop()
        print("[Step 3] Locked onto the gate")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Visual Servoing (Vision + PID)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
