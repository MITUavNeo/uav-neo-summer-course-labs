"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Fly a Distance (PID on Position)  (SOLUTION)
Integrate forward velocity into position and PID to a target distance,
while a proportional term keeps altitude.
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
TARGET_DIST = 4.0    # meters forward
TARGET_HEIGHT = 1.0  # hold launch height
KP = 0.15
KI = 0.0
KD = 0.5    # strong velocity damping to avoid overshoot
PITCH_LIMIT = 0.25
ALT_KP = 0.12
THROTTLE_LIMIT = 0.5
MIN_TRAVEL = 5.0   # fly at least this long before checking 'arrived'
SETTLE_SPEED = 0.25  # must slow below this to count as arrived
HOLD_TIME = 1.5

# -- Module-level state -----------------------------------------------------
_pos = 0.0
_err_int = 0.0
_prev_err = 0.0
_t = 0.0
_hold = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Standard PID law: output = kp*err + ki*err_int + kd*err_dot."""
    return kp * err + ki * err_int + kd * err_dot

def reset():
    global _pos, _err_int, _prev_err, _t, _hold, _done
    _pos = 0.0
    _err_int = 0.0
    _prev_err = 0.0
    _t = 0.0
    _hold = 0.0
    _done = False


def update(drone):
    global _pos, _err_int, _prev_err, _t, _hold, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    _t += dt
    velocity = drone.physics.get_linear_velocity()
    _pos += velocity[2] * dt                     # z axis points forward
    error = TARGET_DIST - _pos
    _err_int += error * dt
    err_dot = -velocity[2]        # d(error)/dt = -forward velocity (clean derivative term)
    _prev_err = error
    pitch = uav_utils.clamp(pid_control(error, _err_int, err_dot, KP, KI, KD),
                            -PITCH_LIMIT, PITCH_LIMIT)
    throttle = uav_utils.clamp(ALT_KP * (TARGET_HEIGHT - neo_lab.height(drone)),
                               -THROTTLE_LIMIT, THROTTLE_LIMIT)
    drone.flight.send_pcmd(pitch, 0, 0, throttle)
    # The PID drives toward TARGET_DIST, but dead-reckoned distance is imprecise and
    # pitch has a deadband, so we call it "arrived" once the drone has flown for a
    # moment (MIN_TRAVEL) and then come to rest.
    if _t > MIN_TRAVEL and abs(velocity[2]) < SETTLE_SPEED:
        _hold += dt
    else:
        _hold = 0.0
    if _hold >= HOLD_TIME:
        drone.flight.stop()
        print(f"[Step 2] Arrived (estimated {_pos:.2f}m of {TARGET_DIST}m)")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Fly a Distance (PID on Position)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
