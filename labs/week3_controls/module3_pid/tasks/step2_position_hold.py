"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 2: Fly a Distance (PID on Position)
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
TARGET_HEIGHT = 3.0  # hold launch height
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
    ##################################
    #### START PUT CODE HERE #########
    output = 0.0  # YOUR CODE HERE (combine the three gain terms)
    ###### END PUT CODE HERE #########
    ##################################
    return output

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
    ##################################
    #### START PUT CODE HERE #########

    # We have no direct (x, z) position, so integrate velocity to estimate distance.
    # Pitch accelerates the drone, so use the velocity itself as the derivative term
    # (KD) to brake before the target instead of overshooting.
    # 1. dt = drone.get_delta_time()
    # 2. velocity = drone.physics.get_linear_velocity()   # (x=right, y=up, z=forward)
    # 3. _pos += velocity[2] * dt
    # 4. error = TARGET_DIST - _pos; update _err_int; err_dot = -velocity[2]
    # 5. pitch = uav_utils.clamp(pid_control(error, _err_int, err_dot, KP, KI, KD),
    #                            -PITCH_LIMIT, PITCH_LIMIT)
    # 6. Hold height too: throttle = clamp(ALT_KP*(TARGET_HEIGHT - neo_lab.height(drone)),
    #                                      -THROTTLE_LIMIT, THROTTLE_LIMIT)
    # 7. send_pcmd(pitch, 0, 0, throttle); finish once _pos reaches TARGET_DIST and the
    #    drone has slowed below SETTLE_SPEED for HOLD_TIME (don't check before MIN_TRAVEL).

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

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
