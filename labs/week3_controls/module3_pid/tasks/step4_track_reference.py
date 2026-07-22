"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 3 · Module 3 — Step 4: Track a Moving Reference
Follow an altitude reference that MOVES in time, not a fixed setpoint. A PID that only
reacts to position error always lags a moving target; a velocity feedforward closes the gap.
Heights are measured above the ground sampled at launch.
"""

import math

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
BASE_HEIGHT = 0.6
AMPLITUDE = 0.3
PERIOD = 6.0
CYCLES = 2
DURATION = PERIOD * CYCLES
KP = 0.18
KI = 0.05
KD = 0.02
KFF = 1.0 / 12.0     # throttle is a vertical-velocity command (~12 m/s per unit)
INT_CLAMP = 3.0
THROTTLE_LIMIT = 0.5

# -- Module-level state -----------------------------------------------------
_t = 0.0
_err_int = 0.0
_prev_err = 0.0
_max_err = 0.0
_done = False


def reference(t):
    """The moving altitude target (meters) and its velocity (m/s) at time t.

    A raised-cosine so it starts at BASE_HEIGHT with zero velocity (no launch jerk).
    This is the drone's "trajectory" in one dimension; Week 4 extends it to a path.
    """
    w = 2.0 * math.pi / PERIOD
    r = BASE_HEIGHT + AMPLITUDE * (1.0 - math.cos(w * t))
    r_dot = AMPLITUDE * w * math.sin(w * t)
    return r, r_dot


def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Return the PID controller output from the three gain terms (see README, Key terms)."""
    ##################################
    #### START PUT CODE HERE #########
    output = 0.0
    ###### END PUT CODE HERE #########
    ##################################
    return output


def reset():
    global _t, _err_int, _prev_err, _max_err, _done
    _t = 0.0
    _err_int = 0.0
    _prev_err = 0.0
    _max_err = 0.0
    _done = False


def update(drone):
    global _t, _err_int, _prev_err, _max_err, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    _t += dt
    r, r_dot = reference(_t)
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: keep the drone ON the moving target r from reference(_t), not behind it.
    #
    # Run PID on the height error (r - neo_lab.height(drone)) exactly as in Step 1:
    # track the integral (clamp to +/-INT_CLAMP) and derivative yourself, then call
    # pid_control(...). THEN add a feedforward term: the target is already moving at
    # r_dot, so command that speed directly (KFF * r_dot) instead of waiting for error
    # to build. Sum feedback + feedforward, clamp to +/-THROTTLE_LIMIT, and send it as
    # throttle. Update _max_err with the largest abs(error) so far. See the README
    # ("Tracking a moving target") for why the feedforward term removes the lag.

    ###### END PUT CODE HERE #########
    ##################################
    if _t >= DURATION:
        drone.flight.stop()
        print(f"[Step 4] Tracked moving reference: max error {_max_err:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(BASE_HEIGHT)

    def start():
        _launcher.reset()
        reset()
        print("Step 4: Track a Moving Reference")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
