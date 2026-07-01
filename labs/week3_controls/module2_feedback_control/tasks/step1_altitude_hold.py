"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 1: Proportional Altitude Hold
Hold a target height using proportional throttle control.
Heights are measured above the ground sampled at launch.
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
TARGET_HEIGHT = 5.0    # meters above ground
KP = 0.2              # throttle ~ 12 m/s per unit, so keep small
THROTTLE_LIMIT = 0.5
TOL = 0.4            # P-control leaves a small steady-state droop
HOLD_TIME = 3.0      # seconds on target before done

# -- Module-level state -----------------------------------------------------
_hold = 0.0
_done = False

def reset():
    global _hold, _done
    _hold = 0.0
    _done = False


def update(drone):
    global _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # Proportional control:  output = Kp * error.  Throttle sets vertical SPEED
    # (~12 m/s per unit) and stop() holds altitude, so use a small gain.
    # 1. height = neo_lab.height(drone)          # meters above the launch ground
    # 2. error = TARGET_HEIGHT - height
    # 3. throttle = uav_utils.clamp(KP * error, -THROTTLE_LIMIT, THROTTLE_LIMIT)
    # 4. drone.flight.send_pcmd(0, 0, 0, throttle)
    # 5. Accumulate _hold while abs(error) < TOL (else reset _hold = 0).
    # 6. When _hold >= HOLD_TIME: stop and set _done = True

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Proportional Altitude Hold")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
