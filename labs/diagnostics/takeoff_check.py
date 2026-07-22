"""
Diagnostic: run only the launch and print the altitude as it climbs, so you can see the takeoff
profile and how far it overshoots the target height. Runs no lab steps; lands after settling.

    sim:   drone sim course/takeoff_check.py     (press ENTER in the sim window)
    drone: python3 takeoff_check.py              (safety pilot arms + OFFBOARD)

Watch the peak vs the target: a large gap means the launch overshoots. Target comes from
neo_lab.DEFAULT_LAUNCH_HEIGHT.
"""

import os
import sys

import drone_core

_d = os.path.dirname(os.path.realpath(__file__))
while os.path.basename(_d) != "labs" and os.path.dirname(_d) != _d:
    _d = os.path.dirname(_d)
if _d not in sys.path:
    sys.path.insert(0, _d)
import neo_lab

PRINT_EVERY_S = 0.25
_launcher = None
_peak = 0.0
_print_t = 0.0
_landed = False


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()   # target = neo_lab.DEFAULT_LAUNCH_HEIGHT

    def start():
        _launcher.reset()
        print(f"Takeoff check: target {neo_lab.DEFAULT_LAUNCH_HEIGHT:.2f} m")

    def _update():
        global _peak, _print_t, _landed
        h = neo_lab.height(_drone)
        _peak = max(_peak, h)
        if not _launcher.done:
            _print_t += _drone.get_delta_time()
            if _print_t >= PRINT_EVERY_S:
                _print_t = 0.0
                print(f"[takeoff] height={h:.2f} m")
            if _launcher.update(_drone):
                over = _peak - neo_lab.DEFAULT_LAUNCH_HEIGHT
                print(f"[takeoff] settled at {h:.2f} m; peak {_peak:.2f} m "
                      f"(overshoot {over:.2f} m above the {neo_lab.DEFAULT_LAUNCH_HEIGHT:.2f} m target)")
        elif not _landed:
            _landed = True
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go(not neo_lab._is_sim(_drone))   # real: run without a controller; sim: wait for ENTER
