"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

CONCEPT lab — no simulator needed. Fill in p_control, then run directly:
    python3 p_control.py
Then move on to step1_altitude_hold.py to use the same idea on the real drone.

A completed reference lives in ../solutions/p_control.py
"""

import numpy as np


def p_control(y_err, kp):
    """Proportional control law:  command = kp * error."""
    ##################################
    #### START PUT CODE HERE #########
    return 0.0  # YOUR CODE HERE (return kp * y_err)
    ###### END PUT CODE HERE #########
    ##################################


class SimpleSlideCamera:
    """A 1-D 'camera on a slide' whose velocity is the commanded value."""

    def __init__(self, x0=0.0):
        self.x = x0

    def step(self, command, dt):
        self.x += command * dt
        return self.x


def _check():
    passed = total = 0

    def ok(name, cond):
        nonlocal passed, total
        total += 1
        passed += bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    ok("p_control(0, 1) == 0", np.isclose(p_control(0.0, 1.0), 0.0))
    ok("p_control(1, 1) == 1", np.isclose(p_control(1.0, 1.0), 1.0))
    ok("p_control(2, 0.5) == 1", np.isclose(p_control(2.0, 0.5), 1.0))

    cam = SimpleSlideCamera(0.0)
    target, kp, dt = 5.0, 0.8, 0.05
    for _ in range(2000):
        cam.step(p_control(target - cam.x, kp), dt)
    ok("closed loop converges to target", np.isclose(cam.x, target, atol=1e-2))

    print(f"\n{passed}/{total} checks passed.")
    return passed == total


if __name__ == "__main__":
    print("Week 3 · Module 2 — Proportional Control\n")
    _check()
