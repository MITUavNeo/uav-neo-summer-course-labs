"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

This is a CONCEPT lab — it does not need the simulator.
Run it directly:
    python3 coordinate_frames.py
It prints PASS/FAIL for each part's self-check.
"""

import numpy as np


# ── Part A: Euler angles -> rotation matrix ─────────────────────────────────────────
def euler_to_rot(roll, pitch, yaw):
    """
    Build a body->world rotation matrix from Euler angles (radians) using the
    aerospace ZYX convention:  R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ── Part A: rotation matrix -> quaternion ───────────────────────────────────────────
def rot_to_quat(R):
    """
    Convert a 3x3 rotation matrix to a quaternion (scalar-last: x, y, z, w).
    """
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = (R[2, 1] - R[1, 2]) / (4.0 * w)
    y = (R[0, 2] - R[2, 0]) / (4.0 * w)
    z = (R[1, 0] - R[0, 1]) / (4.0 * w)
    return np.array([x, y, z, w])


# ── Part 0: static frame transform (ENU <-> NED) ────────────────────────────────────
def enu_to_ned(vec):
    """
    Convert a vector from ENU (East, North, Up) to NED (North, East, Down).
        ned = [east_north_up[1], east_north_up[0], -east_north_up[2]]
    """
    e, n, u = vec
    return np.array([n, e, -u])


# ── Part B: point-mass thrust sizing ────────────────────────────────────────────────
def thrust_allocation(mass, k_f, total_thrust):
    """
    Split a total thrust evenly across 4 rotors and solve for rotor speed.
        thrust_per_motor = total_thrust / 4
        omega = sqrt(thrust_per_motor / k_f)         (k_f: thrust coefficient)
    Returns: (omega, thrust_per_motor).
    """
    per = total_thrust / 4.0
    omega = np.sqrt(per / k_f)
    return omega, per


def hover_thrust(mass, g=9.81):
    """Total thrust (N) needed to hover: T = m * g."""
    return mass * g


# ── Self-check ──────────────────────────────────────────────────────────────────────
def _check():
    passed = total = 0

    def ok(name, cond, detail=""):
        nonlocal passed, total
        total += 1
        passed += bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}")

    R0 = euler_to_rot(0, 0, 0)
    ok("euler_to_rot identity", np.allclose(R0, np.eye(3)))

    R = euler_to_rot(0.3, -0.2, 1.0)
    ok("rotation is orthonormal", np.allclose(R.T @ R, np.eye(3)) and
       np.isclose(np.linalg.det(R), 1.0))

    Ryaw = euler_to_rot(0, 0, np.pi / 2)
    ok("90deg yaw maps x->y", np.allclose(Ryaw @ np.array([1, 0, 0]),
                                          [0, 1, 0], atol=1e-9))

    q = rot_to_quat(np.eye(3))
    ok("rot_to_quat identity -> (0,0,0,1)", np.allclose(q, [0, 0, 0, 1]))

    ok("enu_to_ned", np.allclose(enu_to_ned([1, 2, 3]), [2, 1, -3]))

    omega, per = thrust_allocation(1.0, 1.0, 4.0)
    ok("thrust_allocation", np.isclose(per, 1.0) and np.isclose(omega, 1.0),
       f"(omega={omega:.3f}, per={per:.3f})")

    ok("hover_thrust", np.isclose(hover_thrust(2.0, 9.81), 19.62))

    print(f"\n{passed}/{total} checks passed.")
    return passed == total


if __name__ == "__main__":
    print("Week 3 · Module 1 — Coordinate Frames & Dynamics (SOLUTION)\n")
    _check()
