"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 4 · Module 3 — Step 2: Fly a Smooth Course
Chain the waypoints into ONE smooth trajectory (a cubic spline) so the drone sweeps through
without stopping -- a racing line. Position is dead-reckoned from velocity, so it drifts.
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
WAYPOINTS = [(0.0, 0.0), (2.0, 3.0), (-1.0, 5.0), (2.0, 7.0), (0.0, 9.0)]  # (right, forward)
SEG_TIME = 3.5        # seconds per waypoint-to-waypoint segment
TARGET_HEIGHT = 1.0
KP_POS = 0.6          # position error -> velocity (1/s)
ALT_KP = 0.6          # altitude error -> vertical velocity (1/s)
_VEL_DT = 0.02        # finite-difference step for reading the trajectory's velocity

_N_SEG = len(WAYPOINTS) - 1
TOTAL_TIME = SEG_TIME * _N_SEG

# -- Module-level state -----------------------------------------------------
_t = 0.0
_x = 0.0
_z = 0.0
_max_err = 0.0
_wp = 0
_done = False


def _tangents(points):
    """Catmull-Rom tangents: at each waypoint the path heads toward the NEXT-but-one point,
    so the drone sweeps through smoothly instead of stopping. Endpoints use a one-sided slope.
    """
    n = len(points)
    tan = []
    for i in range(n):
        if i == 0:
            m = points[1] - points[0]
        elif i == n - 1:
            m = points[-1] - points[-2]
        else:
            m = 0.5 * (points[i + 1] - points[i - 1])
        tan.append(m)
    return tan


_RIGHT = [w[0] for w in WAYPOINTS]
_FWD = [w[1] for w in WAYPOINTS]
_TAN_RIGHT = _tangents(_RIGHT)
_TAN_FWD = _tangents(_FWD)


def hermite(p0, m0, p1, m1, s):
    """Cubic Hermite blend at fraction s in [0,1] between p0 (tangent m0) and p1 (tangent m1)."""
    ##################################
    #### START PUT CODE HERE #########
    # Combine the four Hermite basis functions of s with p0, m0, p1, m1 to return one
    # blended value. See the README ("Building a smooth path") for the four basis
    # functions h00, h10, h01, h11 and how they weight the endpoints and tangents.
    result = p0
    ###### END PUT CODE HERE #########
    ##################################
    return result


def path_position(t):
    """Position (right, forward) on the spline at time t seconds."""
    k = min(int(t / SEG_TIME), _N_SEG - 1)
    s = min(max((t - k * SEG_TIME) / SEG_TIME, 0.0), 1.0)
    right = hermite(_RIGHT[k], _TAN_RIGHT[k], _RIGHT[k + 1], _TAN_RIGHT[k + 1], s)
    fwd = hermite(_FWD[k], _TAN_FWD[k], _FWD[k + 1], _TAN_FWD[k + 1], s)
    return right, fwd


def _ease(t):
    """Smoothstep time-warp over the whole course so the spline starts and ends at zero
    speed while still flowing through the interior waypoints. Without it the path leaves the
    first waypoint at the tangent's speed, so the drone lurches out of the previous maneuver
    at the step boundary."""
    u = min(max(t / TOTAL_TIME, 0.0), 1.0)
    return TOTAL_TIME * u * u * (3.0 - 2.0 * u)


def trajectory(t):
    """Desired (right, forward) position and velocity on the spline at time t."""
    pos_r, pos_f = path_position(_ease(t))
    nr, nf = path_position(_ease(t + _VEL_DT))
    return pos_r, pos_f, (nr - pos_r) / _VEL_DT, (nf - pos_f) / _VEL_DT


def reset():
    global _t, _x, _z, _max_err, _wp, _done
    _t = 0.0
    _x = 0.0
    _z = 0.0
    _max_err = 0.0
    _wp = 0
    _done = False


def update(drone):
    global _t, _x, _z, _max_err, _wp, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    _t += dt
    vx, vy, vz = drone.physics.get_linear_velocity()
    _x += vx * dt
    _z += vz * dt
    pos_r, pos_f, vel_r, vel_f = trajectory(_t)
    v_right = vel_r + KP_POS * (pos_r - _x)
    v_forward = vel_f + KP_POS * (pos_f - _z)
    v_up = ALT_KP * (TARGET_HEIGHT - neo_lab.height(drone))
    neo_lab.send_velocity(drone, v_right, v_up, v_forward)
    err = ((pos_r - _x) ** 2 + (pos_f - _z) ** 2) ** 0.5
    _max_err = max(_max_err, err)
    reached = min(int(_ease(_t) / SEG_TIME) + 1, _N_SEG)
    if reached > _wp:
        _wp = reached
        print(f"[Step 2] Reached waypoint {_wp}/{_N_SEG}")
    if _t >= TOTAL_TIME:
        drone.flight.stop()
        print(f"[Step 2] Course done: right={_x:.2f} forward={_z:.2f} m, "
              f"max tracking error {_max_err:.2f} m")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(TARGET_HEIGHT)

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Fly a Smooth Course")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
