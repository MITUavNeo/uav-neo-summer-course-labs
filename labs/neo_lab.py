"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Shared helpers for the simulator labs.

The drone reads a non-zero ground altitude and does not climb from takeoff() alone:
throttle is a vertical-velocity command (about 12 m/s per unit) and stop() holds position.
A flying lab arms with takeoff(), then climbs with throttle to a working height, measuring
altitude relative to the ground sampled at launch. Launcher handles that, and height(drone)
reports altitude above the launch ground.
"""

import csv
import os
import time

import cv2
import numpy as np

import drone_utils as uav_utils

_ground_alt = 0.0


# ── Colored line (line follower) ─────────────────────────────────────────────────────
# The ground line is recolored each run, but always vivid against a grey floor, so HSV
# Saturation isolates it regardless of which color the run picked.

def saturated_mask(image, s_min=100):
    """Binary mask (0/255) of vivid colored regions by HSV Saturation. Color-agnostic,
    so it survives the per-run recoloring of the line."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return (hsv[:, :, 1] > s_min).astype(np.uint8) * 255


# ── Gate markers (ArUco) ─────────────────────────────────────────────────────────────
# Each gate carries four DICT_6X6_250 tags, one per corner. The neon strips share the
# sky's blue hue, so color cannot separate a gate from the background; the tags can.

# One visible tag gives only a gate corner; scale its side up to a rough full-gate span.
_GATE_SPAN_PER_TAG_SIDE = 5.0
_gate_dict = None


def _gate_aruco_dict():
    global _gate_dict
    if _gate_dict is None:
        _gate_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    return _gate_dict


def _detect_gate_markers(gray):
    params = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(_gate_aruco_dict(), params)
        return detector.detectMarkers(gray)
    except AttributeError:                   # OpenCV < 4.7 free-function API
        return cv2.aruco.detectMarkers(gray, _gate_aruco_dict(), parameters=params)


class Gate:
    """A gate located from its corner ArUco tags: image center (cx, cy), inter-tag span
    (gate size proxy), mean tag pixel size (a proximity signal that works with one tag),
    and the decoded corner-tag ids."""

    def __init__(self, cx, cy, span, tag_px, ids):
        self.cx = cx
        self.cy = cy
        self.span = span
        self.tag_px = tag_px
        self.ids = ids
        self.count = len(ids)


def _tag_side(quad):
    p = quad.reshape(-1, 2)
    return float(np.linalg.norm(p[0] - p[1]))


def detect_gate(image):
    """Locate a gate by its DICT_6X6_250 corner tags on the forward camera. Returns a
    Gate, or None when no tag is decoded. The center is the mean of the visible tags:
    exact with all four, approximate with fewer."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detect_gate_markers(gray)
    if ids is None or len(ids) == 0:
        return None
    tag_centers = np.array([c.reshape(-1, 2).mean(axis=0) for c in corners])
    cx, cy = tag_centers.mean(axis=0)
    tag_px = float(np.mean([_tag_side(c) for c in corners]))
    if len(tag_centers) >= 2:
        span = max(np.linalg.norm(a - b) for a in tag_centers for b in tag_centers)
    else:
        span = tag_px * _GATE_SPAN_PER_TAG_SIDE
    return Gate(float(cx), float(cy), float(span), tag_px, [int(i) for i in ids.flatten()])


def set_ground(alt):
    """Record the ground altitude (sampled once at launch)."""
    global _ground_alt
    _ground_alt = alt


def ground():
    """The ground altitude captured at launch (meters, absolute)."""
    return _ground_alt


def height(drone):
    """Altitude above the launch ground, in meters."""
    return drone.physics.get_altitude() - _ground_alt


def world_position(drone):
    """World position (x_east, y_up, z_north) in meters.

    On the sim this is ground truth. On the real drone it is the flight controller's EKF
    local position, whose origin is wherever the EKF initialized rather than a fixed world
    origin -- so absolute values differ from the sim, but relative motion matches.
    """
    return tuple(float(v) for v in drone.physics.get_position())


# ── Velocity commands (portable to the real drone) ────────────────────────────────────
# The labs command a body-frame velocity in m/s. On the real drone that maps straight to
# the ROS2 driver's velocity setpoint (its send_pcmd publishes /mux/cmd_vel). The sim's
# send_pcmd is a TILT command instead, so on the sim a small inner loop turns the velocity
# error into tilt -- the job the real flight controller does in hardware. Writing the labs
# against send_velocity keeps the student controller identical across sim and real.

REAL_MAX_SPEED = 0.5     # m/s mapped to a full normalized command; MUST match mux.yaml max_speed
_SIM_VEL_KP = 0.3        # sim inner loop: tilt per (m/s) of horizontal velocity error
_SIM_VZ_MPS = 12.0       # sim throttle scale: ~12 m/s of vertical velocity per unit throttle
_SIM_TILT_LIMIT = 0.5   # keep tilt gentle: the sim's attitude response is fast and high-authority
_SIM_THROTTLE_LIMIT = 0.5


def _is_sim(drone):
    return "Sim" in type(drone.flight).__name__


def send_velocity(drone, v_right, v_up, v_forward, yaw_rate=0.0):
    """Command a body-frame velocity in m/s: (v_right, v_up, v_forward) plus a yaw rate.

    Same call on the sim and the real drone; only the mapping below differs. On real the
    velocity goes straight to the cmd_vel setpoint; on the sim an inner P loop converts the
    velocity error to tilt (and vertical velocity to throttle).
    """
    if _is_sim(drone):
        vx, vy, vz = (float(v) for v in drone.physics.get_linear_velocity())  # right, up, fwd
        pitch = uav_utils.clamp(_SIM_VEL_KP * (v_forward - vz),
                                -_SIM_TILT_LIMIT, _SIM_TILT_LIMIT)
        roll = uav_utils.clamp(_SIM_VEL_KP * (v_right - vx),
                               -_SIM_TILT_LIMIT, _SIM_TILT_LIMIT)
        throttle = uav_utils.clamp(v_up / _SIM_VZ_MPS,
                                   -_SIM_THROTTLE_LIMIT, _SIM_THROTTLE_LIMIT)
        drone.flight.send_pcmd(pitch, roll, yaw_rate, throttle)
    else:
        drone.flight.send_pcmd(
            uav_utils.clamp(v_forward / REAL_MAX_SPEED, -1.0, 1.0),
            uav_utils.clamp(v_right / REAL_MAX_SPEED, -1.0, 1.0),
            yaw_rate,
            uav_utils.clamp(v_up / REAL_MAX_SPEED, -1.0, 1.0),
        )


_ALT_HOLD_KP = 0.6      # m/s of vertical correction per meter of altitude error (matches module3_trajectory)
_ALT_HOLD_MAX = 1.5     # m/s cap on the altitude-hold correction


def altitude_hold_velocity(drone, target_height):
    """Vertical velocity (m/s) that holds `target_height` above the launch ground. Pass it
    as the v_up argument of send_velocity, so the sim/real vertical scaling is not repeated
    in each lab."""
    return uav_utils.clamp(_ALT_HOLD_KP * (target_height - height(drone)),
                           -_ALT_HOLD_MAX, _ALT_HOLD_MAX)


# Takeoff height in meters above the ground where the program starts. Kept low for constrained
# indoor spaces; every lab that does not pass its own height launches to this.
DEFAULT_LAUNCH_HEIGHT = 1.0


class Launcher:
    """
    Arms the drone once, then climbs to `target_height` meters above the ground measured when
    launching begins under velocity control. Call update(drone) every frame until it returns True.

    takeoff() is called once to arm the motors (velocity commands alone do not arm the sim). In
    the sim takeoff() also imparts an upward impulse, so the climb overshoots the target before
    settling back to it -- a known sim artifact; on the real drone the velocity setpoints drive a
    controlled OFFBOARD climb.
    """

    def __init__(self, target_height=DEFAULT_LAUNCH_HEIGHT, climb_kp=1.0, max_climb_speed=2.0,
                 tol=0.25, settle=1.0):
        self.target_height = target_height
        self.climb_kp = climb_kp
        self.max_climb_speed = max_climb_speed
        self.tol = tol
        self.settle = settle
        self.reset()

    def reset(self):
        self._hold = 0.0
        self._ground_set = False
        self._armed = False
        self.done = False

    def skip(self, drone):
        """Mark the climb complete and run from wherever the drone already is."""
        set_ground(drone.physics.get_altitude())
        self._ground_set = True
        self.done = True

    def update(self, drone):
        if self.done:
            return True
        dt = drone.get_delta_time()
        if not self._ground_set:
            set_ground(drone.physics.get_altitude())
            self._ground_set = True

        # Arm once (velocity commands alone do not arm the sim), then climb under velocity control.
        if not self._armed:
            drone.flight.takeoff()
            self._armed = True

        err = self.target_height - height(drone)
        v_up = uav_utils.clamp(self.climb_kp * err, -self.max_climb_speed,
                               self.max_climb_speed)
        send_velocity(drone, 0, v_up, 0)
        self._hold = self._hold + dt if abs(err) < self.tol else 0.0
        if self._hold >= self.settle:
            drone.flight.stop()
            self.done = True
            print(f"[launch] airborne {height(drone):.2f} m above ground "
                  f"(ground={ground():.2f} m)")
        return self.done


# ── Flight recording (opt-in) ────────────────────────────────────────────────────────
# Set NEO_RECORD=<path>.csv and call neo_lab.record(drone) once per frame in a lab's
# update loop. Each row (time, height, velocity, heading, dead-reckoned x/z, plus any
# extra= channels) is written and flushed immediately, so data survives even if the run
# is stopped early. Plot the CSV afterward with labs/plot_log.py.

class Recorder:
    """Writes per-frame telemetry rows to a CSV (columns fixed by the first row)."""

    def __init__(self, path):
        self._file = open(path, "w", newline="")
        self._writer = None
        self._fields = None

    def log(self, **values):
        if self._writer is None:
            self._fields = list(values.keys())
            self._writer = csv.DictWriter(self._file, fieldnames=self._fields,
                                          extrasaction="ignore")
            self._writer.writeheader()
        self._writer.writerow({k: values.get(k, "") for k in self._fields})
        self._file.flush()


_recorder = None
_rec_t0 = None


def record(drone, **extra):
    """If NEO_RECORD is set, append one telemetry row; otherwise do nothing.

    Universal channels: t, height, vx, vy, vz (body-frame velocity), heading, and
    x/z (TRUE world east/north position from world_position, so trajectory plots do
    not drift). Pass extra named channels (e.g. gate_width=...) to log lab-specific
    values alongside.
    """
    global _recorder, _rec_t0
    path = os.environ.get("NEO_RECORD")
    if not path:
        return
    now = time.time()
    if _recorder is None:
        _recorder = Recorder(path)
        _rec_t0 = now
    vx, vy, vz = (float(v) for v in drone.physics.get_linear_velocity())
    _, _, yaw = (float(a) for a in drone.physics.get_attitude())
    x_east, _y_up, z_north = world_position(drone)
    row = {
        "t": round(now - _rec_t0, 3),
        "height": round(height(drone), 3),
        "vx": round(vx, 3), "vy": round(vy, 3), "vz": round(vz, 3),
        "heading": round(yaw, 2),
        "x": round(x_east, 3), "z": round(z_north, 3),
    }
    row.update(extra)
    _recorder.log(**row)


LED_BLINK_PERIOD_S = 0.5   # full on+off cycle of the flying indicator
LAUNCH_SKIP_ENV = "NEO_NO_LAUNCH"
AUTOSTART_ENV = "NEO_AUTOSTART"


def _launch_enabled(launch):
    if launch is not None:
        return launch
    return not os.environ.get(LAUNCH_SKIP_ENV, "")


def _autostart_enabled(autostart, drone):
    if autostart is not None:
        return autostart
    env = os.environ.get(AUTOSTART_ENV, "")
    if env != "":
        return env not in ("0", "false", "False")
    return not _is_sim(drone)   # real drone runs controller-free; the sim waits for its start key


def run_module(title, steps, launch_height=DEFAULT_LAUNCH_HEIGHT, autostart=None, led_color=None,
               launch=None):
    """Standard lab orchestrator: create the drone, arm and climb, then run each step in
    order and land. `steps` is a list of (label, module) where each module has reset()
    and update(drone) -> done. Records telemetry when NEO_RECORD is set.

    Each lab's main.py / main_solution.py is a thin wrapper that imports its step modules
    and calls this, so the orchestration lives in one place.

    autostart runs the program without the START button / a game controller (the real
    drone's safety pilot still gates motion via OFFBOARD; stop with Ctrl-C). Left None it
    defaults to on for the real drone and off in the simulator (which waits for its start key);
    set NEO_AUTOSTART=1 or 0 to force it either way.

    led_color, if given as an (r, g, b) tuple, blinks the LED strip while the drone is
    flying and turns it off on landing. Leave it None to let a step drive the strip
    itself (the shape flight holds it solid for a long exposure).

    launch=False skips the arm-and-climb phase and runs the steps at whatever altitude the
    drone is already holding, for when the safety pilot flies it up by hand. Leaving it
    None takes the default from the NEO_NO_LAUNCH environment variable, so the whole lab
    set can be switched over without editing each main.py.
    """
    import drone_core
    drone = drone_core.create_drone()
    launching = _launch_enabled(launch)
    launcher = Launcher(launch_height)
    state = {"i": 0}
    blink = {"clock": 0.0, "on": None}

    def start():
        state["i"] = 0
        launcher.reset()
        blink["clock"] = 0.0
        blink["on"] = None
        print("\n" + "=" * 56)
        print(f"  {title}")
        print("=" * 56 + "\n")
        if not launching:
            launcher.skip(drone)
            steps[0][1].reset()
            print("[launch] skipped; running at the current altitude")
            print(f"--- {steps[0][0]} ---")

    def blink_led(landing):
        if led_color is None:
            return
        if landing:
            if blink["on"] is not False:
                blink["on"] = False
                drone.led.off()
            return
        blink["clock"] += drone.get_delta_time()
        on = (blink["clock"] % LED_BLINK_PERIOD_S) < LED_BLINK_PERIOD_S / 2.0
        if on != blink["on"]:
            blink["on"] = on
            drone.led.fill(*led_color) if on else drone.led.off()

    def update():
        record(drone)
        blink_led(landing=launcher.done and state["i"] >= len(steps))
        if not launcher.done:
            if launcher.update(drone):
                steps[0][1].reset()
                print(f"--- {steps[0][0]} ---")
            return
        if state["i"] >= len(steps):
            drone.flight.land()
            return
        if steps[state["i"]][1].update(drone):
            state["i"] += 1
            if state["i"] < len(steps):
                steps[state["i"]][1].reset()
                print(f"\n--- {steps[state['i']][0]} ---")
            else:
                print("\n=== Module complete! Landing... ===")

    def update_slow():
        if not launcher.done:
            waiting = "" if _is_sim(drone) else "  (waiting: safety pilot must arm + OFFBOARD)"
            print(f"[launch] climbing to {launch_height:.1f}m, height={height(drone):.2f}m{waiting}")
        elif state["i"] < len(steps):
            print(f"[{steps[state['i']][0]}] height={height(drone):.2f}m")

    drone.set_start_update(start, update, update_slow)
    drone.go(_autostart_enabled(autostart, drone))
