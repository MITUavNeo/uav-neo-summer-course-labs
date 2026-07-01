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


# ── Gate vision ─────────────────────────────────────────────────────────────────────
# Gates are dark frames with glowing edges — cyan on the forward camera, white on the
# downward camera. Both read as high "Value" in HSV, so brightness is the strongest
# gate signal.

# Cyan gate edges on the forward camera (separates from the blue background ~hue 108).
CYAN_LOWER = np.array([80, 40, 150], dtype=np.uint8)
CYAN_UPPER = np.array([105, 255, 255], dtype=np.uint8)


def bright_mask(image, v_min=200):
    """Binary mask (0/255) of the glowing gate edges, by HSV Value (brightness)."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return (hsv[:, :, 2] > v_min).astype(np.uint8) * 255


def largest_bright_contour(image, v_min=200, min_area=200, dilate=2):
    """Return the largest glowing-edge contour in the image, or None."""
    mask = bright_mask(image, v_min)
    if dilate:
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=dilate)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, float(min_area)
    for c in contours:
        area = cv2.contourArea(c)
        if area > best_area:
            best, best_area = c, area
    return best


def _gate_in_mask(mask, min_area=400, max_aspect=2.5, dilate=2):
    """
    Largest roughly-SQUARE bright contour in a binary mask, or None.
    Gates are square frames (aspect ~1); the long glowing boundary lines have a
    very elongated bounding box, so an aspect-ratio test rejects them.
    """
    if dilate:
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=dilate)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, float(min_area)
    for c in contours:
        area = cv2.contourArea(c)
        if area <= best_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w == 0 or h == 0 or max(w, h) / min(w, h) > max_aspect:
            continue                       # too elongated -> a boundary line, not a gate
        best, best_area = c, area
    return best


def largest_gate(image, v_min=200, min_area=400, max_aspect=2.5):
    """Largest square-ish GLOWING gate (forward: cyan / downward: white), or None."""
    return _gate_in_mask(bright_mask(image, v_min), min_area, max_aspect)


def largest_cyan_gate(image, min_area=400, max_aspect=2.5):
    """Largest square-ish CYAN gate on the forward camera, or None."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, CYAN_LOWER, CYAN_UPPER)
    return _gate_in_mask(mask, min_area, max_aspect)


def gate_nearest_to(image, target_col, v_min=200, min_area=500, max_aspect=2.5,
                    dilate=2):
    """
    Square-ish glowing gate whose center column is closest to `target_col`, or None.

    For yaw visual-servoing in a field of similar gates, picking the LARGEST gate
    flickers between them. Track ONE gate instead: pass the previously-tracked gate's
    column as `target_col` (start at the image center) and update it each frame. The
    loop then locks onto a single gate and follows it as the drone turns.
    """
    mask = bright_mask(image, v_min)
    if dilate:
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=dilate)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_dist = None, float("inf")
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w == 0 or h == 0 or max(w, h) / min(w, h) > max_aspect:
            continue
        dist = abs((x + w / 2.0) - target_col)
        if dist < best_dist:
            best, best_dist = c, dist
    return best


def gate_nearest_center(image, v_min=200, min_area=500, max_aspect=2.5, dilate=2):
    """Square-ish gate nearest the image center (a one-shot gate_nearest_to)."""
    return gate_nearest_to(image, image.shape[1] / 2.0, v_min, min_area,
                           max_aspect, dilate)


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
    """True world position (x_east, y_up, z_north) in meters, straight from the sim.

    Uses the drone's direct position readout (no drift, no GPS round-trip). Requires a
    simulator build new enough to support it.
    """
    return tuple(float(v) for v in drone.physics.get_position())


class Launcher:
    """
    Arms the drone and climbs to `target_height` meters above the ground measured
    when launching begins. Call update(drone) every frame until it returns True.

    Throttle is a velocity command, so the proportional gain is intentionally small.
    """

    def __init__(self, target_height=3.0, kp=0.2, throttle_limit=0.5,
                 tol=0.4, arm_time=1.5, settle=1.0):
        self.target_height = target_height
        self.kp = kp
        self.throttle_limit = throttle_limit
        self.tol = tol
        self.arm_time = arm_time
        self.settle = settle
        self.reset()

    def reset(self):
        self._t = 0.0
        self._hold = 0.0
        self._ground_set = False
        self.done = False

    def update(self, drone):
        if self.done:
            return True
        dt = drone.get_delta_time()
        if not self._ground_set:
            set_ground(drone.physics.get_altitude())
            self._ground_set = True

        # Phase 1: arm the motors.
        self._t += dt
        if self._t < self.arm_time:
            drone.flight.takeoff()
            return False

        # Phase 2: climb to target height (throttle ~ vertical velocity).
        err = self.target_height - height(drone)
        throttle = uav_utils.clamp(self.kp * err, -self.throttle_limit,
                                   self.throttle_limit)
        drone.flight.send_pcmd(0, 0, 0, throttle)
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


def run_module(title, steps, launch_height=3.0):
    """Standard lab orchestrator: create the drone, arm and climb, then run each step in
    order and land. `steps` is a list of (label, module) where each module has reset()
    and update(drone) -> done. Records telemetry when NEO_RECORD is set.

    Each lab's main.py / main_solution.py is a thin wrapper that imports its step modules
    and calls this, so the orchestration lives in one place.
    """
    import drone_core
    drone = drone_core.create_drone()
    launcher = Launcher(launch_height)
    state = {"i": 0}

    def start():
        state["i"] = 0
        launcher.reset()
        print("\n" + "=" * 56)
        print(f"  {title}")
        print("=" * 56 + "\n")

    def update():
        record(drone)
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
        if launcher.done and state["i"] < len(steps):
            print(f"[{steps[state['i']][0]}] height={height(drone):.2f}m")

    drone.set_start_update(start, update, update_slow)
    drone.go()
