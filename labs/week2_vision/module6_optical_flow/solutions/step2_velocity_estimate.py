"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 6 — Step 2: Velocity from Optical Flow  (SOLUTION)
Sparse Lucas-Kanade, processed every SKIP-th frame. The interval used to convert
pixels/frame -> m/s is the time between PROCESSED frames (accumulated dt), not one
sim frame. Scale and signs are approximate; the point is tracking the true velocity.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

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
IMAGE_WIDTH = 640
HFOV_TAN = 1.0
PROBE_SPEED = 0.4      # m/s forward drift; a steady slow speed keeps the flow trackable
RUN_TIME = 6.0
SKIP = 2
MIN_PTS = 20
FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.01, minDistance=8, blockSize=7)
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# -- Module-level state -----------------------------------------------------
_prev_gray = None
_prev_pts = None
_timer = 0.0
_interval = 0.0          # time accumulated since the last processed frame
_frame = 0
_est = (0.0, 0.0)
_true = (0.0, 0.0)
_done = False

def reset():
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _est, _true, _done
    _prev_gray = None
    _prev_pts = None
    _timer = 0.0
    _interval = 0.0
    _frame = 0
    _est = (0.0, 0.0)
    _true = (0.0, 0.0)
    _done = False


def update(drone):
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _est, _true, _done
    if _done:
        return True
    dt = drone.get_delta_time()
    neo_lab.send_velocity(drone, 0, 0, PROBE_SPEED)
    _timer += dt
    _interval += dt
    _frame += 1
    if _frame % SKIP == 0:
        gray = cv2.cvtColor(drone.camera.get_downward_image(), cv2.COLOR_BGR2GRAY)
        if _prev_gray is None or _prev_pts is None or len(_prev_pts) < MIN_PTS or _interval <= 0:
            _prev_pts = cv2.goodFeaturesToTrack(gray, **FEATURE_PARAMS)
        else:
            new_pts, status, _err = cv2.calcOpticalFlowPyrLK(_prev_gray, gray,
                                                             _prev_pts, None, **LK_PARAMS)
            if new_pts is not None and status is not None:
                keep = status.flatten() == 1
                good_new = new_pts[keep].reshape(-1, 2)
                good_old = _prev_pts[keep].reshape(-1, 2)
                if len(good_new) > 0:
                    disp = good_new - good_old
                    mean_dx = float(disp[:, 0].mean())
                    mean_dy = float(disp[:, 1].mean())
                    height = max(neo_lab.height(drone), 0.1)
                    mpp = 2.0 * height * HFOV_TAN / IMAGE_WIDTH
                    _est = (-mean_dx * mpp / _interval, mean_dy * mpp / _interval)
                    vx, vy, vz = drone.physics.get_linear_velocity()
                    _true = (float(vx), float(vz))
                _prev_pts = good_new.reshape(-1, 1, 2)
        _prev_gray = gray
        _interval = 0.0
    if _timer >= RUN_TIME:
        drone.flight.stop()
        print(f"[Step 2] flow est (x,z)=({_est[0]:.2f},{_est[1]:.2f})  "
              f"true (x,z)=({_true[0]:.2f},{_true[1]:.2f}) m/s")
        _done = True
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Velocity from Optical Flow")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
