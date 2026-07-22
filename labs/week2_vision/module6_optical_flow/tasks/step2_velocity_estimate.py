"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 6 — Step 2: Velocity from Optical Flow
Turn the tracked features' average motion into a velocity estimate and compare it
against the drone's true velocity. The flow is in pixels per processed interval;
converting to meters/second needs the ground footprint of one pixel (grows with
altitude) and the time between processed frames.
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
HFOV_TAN = 1.0         # tan(half of a 90 deg horizontal field of view)
PROBE_SPEED = 0.4      # m/s forward drift; a steady slow speed keeps the flow trackable
RUN_TIME = 6.0
SKIP = 2               # do the vision work every Nth frame
MIN_PTS = 20
FEATURE_PARAMS = dict(maxCorners=80, qualityLevel=0.01, minDistance=8, blockSize=7)
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# -- Module-level state -----------------------------------------------------
_prev_gray = None
_prev_pts = None
_timer = 0.0
_interval = 0.0        # time accumulated since the last processed frame
_frame = 0
_done = False

def reset():
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    _prev_gray = None
    _prev_pts = None
    _timer = 0.0
    _interval = 0.0
    _frame = 0
    _done = False


def update(drone):
    global _prev_gray, _prev_pts, _timer, _interval, _frame, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: print an estimated horizontal velocity from optical flow next to the true
    # velocity, so you can see how well vision tracks motion.
    #
    # Tools: drone.camera.get_downward_image(); neo_lab.height(drone);
    #        drone.physics.get_linear_velocity(); drone.get_delta_time();
    #        neo_lab.send_velocity(drone, 0, 0, PROBE_SPEED);
    #        plus the sparse optical-flow tracking you built in Step 1.
    #
    # Every frame: drift at PROBE_SPEED, add dt to _timer AND to _interval, and _frame += 1.
    # Only every SKIP-th frame: track corner points (sparse flow, like Step 1) and average
    # the kept points' displacement in pixels. Convert that to meters/second: one pixel's
    # ground footprint grows with height and the camera's field of view (use HFOV_TAN and
    # IMAGE_WIDTH), and divide by _interval (the time between PROCESSED frames, not one dt);
    # then reset _interval. Watch the signs so the estimate moves the same direction as the
    # true velocity (one image axis may be flipped). Finish at RUN_TIME, printing the estimate
    # vs. true velocity. See the README (Key terms).

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 2: Velocity from Optical Flow")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
