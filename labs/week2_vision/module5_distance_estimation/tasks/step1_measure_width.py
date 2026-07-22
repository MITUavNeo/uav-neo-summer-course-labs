"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2 · Module 5 — Step 1: Measure a Gate Tag's Apparent Size
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
SEARCH_PITCH   = 0.1        # creep forward; ArUco tags only resolve up close
SEARCH_TIMEOUT = 15.0       # give up if no gate decodes in this many seconds

# -- Module-level state -----------------------------------------------------
_timer = 0.0
_done = False

def reset():
    global _timer, _done
    _timer = 0.0
    _done = False


def update(drone):
    global _timer, _done
    if _done:
        return True
    _timer += drone.get_delta_time()
    ##################################
    #### START PUT CODE HERE #########

    # GOAL: creep forward until a gate is detected, then report the apparent size of its
    # corner tags in pixels (this shrinks with distance -- Step 2 turns it into range).
    #
    # Tools: drone.camera.get_color_image(); neo_lab.detect_gate(image) -> Gate or None;
    #        a Gate has .tag_px (mean tag size in px) and .count; drone.flight.send_pcmd(...)
    #        and drone.flight.stop().
    #
    # Send SEARCH_PITCH forward each frame until detect_gate returns a Gate, then stop and
    # print .tag_px. Give up after SEARCH_TIMEOUT seconds.

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher()

    def start():
        _launcher.reset()
        reset()
        print("Step 1: Measure a Gate Tag's Apparent Size")

    def _update():
        if not _launcher.done:
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
