"""
Diagnostic: report what the DOWNWARD camera sees so the line-follower (module 3) and
downward-object (module 4) labs can be tuned to the current sim build. Load a line or
object scene, run this, and the drone creeps forward while sampling the ground below.
Prints brightness vs. color coverage, the dominant color, and the largest blob. Saves
/tmp/probe_down.png. Ctrl-C to stop. Not a lab.
"""

import drone_core
import numpy as np
import cv2

import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.realpath(__file__))
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

FRAMES_PER_REPORT = 45
FORWARD_PITCH     = 0.15   # fly forward to reach a line ahead, then keep features passing under
SAT_MIN           = 60     # ignore washed-out/grey pixels when profiling a color
VAL_MIN           = 60     # ignore near-black pixels
BRIGHT_V          = 200    # brightness threshold, shown for comparison with saturation
HUE_BINS          = 36     # 5-degree bins across the OpenCV 0..179 hue range
MIN_BLOB          = 200    # px, ignore specks when reporting the largest blob
_frame = 0


def _report(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    total = h.size
    bright = int((v > BRIGHT_V).sum())
    keep = (s > SAT_MIN) & (v > VAL_MIN)
    kept = int(keep.sum())
    msg = (f"[down] bright(V>{BRIGHT_V})={100.0 * bright / total:4.1f}%  "
           f"saturated={100.0 * kept / total:4.1f}%")
    if kept:
        hist, edges = np.histogram(h[keep], bins=HUE_BINS, range=(0, 180))
        top = int(np.argmax(hist))
        lo, hi = int(edges[top]), int(edges[top + 1])
        band = keep & (h >= lo) & (h < hi)
        rows, cols = np.nonzero(band)
        msg += (f"  dom hue {lo}-{hi} (H~{int(np.median(h[band]))} "
                f"S~{int(np.median(s[band]))} V~{int(np.median(v[band]))})  "
                f"col_mean={int(cols.mean())}")
        mask = (band.astype(np.uint8)) * 255
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = max(contours, key=cv2.contourArea, default=None)
        if best is not None and cv2.contourArea(best) >= MIN_BLOB:
            x, y, w, hgt = cv2.boundingRect(best)
            msg += f"  largest blob=({x},{y},{w},{hgt}) area={int(cv2.contourArea(best))}"
    print(msg)
    cv2.imwrite("/tmp/probe_down.png", image)


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        print("Probe: downward camera profile (Ctrl-C to stop)")

    def _update():
        global _frame
        _frame += 1
        if not _launcher.done:
            _launcher.update(_drone)
        else:
            _drone.flight.send_pcmd(FORWARD_PITCH, 0, 0, 0)   # creep so features pass under
        if _frame % FRAMES_PER_REPORT == 0:
            _report(_drone.camera.get_downward_image())

    _drone.set_start_update(start, _update)
    _drone.go()
