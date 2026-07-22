# Diagnostics

Standalone check scripts for verifying drone behavior outside a lab. They are not part of the
curriculum sequence and teach nothing on their own; use them to sanity-check the hardware and
the launch before running the labs.

Run them like any lab:

```bash
# simulator (press ENTER in the sim window)
drone sim course/diagnostics/takeoff_check.py

# real drone (from this folder on the Pi)
python3 takeoff_check.py
```

- **`takeoff_check.py`** — runs only the launch and prints the climb profile and how far it
  overshoots the target height (`neo_lab.DEFAULT_LAUNCH_HEIGHT`). Lands after settling.
- **`camera_check.py`** — grabs one frame from each camera and runs the labs' vision on it
  (ArUco gate detection, saturated-line mask). Reports the image shapes and, if a camera returns
  no frame, which one. Does not fly.
