# Diagnostics

Standalone check scripts for verifying drone behavior outside a lab. **Run both before your
first lab on a drone, and again after any update**: `camera_check.py` first (no flight), then
`takeoff_check.py` (a minimal flight). They are not part of the curriculum sequence; they
verify that the hardware and the launch the labs depend on actually work.

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
