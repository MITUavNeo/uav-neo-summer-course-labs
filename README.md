# UAV Neo Summer Course Labs

MIT BWSI Autonomous Drone Racing Course — UAV Neo

Program an autonomous drone in the **UAV Neo simulator**. The course runs four weeks:
Week 1 is hands-on — building your drone, learning to fly, and team building — and
Weeks 2–4 (these labs) are where you program it to see, fly, and navigate on its own:

- **Week 2 — Vision:** find and measure things in the drone's camera images.
- **Week 3 — Controls:** turn sensor readings into smooth, stable flight.
- **Week 4 — Integration:** combine vision and control into multi-axis maneuvers.

Each lab hands you a small piece of code to complete; you run it and watch the drone fly.
The math labs check your work with `PASS`/`FAIL` instead.

---

## 1. Getting started

You need the UAV Neo tooling installed (the `drone` command, the Python library, and the
simulator) from [`uav-neo-installer`](../uav-neo-installer). Once that is set up:

```bash
# 1. Put these labs where the `drone` tool can find them (one time).
#    The installer symlinks this repo in as labs/course, so you can run:
drone sim course/week3_controls/module2_feedback_control/main.py
#    (or copy the contents of labs/ into your drone-student/labs/ folder)

# 2. Launch the simulator window (once per session).
drone open_sim

# 3. Run a lab. The Python side starts and waits for the simulator.
drone sim course/week3_controls/module2_feedback_control/main.py

# 4. Click the simulator window and press ENTER to start the program.
#    (That hands control to your code — the drone arms and the lab begins.)
```

> **The `Enter` key is required every run.** The Python script connects to the simulator
> and prints *"...Enter user program mode to begin"*. Nothing happens until you press
> `Enter` in the sim window.

The math-only labs don't need the simulator — run them directly with `python3` (below).

---

## 2. How a lab is structured

Each lab is a **module** split into small **steps**. Every step file has one clearly
marked spot for your code:

```python
##################################
#### START PUT CODE HERE #########

# YOUR CODE HERE   (the comments above give you the algorithm + hints)

###### END PUT CODE HERE #########
##################################
```

| Folder / file | What it is |
|---------------|------------|
| `tasks/`           | **Your** files — the blanks you fill in |
| `solutions/`       | Completed reference implementations (peek only if you're stuck!) |
| `main.py`          | Runs all the steps in order using your `tasks/` code |
| `main_solution.py` | Same, but runs the `solutions/` code (instructor reference flight) |
| `README.md`        | What the module teaches and how to run it |

Run options:

```bash
drone sim course/<week>/<module>/main.py            # all steps, your code
drone sim course/<week>/<module>/main_solution.py   # all steps, reference
drone sim course/<week>/<module>/tasks/<step>.py    # just one step, your code
```

## Two kinds of labs

- **Concept labs** (pure Python, no drone) teach the underlying math and print `PASS`/`FAIL`:
  ```bash
  python3 labs/week2_vision/module1_image_formation/tasks/image_formation.py
  ```
- **Simulator labs** fly the drone using the live camera / flight / physics APIs (run with
  `drone sim`, as above).

---

## 3. Flying in the simulator — what you need to know

The simulated drone does **not** behave like a simple "go to (x, y, z)" robot. A few facts
about how it flies that you'll rely on when writing your controllers:

- **`get_altitude()` is absolute, not height above the ground.** It reports the drone's
  world altitude, and the drone is usually already airborne when your program takes over
  (you launch it, then press Enter). So measure height **relative to a reference sampled
  when your program starts** — use `neo_lab.height(drone)` (the Launcher records that
  reference), not the raw altitude.
- **`takeoff()` arms the motors but does not climb on its own.** To get airborne you give
  throttle. The shared `neo_lab.Launcher` does this for you (arm, then climb to a safe
  height) — every simulator lab's `main.py` runs it before your steps.
- **Flight commands are like speed commands.** `send_pcmd(pitch, roll, yaw, throttle)` with
  each value in `[-1, 1]`: throttle sets vertical speed, pitch sets forward speed, etc.
  `stop()` (all zeros) makes the drone hold its position. **Small commands (below ~0.05) do
  nothing** — there's a deadband — so a pure proportional controller settles a little short
  of its target (which is exactly why the PID lab adds an integral term).
- **Gates glow; there are no red props or painted lines.** The race scene is dark gates with
  **cyan** edges (forward camera) and **white** edges (downward camera) over a blue wall and
  grey floor. The vision labs find gates by **brightness** / **cyan**, not red. The
  `neo_lab` helpers below do this for you.

---

## 4. The `neo_lab` helper (`labs/neo_lab.py`)

Shared helpers the simulator labs use. Import is set up for you at the top of each file
(the small `import neo_lab` boilerplate — you don't need to touch it).

```python
import neo_lab

# --- flight ---
launcher = neo_lab.Launcher(target_height=3.0)  # arm + climb to N m above ground
launcher.update(drone)        # call each frame; returns True once airborne & stable
neo_lab.height(drone)         # altitude in meters above the launch ground
neo_lab.world_position(drone) # true (x_east, y_up, z_north) m from the sim (no drift)

# --- gate vision ---
neo_lab.bright_mask(image, v_min=200)          # 0/255 mask of glowing gate edges
neo_lab.largest_gate(image, v_min, min_area)   # biggest SQUARE glowing gate (not a line)
neo_lab.largest_cyan_gate(image, min_area)     # biggest square CYAN gate (forward camera)
neo_lab.gate_nearest_center(image, ...)        # gate nearest the image center
neo_lab.gate_nearest_to(image, target_col, ...)# gate nearest a column (for tracking one gate)
neo_lab.CYAN_LOWER, neo_lab.CYAN_UPPER         # HSV bounds for the cyan gate edges
```

## The drone API (quick reference)

```python
import drone_core
import drone_utils as uav_utils
drone = drone_core.create_drone()

# Flight (all pcmd args in [-1, 1])
drone.flight.takeoff()
drone.flight.send_pcmd(pitch, roll, yaw, throttle)  # +pitch=fwd +roll=right +yaw=CW +throttle=up
drone.flight.stop()                                 # hover / hold position
drone.flight.land()

# Physics / sensors
drone.physics.get_altitude()           # meters (NOT zero on the ground; use neo_lab.height)
drone.physics.get_linear_velocity()    # (x=right, y=up, z=forward) m/s
drone.physics.get_attitude()           # (pitch, roll, yaw) degrees; yaw in [0,360)

# Cameras (numpy BGR arrays, 480x640x3)
drone.camera.get_color_image()         # forward camera
drone.camera.get_downward_image()      # downward camera

# Vision helpers (drone_utils)
uav_utils.find_contours(img, hsv_lower, hsv_upper)
uav_utils.get_largest_contour(contours, min_area)
uav_utils.get_contour_center(contour)  # (row, col) or None
uav_utils.get_contour_area(contour)
uav_utils.clamp(v, lo, hi)
uav_utils.remap_range(v, a, b, c, d)

# Frame loop
drone.set_start_update(start, update, update_slow)
drone.get_delta_time()                 # seconds since last frame
drone.go()
```

---

## 5. Contents

### Week 2 — Vision (`labs/week2_vision/`)

| Module | Type | Topic |
|--------|------|-------|
| `module1_image_formation`    | concept   | Pinhole camera model: projection, pixels, intrinsics, distortion |
| `module2_opencv`             | simulator | Thresholding & morphology on the live downward camera |
| `module3_linear_regression`  | simulator | Fit a line (least squares) to the ground line, then follow it |
| `module4_gate_markers`       | simulator | Detect a gate by its ArUco corner tags, then search and reach it |
| `module5_distance_estimation`| simulator | Range to a gate from a tag's apparent size (inverse of Module 1) |
| `module6_optical_flow`       | simulator | Estimate ground velocity from the downward camera |

### Week 3 — Controls (`labs/week3_controls/`)

| Module | Type | Topic |
|--------|------|-------|
| `module1_coordinate_frames` | concept   | Euler↔rotation, rotation→quaternion, ENU/NED, thrust sizing |
| `module2_feedback_control`  | concept + simulator | Proportional control → altitude hold & setpoints |
| `module3_pid`               | simulator | PID altitude, fly-a-distance, vision+PID visual-servo, and feedforward tracking of a moving reference |
| `module4_heading_hold`      | simulator | Yaw heading hold from the IMU, with angle wrap-around |

### Week 4 — Integration (`labs/week4_integration/`)

Multi-axis flight labs that build on Week 3 control.

| Module | Type | Topic |
|--------|------|-------|
| `module1_waypoint`  | simulator | Dead-reckon position and fly to a 3-axis waypoint |
| `module2_patterns`  | simulator | Sequence waypoints to fly a square |
| `module3_trajectory`| simulator | Velocity-commanded trajectory tracking: a timed segment, a cubic-spline waypoint course (drone racing), and orbiting a point |

Each module folder has its own `README.md` with the details.

---

## 6. Recording & plotting your flights

Reading numbers scroll past is a poor way to tell whether a controller is doing the right
thing. You can record a flight to a CSV and plot it.

Set the `NEO_RECORD` environment variable to a file path and run any simulator lab as
usual. Each frame's telemetry is appended — time, height, velocity, heading, and the
true world x/z position:

```bash
NEO_RECORD=run.csv drone sim course/week3_controls/module3_pid/main.py
```

Then plot it (writes `run.png` next to the CSV):

```bash
python3 labs/plot_log.py run.csv
```

Each channel is drawn against time, and if the log has `x`/`z` columns it adds a top-down
trajectory. This shows what the prints can't: a P-controller's steady-state droop vs. PID
settling, overshoot and oscillation, or whether your square is actually square.

To log your own extra channels (for example a gate's pixel width), call
`neo_lab.record(drone, gate_width=w)` from inside a step's `update`.

> A lab keeps running and recording after it lands, so stop it with `Ctrl-C` once it
> finishes — otherwise the CSV keeps growing with on-the-ground samples.

Each flying module has a reference plot of a correct solution run at
`<module>/solutions/reference_run.png` — record your own flight and compare against it.

---

## 7. Troubleshooting

| Symptom | Fix |
|---------|-----|
| Stuck at *"awaiting connection from UAVNeo Simulator"* | Launch the sim (`drone open_sim`). On native Linux/Mac it connects to `127.0.0.1`; if needed, force it with `DRONE_SIM_IP=127.0.0.1`. |
| Connected, but nothing happens | Click the **sim window** and press **`Enter`** to enter user-program mode. |
| Drone won't move / sits on the ground | It must arm and climb first — the lab's launcher handles this. If you wrote a controller, remember small commands (<~0.05) do nothing (deadband). |
| *"every drone already has a connected Python script"* | A previous run is still attached. Stop it (Ctrl-C in its terminal) before starting a new one. |
| `ModuleNotFoundError: neo_lab` | Run labs through `drone sim` (or from inside `labs/`) so the import path resolves. |
| Vision finds nothing | The gates glow cyan/white — detect by brightness, not red. Use the `neo_lab` gate helpers. |

---

GNU General Public License v3.0
