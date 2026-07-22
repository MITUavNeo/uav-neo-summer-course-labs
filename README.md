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
- **Gates are found by their ArUco corner tags, not by color.** A gate's neon strips read at
  the same hue as the sky, so color cannot separate them; each gate instead carries four
  `DICT_6X6_250` ArUco tags, one per corner. The tags only decode up close, so a gate search
  creeps forward rather than spinning in place. The downward line is recolored every run, so the
  line labs isolate it by HSV **saturation**, not a fixed color. The `neo_lab` helpers below do both.

---

## 4. Same code in the simulator and on the drone

You write each lab once and run the **same file** in the simulator and on the real drone; only
how you launch it differs.

**One file, one flag.** `drone sim <path>/main.py` runs the file in the simulator — it adds the
`-s` flag for you, and you press **Enter** in the sim window to start. On the drone you run the
identical file with plain `python3 main.py` (no `-s`), and the safety pilot arms and switches to
**OFFBOARD** to hand control to your program. `create_drone()` reads the `-s` flag and picks the
simulator or the real backend; nothing else in your file changes.

**Keeping your code in sync.** Edit on your laptop, then push it to the drone:

```bash
drone sync labs        # copy your labs/ to the drone
drone sync library     # copy your library/ to the drone
drone sync all         # both
```

`update.sh` (from the installer) pulls new course labs and library onto either machine. Edit
once, `drone sync`, run on either side.

**Before your first lab on the real drone** (and after any update), run the two pre-flight
checks from `labs/diagnostics/` on the drone:

```bash
python3 ~/jupyter_ws/<team>/labs/diagnostics/camera_check.py    # no flight: both cameras stream and the vision runs
python3 ~/jupyter_ws/<team>/labs/diagnostics/takeoff_check.py   # minimal flight: climb to 1 m and land (safety pilot arms + OFFBOARD)
```

If either fails, fix that first — every vision lab needs both cameras, and every flying lab
starts with the same launch these scripts exercise.

**What actually carries over.** Not every lab flies the same on hardware:

| Labs | Runs the same on the drone? |
|------|-----------------------------|
| `labs/flights/*`, `week4_integration/module3_trajectory` | **Yes.** They command a body velocity (`neo_lab.send_velocity`) or a position (`goto_position`), which map straight to real setpoints. |
| Week 2 vision, Week 3 controls | **No — concept labs.** They call `send_pcmd` directly, which is a *tilt* command in the sim but a *velocity* command on real, so their tuned numbers do not transfer as-is. |
| `week4_integration/module1_waypoint`, `module2_patterns` | **Not yet.** Their velocity commands are portable, but they track position by dead reckoning, which drifts on real until a position sensor (`get_position`) replaces it. |

**What silently differs on the real drone** (see `labs/flights/README.md` for the flight-day roles):

- `send_pcmd` is a tilt command in the sim, a velocity command on real — write portable
  controllers through `neo_lab.send_velocity`, which handles both.
- In the sim the `Launcher` arms and climbs for you; on real the **safety pilot** arms and
  enables OFFBOARD. Set `NEO_NO_LAUNCH=1` (or pass `launch=False`) to skip the climb and run from
  wherever a hand-flown drone already is.
- `goto_position` speed is set by PX4 parameters on real, not the library — lower the `MPC_*`
  limits for an indoor space.
- The real cameras (RealSense forward, Arducam nadir) need calibration; the sim cameras are ideal.

---

## 5. The `neo_lab` helper (`labs/neo_lab.py`)

Shared helpers the simulator labs use. Import is set up for you at the top of each file
(the small `import neo_lab` boilerplate — you don't need to touch it).

```python
import neo_lab

# --- flight ---
launcher = neo_lab.Launcher(target_height=3.0)  # arm + climb to N m above ground
launcher.update(drone)        # call each frame; returns True once airborne & stable
neo_lab.height(drone)         # altitude in meters above the launch ground
neo_lab.world_position(drone) # true (x_east, y_up, z_north) m (needs the position-endpoint build)
neo_lab.send_velocity(drone, v_right, v_up, v_forward)  # body-velocity command, same in sim and on real
neo_lab.altitude_hold_velocity(drone, target_height)    # vertical speed (m/s) that holds a height

# --- vision ---
neo_lab.detect_gate(image)              # locate a gate by its ArUco corner tags -> Gate(cx, cy, span, tag_px, ids) or None
neo_lab.saturated_mask(image, s_min=100)# 0/255 mask of the vivid (recolored) ground line, by HSV saturation
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

## 6. Contents

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

## 7. Recording & plotting your flights

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

## 8. Troubleshooting

| Symptom | Fix |
|---------|-----|
| Stuck at *"awaiting connection from UAVNeo Simulator"* | Launch the sim (`drone open_sim`). On native Linux/Mac it connects to `127.0.0.1`; if needed, force it with `DRONE_SIM_IP=127.0.0.1`. |
| Connected, but nothing happens | Click the **sim window** and press **`Enter`** to enter user-program mode. |
| Drone won't move / sits on the ground | It must arm and climb first — the lab's launcher handles this. If you wrote a controller, remember small commands (<~0.05) do nothing (deadband). |
| *"every drone already has a connected Python script"* | A previous run is still attached. Stop it (Ctrl-C in its terminal) before starting a new one. |
| `ModuleNotFoundError: neo_lab` | Run labs through `drone sim` (or from inside `labs/`) so the import path resolves. |
| Vision finds nothing | Gates are located by their ArUco corner tags (`neo_lab.detect_gate`); the ground line by saturation (`neo_lab.saturated_mask`). Tags only decode up close, so creep toward a gate rather than spinning in place. |

---

GNU General Public License v3.0
