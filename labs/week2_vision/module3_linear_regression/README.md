# Week 2 · Module 3 — Linear Regression (Line Following)

Fit a straight line to detected pixels with least squares, then use it to steer. A colored
line runs along the ground; the drone follows it with the downward camera.

## What you'll learn

- **Finding the line pixels** — isolating a colored line by saturation and reading its coordinates with `np.argwhere`.
- **Least-squares line fitting** — the single best-fit straight line through a cloud of points.
- **Pixel offset → steering** — turning "how far off-center is the line" into a roll command.

## How it works

A colored line runs along the floor, and the drone follows it like a lane. That takes three
moves: see the line, describe it with a line fit, and steer to stay on it.

**See the line.** The line is recolored every run (red one time, green the next), so you cannot
key on a fixed hue. What is constant is that the line is **vivid** and the floor is grey, so
threshold by **saturation**: `neo_lab.saturated_mask` keeps the colorful pixels regardless of
which color the run picked. `np.argwhere` then hands back the `(row, col)` of every line pixel —
a scatter of points tracing the line. (Watch the convention: rows are the y-axis, columns are
the x-axis.)

**Describe it with a line.** A cloud of line pixels is noisy, so summarize it with the single
straight line `y = m·x + b` that fits best. "Best" has a precise meaning: **least squares**
picks the line that minimizes the total squared vertical distance from the points to the
line, a balance no single outlier can hijack. `np.polyfit` returns the slope `m` and
intercept `b`.

**Steer to stay on it.** Split the line pixels into a **far** part (top of the view, where the
drone is heading) and a **near** part (under the drone). The far part's offset from center
drives a proportional **yaw** — turning to face along the line, which is what carries the drone
through curves; the near part's offset drives a gentle **roll** to stay laterally centered. That
proportional idea (offset → command) is the same one you will use for altitude in Week 3,
applied to a pixel error instead of a height error.

Why it matters: fitting a model to noisy measurements and acting on it is the core loop of
robot perception. Here it is a line and a roll command; the same pattern reappears whenever
the drone must turn many raw pixels into one clean decision.

## Key terms

- **Linear regression** — finding the straight line that best fits a cloud of points.
- **Least squares** — the rule that picks "best": the line that minimizes the sum of squared vertical distances from the points. `np.polyfit(xs, ys, 1)` does it for you and returns slope `m` and intercept `b`.
- **Saturation mask** — keeping the vivid (high-saturation) pixels. `neo_lab.saturated_mask(image, S_MIN)` isolates a colored line from a grey floor for *any* line color.
- **`np.argwhere(mask)`** — returns the `(row, col)` coordinates of every nonzero pixel in a mask. Rows are the y-axis, columns are the x-axis.
- **Pixel offset** — how far the detected line is from the center column of the image, in pixels. Positive means the line is to the right.
- **Roll** — tilting left/right, which makes the drone strafe sideways. Used here to slide back over the line.
- **Proportional steering** — making the correction proportional to the offset: far off → strong correction, nearly centered → gentle.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module3_linear_regression/main.py            # all steps, your code
drone sim course/week2_vision/module3_linear_regression/main_solution.py   # reference flight
```

Load a line scene (e.g. **LabC_LineFollower**) with a line ahead, then press **Enter** in the
simulator window to start.

## Steps

1. **`step1_detect_line.py`** — find and count the colored line pixels
2. **`step2_fit_line.py`** — fit y = m·x + b to those pixels
3. **`step3_follow_line.py`** — fly forward, yawing to turn along the line and rolling to stay centered

## What to expect

Steps 1-2 fly forward off the spawn pad, then hover and report; Step 3 flies forward, turning to follow the line until it ends.

## You're done when

- Step 1 prints a line-pixel count in the thousands while a line is in view.
- Step 2 prints a slope `m` and intercept `b` (any finite numbers — the exact values depend on what the camera sees).
- Step 3 flies forward, turning to follow the line (with some wobble on sharp curves), and lands when the line ends.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Capture the frame: `image = drone.camera.get_downward_image()`. |
| 0 line pixels | The drone is not over the line yet, or `S_MIN` is too high — lower it or fly until the line is below you. |
| `np.polyfit` raises or returns `nan` | You passed too few points, or columns/rows are swapped. Convert to float and use column as x, row as y. |
| Drone rolls the wrong way and loses the line | Flip the sign of your roll command (camera/strafe direction may be inverted). |
| Step 3 over- or under-turns on curves | Tune `MAX_YAW` (higher turns harder through curves, lower reduces wobble) and `FORWARD_PITCH` (slower gives the turn more time). |

## Going further (optional)

- A nearly-vertical line makes `y = m·x + b` blow up (slope → ∞). Detect that case and fit `x = m·y + b` instead.
- Feed the yaw/roll steering through a PID (Week 3) instead of a plain proportional gain, to damp the wobble on sharp curves.
- Reject outliers: refit after dropping points far from the first fit (a one-step RANSAC).

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
