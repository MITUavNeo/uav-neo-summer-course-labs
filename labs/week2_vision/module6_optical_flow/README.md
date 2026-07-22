# Week 2 · Module 6 — Optical Flow

Estimate how fast the drone is moving by watching the ground slide past the
downward camera. This is how many real drones hold position indoors without GPS.

## What you'll learn

- **What optical flow is** — the apparent motion of the image between two frames.
- **Sparse feature tracking** — following a few corner points (Lucas-Kanade) instead of every pixel.
- **Flow → velocity** — converting pixel motion to a real ground speed using altitude and time.

## How it works

Indoors there is no GPS, but the downward camera still sees the ground slide past — and how
fast it slides is how fast the drone is moving. Optical flow turns that sliding into a
velocity.

**Flow is image motion.** **Optical flow** is the apparent `(dx, dy)` shift of image patterns
between two frames. Computing it for every pixel (**dense** flow) is expensive and lags the
sim, so this lab uses **sparse** flow: `goodFeaturesToTrack` picks a handful of distinctive
**corners**, and `calcOpticalFlowPyrLK` (Lucas-Kanade) finds where each moved next frame. The
average displacement of the surviving points is the flow.

**Flow encodes ground speed — but in pixels.** A larger **flow magnitude**
(`sqrt(dx² + dy²)`) means faster apparent motion. Getting to meters per second takes two
conversions. First, how much ground one pixel covers — the **footprint** — which grows with
altitude: `meters_per_pixel = 2 · height · tan(½ FOV) / image_width` (fly higher and each
pixel spans more ground). Second, divide by the time between processed frames to turn
pixels-per-frame into a rate. One sign detail: the camera moves *opposite* to the scene flow,
so the estimate is negated.

**Why the Optical Flow Sandbox.** Flow needs texture to track. A blank floor gives near-zero flow
even while moving, so run this in the textured **Optical Flow Sandbox** — flow ≈ 0 over bare
ground is the scene, not your code.

Why it matters: this is real **optical-flow velocity estimation**, the same sensor many drones
use to hold position indoors. It also drifts over time, which is the motivation for sensor
fusion — the theme that ties Week 2 sensing to Week 4 integration.

## Key terms

- **Optical flow** — the apparent motion of image patterns between two frames, as a `(dx, dy)` displacement.
- **Sparse flow** — track a small set of distinctive corner points frame-to-frame (Lucas-Kanade). Far cheaper than **dense** flow (a vector for every pixel), so it keeps the simulator real-time.
- **Feature / corner** — a locally distinctive pixel patch that is easy to find again next frame; `goodFeaturesToTrack` picks them.
- **Flow magnitude** — the length `sqrt(dx² + dy²)` of a point's displacement; larger means faster apparent motion.
- **Ground footprint / meters-per-pixel** — how much real ground one pixel covers. It grows with altitude: `2 · height · tan(½ FOV) / image_width`.
- **Frame rate (dt)** — seconds between frames. Pixels/frame ÷ dt gives pixels/second, which the footprint converts to meters/second.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module6_optical_flow/main.py            # all steps, your code
drone sim course/week2_vision/module6_optical_flow/main_solution.py   # reference flight
```

From the sim menu's **Drone Environments** section, load **Optical Flow Sandbox** (its textured
floor is what produces flow), then press **Enter** in the simulator window to start.

## Steps

1. **`step1_flow_magnitude.py`** — compute the average optical-flow magnitude while drifting
2. **`step2_velocity_estimate.py`** — convert flow to a velocity estimate and compare to the true velocity

## What to expect

The drone drifts gently forward so the ground moves under it. Step 1 prints a mean
flow magnitude; Step 2 prints an estimated velocity beside the drone's true velocity.

> **Run this lab in the Optical Flow Sandbox.** Optical flow tracks moving *features*, so a
> plain, untextured floor gives near-zero flow even while the drone moves. Select **Optical
> Flow Sandbox** from the sim menu's **Drone Environments** section — its textured floor
> produces clear flow. Flow ≈ 0 over bare ground is the scene, not your code.

## You're done when

- Step 1 prints a mean flow magnitude greater than 0 while drifting.
- Step 2 prints an estimated `(x, z)` velocity whose **sign and rough size** track the true velocity from `get_linear_velocity()`. The scale is approximate — exact match is not expected.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Flow is ~0 | Run in the **OpticalFlow** world (textured floor); a bare floor gives near-zero flow. Also confirm the drone is drifting (`PROBE_SPEED`). |
| First frame errors | Sparse flow needs a previous frame *and* points — find features and store `_prev_gray`/`_prev_pts` before tracking. |
| Lab lags the sim | Don't run dense flow per pixel; sparse tracking (this lab) stays real-time. Lower `maxCorners` if needed. |
| Estimate has the wrong sign | The camera moves opposite to the scene flow; negate the mean flow. |
| Estimate is way too big/small | Check `meters_per_pixel` uses the current `height` and that you divided by `dt`. |

## Going further (optional)

- Compare against **dense** flow (`cv2.calcOpticalFlowFarneback`) and measure how much slower it runs — that's why this lab tracks sparse features.
- Use the flow velocity to **hold position**: command roll/pitch to drive the estimated velocity to zero.
- Optical flow drifts over time. Combine it with the Module 5 distance estimate to bound the error (a tiny taste of sensor fusion).

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
