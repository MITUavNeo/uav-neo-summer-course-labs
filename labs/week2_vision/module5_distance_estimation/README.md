# Week 2 · Module 5 — Distance Estimation

Use a gate tag's apparent size to estimate how far away it is, then approach it. This
connects the pinhole math from Module 1 to live flight: a marker of known real-world
size projects to a pixel size that shrinks with distance, so distance is recoverable.

## What you'll learn

- **Inverting perspective projection** — recovering distance from how big something looks.
- **Monocular range from a known size** — why one camera plus a known marker size gives distance.
- **Distance-driven approach** — using the estimate to decide when to stop.

## How it works

Module 1 projected a known 3-D size *down* to a pixel size. This module runs that arrow
*backward*: if you already know how big the gate's ArUco tag really is, its pixel size tells
you how far away it is. (The tag is the reference here because, as Module 4 showed, the gate's
color matches the sky and its tags are what the drone can reliably measure.)

**Size shrinks with distance.** From the pinhole model, a tag of real side `S` at distance `d`
projects to a pixel size `p = FOCAL_PX · S / d`. Every term but `d` is known: `S` is the tag's
true side length (`REAL_TAG_SIZE`), `FOCAL_PX` is the camera's focal length in pixels, and `p`
is `gate.tag_px` from the detector. Solve for the unknown and you get **monocular range**:
`d = FOCAL_PX · S / p`. One camera, no stereo, no depth sensor — just a known size.

**Where FOCAL_PX comes from.** It is the focal length expressed in pixels, set by the image
width and the **field of view**: for a 640-pixel image with a ~90° horizontal FOV it is about
320 (half the width divided by `tan(½ FOV)`). A wider FOV means a smaller `FOCAL_PX`.

**Fly by the estimate.** Each frame, read `gate.tag_px`, compute the distance, and act on it:
keep the gate centered with yaw, add forward pitch to approach, and stop once the distance
drops to `STOP_DIST`. The estimate is only as good as your `S` and `FOCAL_PX`, so a wrong
constant shows up as a wrong stopping distance.

Why it matters: recovering 3-D information (range) from a 2-D image with a known reference is
a workhorse trick in robotics — the same idea behind sizing an AprilTag or a person from a
single camera.

## Key terms

- **Apparent (pixel) size** — how big the tag looks in the image, in pixels (`gate.tag_px`). It grows as you get closer.
- **Focal length in pixels (FOCAL_PX)** — the camera's focal length expressed in pixels. For a 640-pixel-wide image with a ~90° horizontal field of view, it is about 320. (Half the width divided by tan(half the FOV).)
- **Monocular range estimation** — estimating distance from a single camera using a known object size: `distance = FOCAL_PX · real_size / pixel_size`.
- **Field of view (FOV)** — the angular width the camera sees. Wider FOV → smaller focal length in pixels.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module5_distance_estimation/main.py            # all steps, your code
drone sim course/week2_vision/module5_distance_estimation/main_solution.py   # reference flight
```

Load a gate scene (e.g. **LabD_GateNavigation**). The ArUco tags only decode up close, so from
the spawn point they are usually unreadable — **manually fly the drone up near a gate before you
start**, so the script has a tag to measure. Then press **Enter** in the simulator window to
hand control to your script.

## Steps

1. **`step1_measure_width.py`** — creep in until a gate is seen, measure its tag's apparent size in pixels
2. **`step2_estimate_range.py`** — convert tag size to distance and fly forward until close

## What to expect

Step 1 creeps forward until the tags decode, then prints a pixel size. Step 2 flies toward the gate, keeping it centered, and stops once the estimated distance reaches `STOP_DIST`, then lands.

## You're done when

- Step 1 prints a tag size of tens-to-low-hundreds of pixels.
- Step 2 flies forward, prints a shrinking distance, and stops at about `STOP_DIST` meters.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Capture the frame: `image = drone.camera.get_color_image()`. |
| Always "no gate" | Tags only decode up close — creep forward (`SEARCH_PITCH`) instead of hovering, and start with a gate ahead. |
| Distance is wildly wrong | Check `REAL_TAG_SIZE` matches the sim's corner tag, and that `FOCAL_PX` is in pixels (not meters or degrees). |
| Stops too early/late | The estimate is only as good as `REAL_TAG_SIZE` and `FOCAL_PX` — calibrate them (see below). |
| Drone flies past the gate | Make sure you stop on `distance <= STOP_DIST`, and yaw to keep the gate centered while approaching. |

## Going further (optional)

- Calibrate `FOCAL_PX` empirically: hover a known distance from a gate, read `gate.tag_px`, and solve `FOCAL_PX = tag_px · distance / real_size`.
- Use `cv2.aruco.estimatePoseSingleMarkers` with the real tag size and a camera matrix for a full 3-D pose (distance *and* bearing), not just range.
- Combine with Module 3's PID: feed the estimated distance error into a PID controller for a smooth stop instead of a hard cutoff.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
