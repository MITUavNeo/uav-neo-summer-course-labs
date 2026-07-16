# Week 2 · Module 1 — Image Formation

The pinhole camera model: how a 3-D point in the world becomes a pixel. This is pure math — no simulator needed.

## What you'll learn

- **Perspective projection** — how dividing by depth (`/Z`) makes distant things look smaller.
- **Meters → pixels** — converting a position on the sensor into a pixel coordinate.
- **The intrinsic matrix K** — packaging focal length and principal point into one 3×3 matrix.
- **Projecting a world point** — combining the camera's pose (R, t) with K to find a pixel.
- **Radial (lens) distortion** — how real lenses bend straight lines near the frame edges.

## How it works

A camera turns a 3-D world into a 2-D grid of pixels. The **pinhole model** is the simple
rule for that: imagine every ray of light passing through one tiny hole and landing on a
flat sensor behind it. Two ideas fall out of that picture.

**Things shrink with distance.** A point at camera-frame position `(X, Y, Z)` lands on the
sensor at `x = f·X/Z`, `y = f·Y/Z`. The `/Z` is the whole reason perspective works: double
the distance, halve the apparent size. `f`, the **focal length**, is how far the sensor
sits behind the pinhole — a longer `f` zooms in.

**Sensor meters become pixels.** The projection gives a position in meters on the sensor,
but a pixel grid needs integers. Divide by the physical width of one pixel and shift by the
**principal point** `(cx, cy)` — the pixel where the optical axis hits the sensor — to get
`(u, v)`. Bundling focal length and principal point into a 3×3 matrix gives the **intrinsic
matrix K**, so a single `K @ point` does the camera → pixel step.

**World points need the camera's pose.** A point given in *world* coordinates must first be
expressed in the *camera's* frame: rotate and translate it by the pose `(R, t)`
(`p_cam = R·p_world + t`), then apply `K`. The result is homogeneous, so you divide by the
third coordinate at the end to land on the pixel.

**Real lenses aren't perfect pinholes.** They bend straight lines outward or inward near the
edges (**radial distortion**), which the `k1, k2` coefficients model as a stretch that grows
with distance from the image center. Robotics code usually *undistorts* an image before
measuring anything in it.

Why it matters: every later vision lab — following an edge, centering on a gate, estimating
range — depends on this map between pixels and the world. Module 5 literally runs it backward
to recover distance from a gate's pixel width.

## Key terms

- **Pinhole camera model** — the idealized camera where every light ray passes through a single point. It makes far-away things look smaller, which is the `/Z` in the projection equations.
- **Focal length (f)** — the distance from the pinhole to the image plane. Larger f zooms in.
- **Image plane** — the flat surface (the sensor) where the image forms, measured in meters before you convert to pixels.
- **Principal point (cx, cy)** — the pixel where the camera's optical axis hits the sensor, usually near the image center.
- **Intrinsic matrix (K)** — a 3×3 matrix holding focal length and principal point. It converts a 3-D point in camera coordinates into pixel coordinates.
- **Camera pose (R, t)** — the camera's orientation (rotation R) and location (translation t) in the world. You need it to project a *world* point.
- **Radial distortion** — real lenses bend straight lines near the edges of the frame (barrel/pincushion). The k1, k2 coefficients model how much.

## How to run

```bash
python3 tasks/image_formation.py        # your work (prints PASS/FAIL)
python3 solutions/image_formation.py    # reference
```

## You're done when

The script prints `7/7 checks passed.` — every `Q*` line shows `[PASS]`. A `[FAIL]` line prints the value your function returned next to it, so you can compare against what the formula in the docstring should produce.

## If it doesn't work

| Symptom | Likely cause |
|---------|--------------|
| `Q1`/`Q2` fail | Check you divided by `Z` (Q1) and divided by `pixel_size` before adding the principal point (Q2). |
| `Q3` fails | `K` must be a real `np.array`, not `np.eye(3)`. Put `fx, fy, cx, cy` in the right cells. |
| `Q4` fails | Apply `R @ point + t` first (world → camera), then `K @ ...`, then divide by the third element. |
| `Q5` fails | `factor = 1 + k1*r² + k2*r⁴`, and `r² = x² + y²`. |

## Going further (optional)

- The labs run at 640×480. Given a horizontal field of view of 90°, what `fx` does that imply? Plug it into `intrinsic_matrix` and project a point 5 m away.
- Invert the process: given a pixel and a known depth `Z`, recover the 3-D camera-frame point. (This is what the downward-camera labs do implicitly.)
- Distortion is usually *removed* before processing. Look up `cv2.undistort` and describe what it needs.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
