# Week 2 · Module 4 — Gate Markers (Seek a Gate)

Detect a gate by the ArUco tags on its corners, then search for, center, and approach one.

## What you'll learn

- **Why color fails here** — the gate's neon and the sky are the same blue, so hue can't separate them.
- **ArUco markers** — square black-and-white tags the drone can detect and identify by number.
- **From tags to a gate** — turning the four corner tags into one gate center and a distance cue.
- **Search then center then approach** — creeping in to find a gate, turning to face it, then flying to it.

## How it works

The obvious idea is to segment the gate by color. It doesn't work: the neon strips read as blue
(hue ~105) and so does the sky behind them (hue ~105), so a hue mask grabs both. The gates carry
a better signal — **ArUco markers**, one on each corner.

**Read the tags, not the color.** An **ArUco marker** is a square grid of black and white cells
encoding a number, built for exactly this: `cv2.aruco` finds each tag's four corners in the image
and decodes its id. These gates use the `DICT_6X6_250` dictionary, four tags per gate. Because
they encode an id, the drone also learns *which* gate it is looking at, not just that something is
there.

**Turn four tags into one gate.** `neo_lab.detect_gate(image)` runs the detector and returns a
`Gate`: `.cx`/`.cy` is the mean of the visible tag centers (the gate's image position), `.tag_px`
is the average tag size in pixels (it grows as you get closer — a distance cue), and `.count`/`.ids`
are how many tags decoded and their numbers. It returns `None` when no tag is readable.

**Search, center, approach.** A 6×6 tag only decodes once it is big enough (~30–40 px), so from
across the arena the tags are unreadable — you must **creep forward to find a gate**, not spin in
place. Once `detect_gate` returns a `Gate`, the gap between `.cx` and the image center is a **yaw**
error; turn to close it, and only add forward **pitch** once centered so you face the gate before
chasing it. Stop when the gate is centered *and* `.tag_px` is large enough that you have arrived.

Why it matters: real markers survive lighting and background that break color, and sequencing
search / aim / approach is the skeleton of flying a course of gates.

## Key terms

- **ArUco marker** — a square black-and-white grid that encodes a number; `cv2.aruco` detects its corners and decodes its id. Robust to lighting and viewing angle.
- **Dictionary** — the family a marker belongs to (here `DICT_6X6_250`: 6×6 cells, ids 0–249). The detector must use the same dictionary the tags were made with.
- **`neo_lab.detect_gate(image)`** — returns a `Gate` (`.cx`, `.cy`, `.span`, `.tag_px`, `.count`, `.ids`) or `None`. Center is exact with all four tags, approximate with fewer.
- **Tag size (`.tag_px`)** — the average marker side length in pixels; it grows as the drone nears the gate, so it stands in for distance.
- **Yaw** — rotating in place (turning left/right) without moving. Used to point the drone at the gate before flying toward it.
- **Approach control** — only adding forward speed once the gate is centered, so you drive toward it rather than past it.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week2_vision/module4_gate_markers/main.py            # all steps, your code
drone sim course/week2_vision/module4_gate_markers/main_solution.py   # reference flight
```

Load a gate scene (e.g. **LabD_GateNavigation**). The ArUco tags only decode up close, so from
the spawn point they are usually unreadable — **manually fly the drone up near a gate before you
start**, so the script has tags to work with. Then press **Enter** in the simulator window to
hand control to your script.

## Steps

1. **`step1_hsv_mask.py`** — creep forward until a gate's tags decode; report how many and their ids
2. **`step2_bounding_box.py`** — creep in, then report the gate's image center and span
3. **`step3_seek_gate.py`** — creep-and-scan to find a gate, center it with yaw, fly toward it

## What to expect

The drone climbs, then creeps forward until the corner tags decode; Steps 1–2 report and stop, Step 3 turns to center the gate and flies forward until the tags fill the view ('Reached the gate').

## You're done when

- Step 1 prints a nonzero tag count and a list of ids once it reaches a gate.
- Step 2 prints the gate's center `(cx, cy)` and a span in pixels.
- Step 3 centers the gate, flies forward until `tag_px` reaches `TARGET_TAG_PX`, prints that it reached the gate, and lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` | Capture the frame: `image = drone.camera.get_color_image()` (the **forward** camera here). |
| Always "no gate" | The tags only decode up close — make sure you creep forward (`SEARCH_PITCH`) instead of hovering, and start with a gate ahead. |
| Finds a gate but never centers | Reduce yaw as the error shrinks and check the sign of the `.cx - COL_CENTER` error. |
| Declares "reached" while off to the side | Require *both* centered (`< CENTER_TOL`) and close (`.tag_px >= TARGET_TAG_PX`) before finishing. |
| Aims between two gates | With two gates decoding at once, `detect_gate` averages all tags; approach one gate at a time for now. |

## Going further (optional)

- Steer by a specific tag id so you target one chosen gate instead of the average of all visible tags.
- Use `cv2.aruco.estimatePoseSingleMarkers` with the real tag size to get metric distance and heading, not just a pixel cue.
- Detect when you've passed *through* the gate (its tags leave the frame) and immediately search for the next one.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
