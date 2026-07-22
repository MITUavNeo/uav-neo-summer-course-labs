# Week 4 · Module 1 — Waypoint Navigation

Fly to a point in space, given as a distance right, up, and forward from the start.
Every earlier flying lab controlled one axis at a time; this one drives all three
together, which is the foundation for flying a pattern and running a gate course.

## What you'll learn

- **Dead reckoning on three axes** — estimating right/up/forward position from velocity.
- **A three-axis position controller** — commanding a body velocity on all three axes at once.
- **Velocity commands** — turning position error into a target speed the flight layer tracks, the same call in the sim and on the real drone.

## How it works

Every flying lab so far moved one axis at a time. A **waypoint** — a target point given as
`(right, up, forward)` from the start — forces all three at once, the leap from "hold a value"
to "go somewhere."

**Know where you are.** The sim has no position sensor, so you track position by **dead
reckoning**: integrate the velocity reading each frame, `position += velocity · dt`, on the
right and forward axes. The velocity already arrives in the **body frame** (x=right, y=up,
z=forward), so no rotation is needed. The estimate drifts as small errors pile up — which is
why the arrival tolerance is generous and why real drones correct it with a camera or GPS.

**Drive three axes together.** Each axis gets a target velocity: right, forward, and up. You
command them together with `neo_lab.send_velocity(drone, v_right, v_up, v_forward)` and the drone
moves diagonally toward the point. The same call runs on the real drone, where the flight
controller tracks the velocity; in the sim a shared inner loop turns it into tilt and throttle,
so the vertical and speed scaling lives in one place instead of being retuned in this lab.

**Let the shrinking target speed brake you.** Turn each position error into a target speed,
`v = KP · error`, and hand it to `send_velocity`. As you close in, the error shrinks, so the
commanded speed shrinks with it and you ease into the waypoint instead of blowing past it — the
velocity loop underneath supplies the damping a hand-tuned brake term used to. `send_velocity`
turns that speed into motion, applying the real drone's `REAL_MAX_SPEED` limit on hardware and a
matching tilt limit in the sim, so you don't cap it yourself. Hold height with
`neo_lab.altitude_hold_velocity`, which returns the vertical speed that keeps you at `TARGET_HEIGHT`.

Why it matters: "estimate position, command a velocity on all axes, ease in on arrival" is the
core of autonomous navigation. The next module chains waypoints into a path, and a gate course
is just a longer version of the same loop.

## Key terms

- **Waypoint** — a target point to fly to, here `(right, up, forward)` meters from where you started.
- **Dead reckoning** — estimating position by integrating velocity (`position += velocity · dt`) because the sim has no position sensor. It drifts over time, which is why the tolerance is generous.
- **Body frame** — directions relative to the drone: x = right, y = up, z = forward. The velocity reading is already in this frame.
- **Velocity command** — `neo_lab.send_velocity(drone, v_right, v_up, v_forward)`: you ask for a body-frame speed and the flight layer produces it (a velocity setpoint on the real drone, tilt and throttle in the sim). The same call works in both.
- **Proportional speed** — `v = KP · error`: the target speed shrinks as you approach, so you settle without a separate brake term. `send_velocity` applies the real-drone speed limit for you.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week4_integration/module1_waypoint/main.py            # all steps, your code
drone sim course/week4_integration/module1_waypoint/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start.

## Steps

1. **`step1_track_position.py`** — integrate velocity into a right/up/forward position estimate
2. **`step2_goto_waypoint.py`** — fly to a target waypoint and hold there

## What to expect

Step 1 nudges the drone diagonally and prints its estimated position. Step 2 flies
to the target point, brakes, holds, and lands.

## You're done when

- Step 1 prints a position with positive right and forward values after the nudge.
- Step 2 arrives within `POS_TOL` of the target on both horizontal axes, slows to a stop, reports arrival, and lands.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| Drone flies the wrong direction | Check axis signs: `v_right` drives right (x), `v_forward` drives forward (z); positive error means go that way. |
| Overshoots the target | Lower `KP_POS` for a gentler approach. The target speed must fall to zero as the error does. |
| Never finishes | Position drift may keep the error above `POS_TOL`; confirm the speed-and-position-and-hold logic, and that `POS_TOL` is generous. |
| Loses height while translating | Keep passing `v_up` from `altitude_hold_velocity` every frame, not only once stopped. |

## Going further (optional)

- The dead-reckoned position drifts. After arriving, how far off is the true position? How could the downward camera (Module 6 optical flow) correct it?
- Add a final yaw so the drone faces its direction of travel before moving.
- Chain two waypoints back to back — the next module turns this into a full pattern.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
