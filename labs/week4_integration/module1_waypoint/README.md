# Week 4 · Module 1 — Waypoint Navigation

Fly to a point in space, given as a distance right, up, and forward from the start.
Every earlier flying lab controlled one axis at a time; this one drives all three
together, which is the foundation for flying a pattern and running a gate course.

## What you'll learn

- **Dead reckoning on three axes** — estimating right/up/forward position from velocity.
- **A three-axis position controller** — combining roll, pitch, and throttle at once.
- **A velocity (D) term** — braking on approach so you arrive without overshooting.

## How it works

Every flying lab so far moved one axis at a time. A **waypoint** — a target point given as
`(right, up, forward)` from the start — forces all three at once, the leap from "hold a value"
to "go somewhere."

**Know where you are.** The sim has no position sensor, so you track position by **dead
reckoning**: integrate the velocity reading each frame, `position += velocity · dt`, on the
right and forward axes. The velocity already arrives in the **body frame** (x=right, y=up,
z=forward), so no rotation is needed. The estimate drifts as small errors pile up — which is
why the arrival tolerance is generous and why real drones correct it with a camera or GPS.

**Drive three axes together.** Each axis gets its own controller and its own actuator: roll for
the right error, pitch for the forward error, throttle for the height error. Run all three
every frame and the drone moves diagonally toward the point.

**Brake so you arrive, not overshoot.** Pure proportional control on position tends to build
speed and blow past the target. Adding a **derivative** term — `KP · error − KD · velocity` —
turns the velocity itself into a brake: the faster you move, the harder it pulls back, so you
ease into the waypoint instead of circling it. That is **PD control**, the P+D of PID without
the integral.

Why it matters: "estimate position, command all axes, brake on arrival" is the core of
autonomous navigation. The next module chains waypoints into a path, and a gate course is just
a longer version of the same loop.

## Key terms

- **Waypoint** — a target point to fly to, here `(right, up, forward)` meters from where you started.
- **Dead reckoning** — estimating position by integrating velocity (`position += velocity · dt`) because the sim has no position sensor. It drifts over time, which is why the tolerance is generous.
- **Body frame** — directions relative to the drone: x = right, y = up, z = forward. The velocity reading is already in this frame.
- **PD control** — proportional plus derivative: `KP · error − KD · velocity`. The derivative term acts as a brake so you settle instead of overshooting.

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
| Drone flies the wrong direction | Check axis signs: roll handles right (x), pitch handles forward (z); positive error means go that way. |
| Overshoots and circles the target | Raise `KD_POS`, or lower `KP_POS`. The velocity term must oppose motion. |
| Never finishes | Position drift may keep the error above `POS_TOL`; confirm the speed-and-position-and-hold logic, and that `POS_TOL` is generous. |
| Loses height while translating | Throttle must still run the altitude term while you roll and pitch. |

## Going further (optional)

- The dead-reckoned position drifts. After arriving, how far off is the true position? How could the downward camera (Module 6 optical flow) correct it?
- Add a final yaw so the drone faces its direction of travel before moving.
- Chain two waypoints back to back — the next module turns this into a full pattern.

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
