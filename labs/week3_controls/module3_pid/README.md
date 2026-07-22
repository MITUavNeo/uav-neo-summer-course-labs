# Week 3 · Module 3 — PID Control

Full PID control, and a capstone that combines Week 2 vision with Week 3 control.

In Module 2 you held altitude with a proportional controller and watched it settle a
little short of the target — a permanent gap that never closed. This module fixes that
gap and adds damping, so the drone reaches a setpoint quickly *and* sits on it exactly.
The same three-term controller (PID) is the workhorse behind nearly every altitude,
position, and heading loop on a real drone.

## What you'll learn

- **The PID law (P + I + D)** — how three simple terms combine into one controller that is
  fast, accurate, and stable.
- **Why the integral term removes steady-state error** — the reason a P-only controller
  droops, and how the I term erases the gap.
- **What the derivative term does** — how reacting to the *rate* of error damps overshoot.
- **Anti-windup** — why the integral has to be clamped, and what goes wrong if it isn't.
- **Dead reckoning** — estimating how far you have travelled by integrating velocity when
  there is no position sensor.
- **Visual servoing** — closing a PID loop on a *camera pixel* error instead of a physical
  distance, so the drone steers using what it sees.
- **Feedforward** — tracking a target that is *moving*: commanding its known velocity
  directly so the controller leads the target instead of always lagging behind it.

## How PID control works

A feedback controller has one job: look at the **error** (how far you are from the target)
and decide what command to send. PID builds that command from three terms, each answering
a different question about the error.

**Proportional (P) — "how far off am I right now?"**
The P term is just `Kp · error`: a big error gives a big correction, and the push shrinks
as you close in. On its own, P has a flaw. To counteract a constant pull — gravity acting
on the throttle, for instance — it needs to output a non-zero command, but it only
*produces* one when there is a non-zero error. So it parks a little short of the target
forever. That leftover gap is the **steady-state error**.

**Integral (I) — "how long have I been off?"**
The I term sums the error over time: `Ki · Σ(error · dt)`. Even a tiny, constant error
keeps accumulating, so the integral grows and pushes harder until the error is driven to
*zero* — erasing the gap P leaves behind. The hazard: if the drone stays far from target
for a while, the integral can balloon and then violently overshoot once it catches up.
**Anti-windup** clamps the accumulated integral to a fixed range (`INT_CLAMP`) so it can
never wind that far up.

**Derivative (D) — "how fast is the error changing?"**
The D term is `Kd · d(error)/dt`. If the error is collapsing quickly you are about to
overshoot, so D pushes back to slow the approach — a shock absorber that trades a little
speed for much less overshoot and oscillation. D amplifies noise, so it is usually the
smallest gain.

Together: `output = Kp·error + Ki·∫error dt + Kd·d(error)/dt`. Tuning is the art of
balancing them: raise `Kp` for a faster response, add `Ki` to kill the steady-state gap,
add `Kd` to tame the overshoot that more `Kp` and `Ki` create.

**Dead reckoning (Step 2).** The sim reports velocity but not forward position, so you
estimate position by *integrating* velocity — `position += velocity · dt` every frame,
the same idea as the integral term. It works, but small velocity errors pile up, so the
estimate slowly drifts. That drift is a real limitation of navigating without a position
fix, and it is why real drones fuse in GPS or a camera.

**Visual servoing (Step 3).** The error need not be a distance in meters. Here it is a
*pixel* offset: how far the gate's center sits from the middle of the image. Feed that
pixel error through the same PID to command yaw, and the drone turns until the gate is
centered. This is the bridge between Week 2 (finding things in images) and Week 3
(controlling the drone): vision produces the error, the controller acts on it.

**Tracking a moving target: feedforward (Step 4).** Every step so far chased a *fixed*
setpoint. Step 4 gives the drone a target that keeps moving — a reference height `r(t)`
that rises and falls in time. A pure PID reacts only *after* an error appears, so against a
moving target it is always a step behind: it lags. The fix is **feedforward**. You already
know how fast the target is moving (`r_dot`, the reference's velocity), so command that speed
directly and let PID correct only the small leftover error. The output becomes
`feedback (PID on error) + feedforward (the target's own velocity)`. Because throttle here is
a vertical-*velocity* command, the feedforward is just `r_dot` scaled into throttle units
(`KFF`). Feedforward is what separates *following a setpoint* from *tracking a trajectory* —
the exact skill Week 4 builds on to fly a smooth path through a gate course.

**How this scales to a real quadrotor.** On a full drone controller (like the geometric
controller in MIT's VNAV course), the same split appears with different names: a **position
gain** pulls the drone toward where it should be, a **velocity gain** damps how fast it gets
there, and the desired trajectory's own velocity and acceleration are fed forward. Tuning is
still "raise the position gain for a snappier response, raise the velocity gain to stop it
overshooting" — the intuition you build here on one axis is the same one that stabilizes all
six degrees of freedom.

## Key terms

- **PID control** — a controller that sums three terms: `output = Kp·error + Ki·(integral of error) + Kd·(rate of change of error)`.
- **Integral term (I)** — adds up error over time. A small constant error builds the integral until the controller pushes hard enough to erase it — this is what removes the steady-state error P alone leaves.
- **Derivative term (D)** — responds to how fast the error is changing; it acts as a brake to reduce overshoot and oscillation.
- **Anti-windup** — clamping the accumulated integral so it can't grow huge while the drone is far from target (which would cause a big overshoot later). Here `INT_CLAMP` bounds it.
- **Dead reckoning** — estimating position by integrating velocity over time (`position += velocity · dt`) when you have no direct position sensor.
- **Visual servoing** — closing a control loop on a camera pixel error (here, yaw until the gate's column equals the image center).
- **Normalized error** — a pixel error divided by half the image width, so it lands in roughly −1…+1 regardless of resolution.
- **Feedforward** — adding the target's own known velocity to the command so the controller leads a moving reference instead of lagging it. Output = `PID(error) + KFF·(reference velocity)`.
- **Trajectory** — a target given as a function of time `r(t)` (with its velocity `r_dot`), rather than a single fixed setpoint. Tracking one is the job of Week 4.

## How to run

```bash
drone open_sim                          # launch the sim once
drone sim course/week3_controls/module3_pid/main.py            # all steps, your code
drone sim course/week3_controls/module3_pid/main_solution.py   # reference flight
```

Press **Enter** in the simulator window to start. **Step 3 needs a gate scene** (e.g.
**LabD_GateNavigation**) with a gate ahead: the ArUco tags decode only up close, so Step 3 creeps
forward while scanning until the tags read, then yaws to center the gate — or lands after
`SEARCH_TIMEOUT` if it never finds one (a gate off to the side or absent will time out).

## Steps

1. **`step1_pid_altitude.py`** — hold a target height with a full PID controller
2. **`step2_position_hold.py`** — fly a set distance forward (PID on integrated position)
3. **`step3_visual_servo.py`** — yaw with a PID loop to lock onto a gate
4. **`step4_track_reference.py`** — follow a height that keeps moving, using PID + feedforward

## What to expect

Runs the four steps in order: hold 5 m, fly forward, turn to lock onto a gate, then ride a
rising-and-falling height reference, and land.

## You're done when

- Step 1: the drone settles to `TARGET_HEIGHT` with **no** lasting gap (tighter than the P-only Module 2) and holds for `HOLD_TIME`.
- Step 2: the drone flies forward about `TARGET_DIST` meters and brakes to a stop instead of overshooting.
- Step 3: the drone turns until a gate is centered (within `CENTER_TOL`), holds, then lands.
- Step 4: the drone rides the moving reference up and down for `DURATION` seconds and reports a small **max error** — with the feedforward term it stays on the target instead of trailing it.

## If it doesn't work

| Symptom | Fix |
|---------|-----|
| `NameError: name 'image' is not defined` (Step 3) | Capture the frame: `image = drone.camera.get_color_image()`. |
| Altitude oscillates and grows | Integral windup — make sure you clamp `_err_int` to `±INT_CLAMP`, and add some `Kd`. |
| Step 2 overshoots the distance | Use velocity as the derivative term (`err_dot = -velocity[2]`) so it brakes early; raise `KD`. |
| Step 3 never sees a gate | ArUco tags decode only up close — start with a gate ahead so the forward-creep search can reach it; a gate off to the side or absent will time out and land. |
| Step never finishes | Check your "settled" timer logic — it must require staying within tolerance for the full hold time. |
| Step 4 always trails the target (large max error) | You left out the feedforward — add `KFF * r_dot` to the throttle so you command the target's speed, not just react to error. |

## Going further (optional)

- Tune Step 1 for the fastest settle with **no** overshoot. Record the `Kp, Ki, Kd` you land on.
- Step 2 dead-reckons distance from velocity, which drifts. How far off is it after 4 m? Could the downward camera correct the drift?
- Combine Steps 2 and 3: visual-servo the yaw *while* flying forward, so the drone both aims at and approaches the gate.
- Step 4: run it once with the feedforward term removed and record the max error, then add it back. How much of the lag was the feedforward removing? Try doubling `PERIOD` (a slower target) — does the lag shrink?

---

Fill in the blanks in `tasks/`; completed references are in `solutions/` (try it yourself first!).
