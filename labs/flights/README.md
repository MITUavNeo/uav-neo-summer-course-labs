# Flight labs

Three short autonomous flights that run with the **same code** in the simulator and on
the real drone. They show how the drone_core library carries sim-developed code onto
real PX4 hardware.

| Flight | What it does | How the motion is produced |
|---|---|---|
| `waypoint_flight` | takeoff, fly to a point offset from takeoff, hold, land | `drone.flight.goto_position(...)` - PX4's position controller on real, a hidden controller in sim |
| `maneuver_flight` | takeoff, fly a body-velocity maneuver, land | `neo_lab.send_velocity(...)` - a velocity setpoint on real, velocity-to-tilt in sim |
| `shape_flight` | takeoff, trace a repeating shape, land | open-loop patterns via `send_velocity`, or waypoint polygons via `goto_position`; pick shape/mode in `shape_flight/step_shape.py` |

Each flight is a folder with a `main.py` (the entry point) and its step module(s). The
`shape_flight` is the portable counterpart to `uav_neo_ros2_driver/shape_node.py`, which
draws the same shapes by talking to MAVROS directly (a ROS 2 learning example, real only).

## Run in the simulator

```bash
drone open_sim                                       # launch the sim, press ENTER in its window
drone sim course/flights/maneuver_flight/main.py     # then run a flight
drone sim course/flights/shape_flight/main.py
```

Note: `waypoint_flight` and the closed-loop shapes call `get_position()`, which needs the
**position-endpoint** simulator build. The maneuver and the open-loop shapes work on any
build.

## Run on the real drone

The autonomy stack (MAVROS + relays, no manual-teleop mux) starts on boot, so you only
run the flight. From a shell on the drone:

```bash
python3 ~/jupyter_ws/labs/flights/maneuver_flight/main.py
```

The flight **starts on its own** - there is no button to press. It streams setpoints
immediately, but nothing moves until the safety pilot enables it (below). Stop the program
with `Ctrl-C`.

### Who does what during a real flight

1. **Safety pilot (RC transmitter):** arms the drone and switches to **OFFBOARD** to hand
   control to the program. Switching out of OFFBOARD takes control back at any time - this
   is the abort, and the only safety layer, so a pilot must always be ready.
2. **The program:** climbs, runs the flight, then lands (`waypoint`/`shape` command PX4
   `AUTO.LAND`; `maneuver` descends). The LED strip **blinks while flying** on the maneuver
   and waypoint flights, and holds a **solid** color during the shape (for a long exposure).

## Before flying the waypoint or a closed-loop shape: lower the PX4 speed

Those use PX4's position controller, so their speed is set by PX4 parameters, not the
library's 0.5 m/s cap. Set these in **QGroundControl** before flying (defaults are far too
fast for an indoor space):

- `MPC_XY_CRUISE` = 0.6, `MPC_XY_VEL_MAX` = 1.0
- `MPC_Z_VEL_MAX_UP` = 0.6, `MPC_Z_VEL_MAX_DN` = 0.6

The maneuver and open-loop shapes are capped by the library, so they don't depend on this.

## Sizing for your space

Defaults target a ~3 x 6 m (10 x 20 ft) area. The waypoint offset and shape sizes are
constants at the top of each step module (`step_goto_waypoint.py`, `shapes.py`,
`step_shape.py`) - shrink them for a smaller space.
