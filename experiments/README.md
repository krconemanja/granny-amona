# Experiments

A more ambitious version of the mission, based on the course lectures, that we
started before scaling back to the simpler go-to-goal mission in the main
folder. It was not used in the final runs on the lake.

This folder is self-contained: it has its own [`lib/`](lib/) and only shares
the hardware drivers ([`../drivers-ddboat-v2/`](../drivers-ddboat-v2/)) with
the final project.

**Status: does not run as is.** `twin_mission.py` expects
`MAX_PWM` in `lib/boat_io.py`, and `next_waypoint()` and
`control(..., last=...)` in `lib/control.py`, which are missing.

## What it does

- **Multi-waypoint route**: the mission is a path drawn in Google Earth
  ("Add Path", saved as `.kml`), read by `load_kml()` and `build_route()` in
  [`lib/geo.py`](lib/geo.py). Path points within 10 m of home are dropped
  so the boat never drives into the dock, and home is always the last
  waypoint.
- **Digital twin**: the tank model of the boat (`Twin` in
  [`lib/model.py`](lib/model.py)) runs next to the real boat. Every loop
  it predicts the motion from the motor commands, then is pulled towards the
  GPS fix and the compass heading. The controller uses the twin's position,
  which is smoother than the raw GPS (one fix per second, 2–3 m of noise).
- **Log analysis**: every run is logged to CSV. `plot_log.py` plots the GPS
  track against the twin and the route, exports the track to Google Earth,
  and measures the model constants from manual runs.

## Files

| File | Purpose |
| --- | --- |
| `twin_mission.py` | Main loop: laptop commands, sensors, twin, control, motors, log |
| `plot_log.py` | Plot a log, export it to `.kml`, measure `K_SPEED` and `K_TURN` |
| `missions/sim_square.kml` | Example route for the simulation |
| `lib/model.py` | Tank model and the `Twin` |
| `lib/geo.py` | GPS ↔ local metres, Google Earth `.kml` routes |
| `lib/control.py` | Proportional heading controller towards a waypoint |
| `lib/boat_io.py` | Real boat and simulated boat, same interface |
| `lib/imu.py`, `lib/calibration.py` | Compass heading and its calibration |

## Tuning the model on the water

The plan for identifying the model constants in
[`lib/model.py`](lib/model.py) with `plot_log.py`:

1. In MANUAL, at a constant command, about 20 s each: straight out (W), spin
   in place until facing back, straight back.
2. `plot_log.py <log>` prints `K_SPEED` (use the mean of the two straight
   runs, so the water current cancels out) and `K_TURN`.
3. Then tune the controller (`SPEED`, `K`, `RADIUS` in `lib/control.py`) and the
   twin (`K_POS`, `K_HEAD`: lower is smoother but slower to follow the GPS).
