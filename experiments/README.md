# Experiments

A more ambitious version of the mission, based on the course lectures, that we
started before scaling back to the simpler go-to-goal mission in the main
folder. It was not used in the final runs on the lake.

**Status: archived, does not run as is.** It was written against an earlier
version of [`lib/`](../lib/): the current `lib/boat_io.py` has no `MAX_PWM`,
and `control.py` here has no `next_waypoint()` or `control(..., last=...)`.

## What it does

- **Multi-waypoint route**: the mission is a path drawn in Google Earth
  ("Add Path", saved as `.kml`), read by `load_kml()` and `build_route()` in
  [`lib/geo.py`](../lib/geo.py). Path points within 10 m of home are dropped
  so the boat never drives into the dock, and home is always the last
  waypoint.
- **Digital twin**: the tank model of the boat (`Twin` in
  [`lib/model.py`](../lib/model.py)) runs next to the real boat. Every loop
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
| `control.py` | Proportional heading controller towards a waypoint |
| `plot_log.py` | Plot a log, export it to `.kml`, measure `K_SPEED` and `K_TURN` |
| `missions/sim_square.kml` | Example route for the simulation |

## Tuning the model on the water

The plan for identifying the model constants in
[`lib/model.py`](../lib/model.py) and `plot_log.py`:

1. In MANUAL, at a constant command, about 20 s each: straight out (W), spin
   in place until facing back, straight back.
2. `plot_log.py <log>` prints `K_SPEED` (use the mean of the two straight
   runs, so the water current cancels out) and `K_TURN`.
3. Then tune the controller (`SPEED`, `K`, `RADIUS` in `control.py`) and the
   twin (`K_POS`, `K_HEAD`: lower is smoother but slower to follow the GPS).
