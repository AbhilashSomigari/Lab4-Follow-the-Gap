# Lab 4: Follow the Gap

Reactive obstacle avoidance for the F1TENTH car using the Follow the Gap
algorithm, implemented as the ROS 2 Python package `gap_follow`.

## Repository structure

```
.
├── gap_follow/                    # ROS 2 ament_python package
│   ├── gap_follow/
│   │   └── gap_follow_node.py     # Follow the Gap node
│   ├── config/params.yaml         # tunable parameters
│   ├── launch/gap_follow.launch.py
│   ├── test/                      # ament flake8 / pep257 / copyright tests
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── maps/                          # test maps for f1tenth_gym_ros
    ├── levine_blocked.png / .yaml
    └── levine_obs.png / .yaml
```

## Algorithm

`GapFollowNode` subscribes to `/scan` (`sensor_msgs/LaserScan`) and publishes
`/drive` (`ackermann_msgs/AckermannDriveStamped`). For every scan it runs:

1. **Preprocess** (`preprocess_lidar`): replace NaN/inf with `max_range`,
   clip far returns, apply a moving-average filter, and keep only the forward
   ±70° window (beams outside it are zeroed so they can never form a gap).
2. **Disparity extension** (`extend_disparities`): at every large jump between
   neighbouring beams, the far side is overwritten with the near distance for
   half a car width. Gaps the car cannot physically fit through disappear and
   the chosen heading stays off obstacle corners.
3. **Closest point and safety bubble**: find the closest beam and zero every
   beam within the bubble around it. The bubble radius grows with speed:
   `bubble_radius + bubble_speed_gain * speed`.
4. **Find max gap** (`find_max_gap`): the longest run of consecutive free
   beams. A beam is free when it is farther than `gap_threshold`; near corners
   that threshold is capped at `gap_threshold_ratio` × the deepest beam so the
   gap still points at the most open region. Equal-length gaps are ranked by
   total depth.
5. **Find best point** (`find_best_point`), the "better idea": smooth the gap
   with a `best_point_window`-beam moving average and aim at the centre of the
   deepest window. On a long straight, where many beams are equally far, this
   resolves to the middle of the corridor instead of twitching between beams.
6. **Corner guard**: Follow the Gap aims at the deepest point around a
   corner, and the line to it skims the inside wall. The node checks the
   clearance beside the car (60°–120° on each side of the raw scan). If the
   car is steering toward a side closer than `corner_clearance`, the steering
   toward that side is scaled down, reaching zero at half the car width.
7. **Steering and speed**: the target angle is clipped to the steering limit,
   then smoothed (exponential moving average plus a per-scan rate limit).
   Speed blends linearly from `high_speed` (straight) to `low_speed` (full
   lock), and is capped by `free path ahead / time_headway` (never below
   `min_speed`) so the car slows early when driving toward a wall.

## Build and run

```bash
# inside the sim container / ROS 2 workspace
cp -r gap_follow ~/sim_ws/src/
cd ~/sim_ws
colcon build --packages-select gap_follow
source install/setup.bash

ros2 launch gap_follow gap_follow.launch.py   # uses config/params.yaml
# or
ros2 run gap_follow gap_follow_node
```

Run the tests with `colcon test --packages-select gap_follow`.

### Using the test maps

Copy the files in `maps/` into `f1tenth_gym_ros/maps/`, then point
`f1tenth_gym_ros/config/sim.yaml` at the one you want, for example
`map_path: '<path>/f1tenth_gym_ros/maps/levine_obs'`.

On `levine_obs` the default spawn `(0, 0)` overlaps a small box
(x ∈ [-0.47, 0.33], y ∈ [0.03, 0.38]). If the car starts in collision, set
`sy: -0.3` in `sim.yaml`.

## Parameters

All parameters live in `gap_follow/config/params.yaml`.

| Parameter | Default | Meaning |
|---|---|---|
| `fov_deg` | 70.0 | half-angle of the forward window used (deg) |
| `max_range` | 6.0 | returns are clipped to this distance (m) |
| `smoothing_window` | 5 | moving-average window in preprocessing (beams) |
| `car_width`, `width_margin` | 0.31, 0.15 | width used by the disparity extender (m) |
| `disparity_threshold` | 0.3 | range jump treated as an obstacle edge (m) |
| `bubble_radius`, `bubble_speed_gain` | 0.2, 0.03 | safety bubble radius = base + gain × speed |
| `gap_threshold`, `gap_threshold_ratio` | 2.5, 0.7 | free-space threshold and its cap relative to the deepest beam |
| `best_point_window` | 40 | smoothing window when choosing the best point (beams) |
| `high_speed`, `low_speed` | 4.0, 2.0 | speed when straight / at full steering lock (m/s) |
| `min_speed` | 1.0 | floor for the free-path-ahead speed cap (m/s) |
| `corner_clearance` | 0.55 | side clearance below which steering toward that side is reduced (m) |
| `time_headway` | 0.5 | speed ≤ free path ahead / headway (s) |
| `steering_alpha`, `max_steering_step` | 0.6, 0.12 | steering smoothing and per-scan rate limit |

## Results (offline simulation)

The parameters were tuned in a lightweight offline simulator: a ray-cast LiDAR
on the map PNGs (1080 beams, 270° FOV), a kinematic bicycle model with
f1tenth_gym geometry, and a full-footprint collision check.

- **levine_blocked**: completes laps without collision from four different
  spawn poses in both directions around the loop. The car averages about
  3.4 m/s, takes corners at about 2.7 m/s, and stays at least 11 cm from the
  walls (the closest point is at the inside of corners).
- **levine_obs**: gets through most of the obstacle course but still collides
  at its hardest spots. The bottom-right corner, where a box narrows the exit
  to about 0.8 m, and lanes beside the ellipse that are 0.45 m wide for a
  0.31 m car need a few centimetres of precision.
