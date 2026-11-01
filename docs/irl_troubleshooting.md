# Real-robot troubleshooting and tuning

How to get `wall_following.py` working on the real robot: what to check first, which parameters to change for each symptom, and how to change them live without restarting.

See also: [functions.md](functions.md) · [algorithms.md](algorithms.md) · [ROTATE](mode_rotate.md) · [STRAFE](mode_strafe.md) · [ARC](mode_arc.md)

---

## 1. Before the first run

| Check | How |
|---|---|
| **`is_simulation = False`** | Top of `wall_following.py`. If you forget, the node stops with `Error: is_simulation is set to True…` as soon as it receives a 720-point real scan. |
| **Rebuild** | `colcon build --packages-select rb1101`. Not needed if the workspace was built with `--symlink-install`. |
| **Correct settings are running** | At startup the log should show `Movement mode: …, algorithm: …, max turns: …` with the values you expect. |
| **Lidar is publishing** | `ros2 topic hz /scan` should show about 10 Hz. If the log keeps saying `No scan detected, pausing...`, the lidar isn't publishing on `/scan`. Run `ros2 topic list` to find its topic name. |
| **Lidar direction** ⚠️ | See below. A rotated or mirrored lidar is the most common real-robot problem. |
| **Sideways movement** (STRAFE only) | With the wheels off the ground: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"`, then send a zero command to stop. If the robot can't drive sideways, don't use STRAFE. |

### Checking the lidar's direction

The code assumes **index 0 = front and angles count counter-clockwise**, so **index 90 = left and 270 = right** (after the real scan is reduced from 720 to 360 points).

1. Put a box about 30 cm **directly in front** of the robot and run `ros2 topic echo /scan --once`. The short readings should be at the **start and end** of `ranges`.
2. Move the box to the robot's **right**. The short readings should be around index **540 of 720** (270° after halving).

| What you see | Problem | Effect |
|---|---|---|
| Right-side box shows up near index 180 of 720 (90°) | Lidar counts clockwise (mirrored) | Left and right are swapped. The robot turns the wrong way and follows the wrong wall. |
| Front box isn't near index 0 | Lidar is mounted rotated | Every direction is shifted. Walls show up in the wrong detection box. |

**Fix:** shift or reverse the scan in `sub_scan_callback`. For example, `np.roll(ranges, k)` corrects a rotation of k degrees, and `ranges[::-1]` reverses the direction. Do this before anything else; no amount of tuning will fix it.

---

## 2. Changing parameters live (no restart)

Every value in the `parameters` list is a ROS parameter, and the code reads it fresh on every tick. While the node is running, in another terminal:

```bash
ros2 param list /WallFollowingNode                                   # see all parameters
ros2 param get  /WallFollowingNode right_wall_buffer_distance        # read one
ros2 param set  /WallFollowingNode right_wall_buffer_distance 0.18   # change it (decimal number)
ros2 param set  /WallFollowingNode angle_tolerance 10                # whole-number parameters need whole numbers
ros2 param set  /WallFollowingNode diagonal_x_threshold "[0.05, 0.45]"   # pairs of values
```

- **Match the type.** `angle_tolerance`, `slide_correction_factor`, `turn_correction_factor`, `ransac.start_inliers`, `ransac.inliers_decrease_factor` and `ransac.max_iter` are **whole numbers**. Everything else is a decimal, so write `0.2`, not `0`.
- **Live changes are lost** when the node restarts. Copy values that work into the `else:` (real-robot) block of `parameters` in `wall_following.py`.
- **Exceptions:** `movement_mode`, `algorithm`, `max_turns` and `is_simulation` are plain Python variables, not ROS parameters. Changing them means editing the file and restarting. `robot_dimensions` can be changed live, but ARC's clearance check only runs at startup.

---

## 3. Parameter reference (real-robot block)

| Parameter | Real-robot value | What it does | Raise it → | Lower it → |
|---|---|---|---|---|
| `translate_velocity` | 0.2 | Driving speed (m/s) | Faster, but moves further between scans, so it's likelier to miss corners or react late | Slower and more reliable |
| `translate_limit` | 0.4 | Speed cap | ⛔ **Don't change.** The README says changing it costs marks. | — |
| `turn_velocity` | 0.6 | Speed of the timed 90° turns in ROTATE (rad/s) | Turns more than 90° | Turns less than 90° |
| `turn_limit` | 1.0 | Turn-rate cap; also limits ARC turn speed | ⛔ **Don't change.** | — |
| `right_wall_buffer_distance` (**b**) | 0.16 | Target distance from the robot's centre to the right wall (m). ARC arc radius = b. | Follows further from the wall; wider arcs | Hugs the wall more closely |
| `slide_correction_factor` | 4 | Sideways correction speed = `translate_velocity / factor` | Gentler, slower distance correction (less zig-zag) | Stronger, faster correction (more zig-zag) |
| `turn_correction_factor` | 1 | Heading correction = angle × factor | Straightens up faster, but may overshoot and wobble | Gentler, slower to straighten up |
| `angle_tolerance` | 15 (degrees) | Off-parallel angle above which it **stops and rotates on the spot** to re-align (ROTATE/ARC) | Fewer stop-and-rotate moments | Straighter, but stops more often |
| `vertical_threshold` | 0.25 | How far ahead (from the centre) a wall counts as "wall ahead" (m) | Turns earlier at walls ahead | Turns later (risk of hitting the wall) |
| `diagonal_x_threshold` / `diagonal_y_threshold` | (0.05, 0.5) | (min, max) size of the corner detection boxes; the back-right box decides "open corner" in ROTATE/STRAFE | **max:** sees walls further away (more false detections) | **min:** ignores points too close to the robot |
| `robot_dimensions` | (0.1, 0.05) | (length, width): size of the side/front boxes and ARC's clearance check | — | — |
| `ransac.point_distance_threshold` | 0.03 | How close (m) a point must be to a line to count as part of the wall | Accepts noisier walls | Stricter. A noisy lidar may stop finding walls. |
| `ransac.start_inliers` | 15 | Minimum points for a line to count as a wall (lowered automatically if none is found) | Ignores small clutter | Finds short walls, but also clutter |
| `ransac.max_iter` | 20 | Attempts per line | More reliable, but slower each tick | Faster ticks |

> **Check `robot_dimensions`.** The real-robot block says 0.10 × 0.05 m, but the simulated nanocar's body is about 0.21 × 0.16 m. Measure the real robot. If it's bigger than 0.10 × 0.05, the side/front boxes are too small and ARC's clearance check is too optimistic.

---

## 4. Symptoms → what to change

**Read the log first.** Every tick logs the decision it made (for example `wall right, move forward`, `[arc] wall ahead at 0.31 m, arc left`, `Turn 3/12: left, turn sum = 3`). For more detail, set line 13 to `LoggingSeverity.DEBUG`; it then also prints the 8-direction `wall_array` and the wall angle every tick.

| Symptom | Likely cause | What to change |
|---|---|---|
| **Turns left at an open corner instead of right** | (1) The algorithm chose to go straight; the log says `…PLEDGE/HEADING says go straight (turn sum = 0)`. (2) The lidar is mirrored. (3) The back-right box missed the wall. | (1) That's the algorithm working as designed; try `algorithm = "PLAIN"` to confirm. (2) See §1. (3) With DEBUG on, check `wall_array[5]` at the corner. If it's 0, adjust `diagonal_*_threshold` or `right_wall_buffer_distance` so the wall falls inside the box. |
| **Follows the left wall / turns the wrong way everywhere** | Lidar mirrored or rotated | §1. Fix the scan, not the parameters. |
| **Zig-zags towards and away from the wall** | Distance correction too strong | Raise `slide_correction_factor` (e.g. 4 → 6) |
| **Slowly drifts away from or into the wall** | Distance correction too weak, or b is wrong | Lower `slide_correction_factor`; check `right_wall_buffer_distance` |
| **Wobbles its heading left and right** | Heading correction too strong | Lower `turn_correction_factor`; raise `angle_tolerance` |
| **Keeps stopping to rotate on the spot** | `angle_tolerance` too tight for a noisy lidar | Raise `angle_tolerance` (e.g. 15 → 20); raise `ransac.point_distance_threshold` |
| **Creeps forward very slowly** | No usable right wall: the wall angle is over 60° | Check the lidar direction (§1); lower `ransac.start_inliers` |
| **Hits the wall ahead before turning** | Turns too late | Raise `vertical_threshold`; lower `translate_velocity` |
| **Turns well before reaching the wall ahead** | Turns too early | Lower `vertical_threshold` |
| **Misses corners or reacts late** | Too fast for 10 Hz decisions | Lower `translate_velocity` |
| **ROTATE turns aren't 90°** | Timed turn and wheel slip on your floor | Change `turn_velocity` (see §5) |
| **Clips the wall end at outside corners** | Turns too early, or b too small for the robot | Raise `right_wall_buffer_distance`; check `robot_dimensions` |
| **"No scan detected, pausing..." all the time** | Lidar not on `/scan`, or not running | `ros2 topic list`, `ros2 topic hz /scan` |
| **Ticks are laggy and decisions late** | RANSAC too slow on the robot's computer | Lower `ransac.max_iter` (e.g. 20 → 10) |
| **Stops for good after a while** | `max_turns` reached; the log says `Reached N turns, stopping` | Raise `max_turns` or set it to `None` |

---

## 5. Values set in the code (not parameters)

These need a code edit and a restart:

| What | Where | Value | Notes |
|---|---|---|---|
| ROTATE turn duration | `turn_left_90deg` / `turn_right_90deg` | 50 × 0.05 s = 2.5 s | At `turn_velocity` 0.6 that's 1.5 rad ≈ **86°**. To calibrate, command one turn and measure the actual angle θ. Then set `turn_velocity = 0.6 × 90 / θ`, which must stay ≤ `turn_limit` (1.0). |
| Points needed to detect a wall in a box | `wall_scan(obstacle_threshold=5)` | more than 5 | Raise it if stray points trigger false walls |
| Right-distance window | `move_forward`, `step_arc` | indices 260–280 | The "directly to the right" reading |
| Distance dead band | `move_forward` | 0.8·b – 1.2·b (and ignores walls beyond 3·b) | Distances inside this band get no sideways correction |
| ARC triggers | `step_arc` | wall ahead within 2·b; open corner when the right side reads > 2·b | Both scale with b |
| Pause after a turn | `cooldown` | ROTATE 5 ticks, ARC 2, STRAFE 1 | Throws away scans taken mid-turn |
| Wall angle limit | `align_heading` | 60° | Above this, the line found probably isn't the right wall |

---

## 6. Mode-specific notes for the real robot

- **ROTATE:** turns are timed, so calibrate `turn_velocity` on the actual floor (§5). Small errors are corrected afterwards by the heading alignment.
- **ARC:** arcs are timed too. `turn_limit` = 1.0 caps the turn rate, so arcs with radius b = 0.16 m run at about 0.16 m/s and take about 1.6 s. There's no calibration parameter for arcs yet. If they consistently over- or under-turn on your floor, a scale factor could be added.
- **STRAFE:** needs a base that can move sideways (§1) and never corrects its heading, so expect drift on real wheels. Less clearance when moving sideways.

---

## 7. Recording a problem to look at later

```bash
ros2 bag record /scan /cmd_vel          # while the robot drives through the problem spot; Ctrl+C to stop
ros2 bag play <bag_folder>              # replay later (robot off), with the node running, to step through its decisions
rviz2                                    # add a LaserScan display on /scan to see what the lidar sees
ros2 topic echo /cmd_vel                 # see exactly what the robot is being told to do
```

Black, shiny or glass surfaces can return no reading at all (`inf` or 0). The code ignores those readings, so a wall made of that material can look like an opening.
