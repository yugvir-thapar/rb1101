# `wall_following.py`: function reference

This file explains every function in `src/rb1101/rb1101/wall_following.py`, in the order they appear. Each movement mode has its own page:

- [ROTATE mode](mode_rotate.md): turn the robot 90° on the spot at corners
- [STRAFE mode](mode_strafe.md): never rotate; slide sideways or forward instead
- [ARC mode](mode_arc.md): drive quarter-circle arcs around corners

The algorithms (when to leave the wall) and the turn limit are covered in [algorithms.md](algorithms.md). Real-robot tuning and troubleshooting is in [irl_troubleshooting.md](irl_troubleshooting.md).

`wall_following.sh` just sources the workspace and runs this node:

```bash
source install/setup.bash
ros2 run rb1101 wall_following
```

---

## Coordinate conventions

The lidar gives 360 range readings, **one per degree, counter-clockwise from the front** (standard ROS):

```
            0° (front)
               ↑
  90° (left) ←   → 270° (right)
               ↓
           180° (back)
```

Each reading at angle θ with range r becomes a point `x = r·cos(θ)`, `y = r·sin(θ)`. **+x is forward and +y is left**, so anything on the right has a negative y.

The robot follows the **right-hand rule**: it keeps a wall on its right and follows it around the maze.

---

## Module-level settings

| Variable | Meaning |
|---|---|
| `is_simulation` | `True` uses the Gazebo parameter set; `False` uses the real-robot set. |
| `movement_mode` | `"ROTATE"`, `"STRAFE"` or `"ARC"`: how the robot turns at corners. An `assert` stops the node at startup if it's misspelled. |
| `algorithm` | `"PLAIN"`, `"HEADING"` or `"PLEDGE"`: when the robot leaves the wall (see [algorithms.md](algorithms.md)). Also checked by an `assert`. |
| `max_turns` | Stop for good after this many 90° turns. `None` = no limit. |
| `parameters` | Tuning values, declared as ROS parameters in `__init__` (see below). |

Main parameters:

| Parameter | Used for |
|---|---|
| `ransac.*` | How strict the wall (line) detection is |
| `translate_velocity`, `translate_limit` | Driving speed and its cap |
| `turn_velocity`, `turn_limit` | Turning speed and its cap |
| `diagonal_x_threshold`, `diagonal_y_threshold`, `vertical_threshold` | Size of the detection boxes in `wall_scan` |
| `robot_dimensions` | Robot (length, width) |
| `right_wall_buffer_distance` (**b**) | Target distance from the robot's centre to the right wall |
| `angle_tolerance` | How far off parallel the robot can be before it stops to re-align |
| `slide_correction_factor`, `turn_correction_factor` | Gains for the distance and heading corrections |

---

## `ransac(points, maxIter, threshold, d)`

Finds **one straight line** (one wall) in a cloud of 2D points using RANSAC (Random Sample Consensus).

1. If there are fewer than 2 points, it returns "no line" straight away.
2. It picks two random points B and C. These define a candidate line.
3. Every other point within `threshold` (3 cm) of the line is an **inlier**. The rest are **outliers**.
4. If there are at least `d` inliers, it accepts the line and stops. Otherwise it tries another random pair, up to `maxIter` times.
5. It returns `(line, B, C, inliers, outliers)`. `line` is `None` if nothing was found.

RANSAC is used because it ignores noise and clutter. A real wall produces many points in a row, and random noise doesn't.

## `point2lineDist(A, B, C)`

Returns the perpendicular distance from point A to the line through B and C.

It uses the 2D cross product. `|(C−B) × (B−A)|` is the area of the parallelogram the two vectors form. Dividing by the base length `|C−B|` gives its height, which is the distance.

---

## `WallFollowingNode.__init__`

Sets up the node:

- **Publisher** on `cmd_vel`, to send velocity commands.
- **Subscriber** on `scan`, to receive lidar data.
- **Timer** at 10 Hz that runs `timer_callback`, the main control loop.
- **State:**
  - `last_scan`: the newest raw scan.
  - `scan`: the scan as seen from the travel direction.
  - `travel_dir`: the direction the robot is moving, relative to its body (only changes in STRAFE mode).
  - `cooldown`: ticks to wait after a turn.
  - `ransacLineParams`: the detected walls.
  - `turn_count`, `heading_sum`, `heading`, `finished`: turn tracking for the algorithms and the turn limit.
- It declares the parameters, logs the movement mode, algorithm and turn limit, and in ARC mode runs `check_arc_clearance()`.

## `sub_scan_callback(msg)`

Stores the newest lidar ranges in `self.last_scan`. The real lidar can return more than 360 points, so it keeps every second one to get 1 reading per degree. If that happens while `is_simulation = True`, it raises an error, because the wrong settings are in use.

## `fitRansacLines(nInliers, start_idx, stop_idx)`

Finds **all the walls within a range of angles** of the scan.

1. **Pick the range.** It uses `self.scan`, which is shifted to the travel direction. If `stop_idx > 360`, the start of the scan is appended so the range wraps past 0°. That's how 270–440 reaches through the front to 80°.
2. **Convert to x/y.** It drops readings that are `inf`, `nan` or `0` (no return), and points with `x ≥ 0.3` or `y ≥ 0.3`.
3. **Find lines repeatedly:**
   - Run `ransac` to find a line.
   - Work out the line's **angle** relative to the robot's heading, wrapped to −90°…+90°. A line has no direction, so 180° apart is the same line.
   - **Sign:** 0° means the wall is parallel to the robot. A positive angle means the robot is turned left of the wall, so it should correct by turning right.
   - Save `{p1, p2, angle}`, remove that line's inliers, and look for another wall in what's left.
   - Stop when nothing is found, too few points are left, or 4 lines have been found.
4. **Fallback:** if it found no lines, it retries the same range with fewer required inliers (`nInliers − 5`), until that would reach 0.

## `angle_right()`

From all the detected right-side walls, returns the angle of the **most parallel** one (smallest |angle|). That's how far the robot is turned away from the wall it's following. Returns `None` if no walls were found.

## `wall_scan(obstacle_threshold=5)`

Checks for a wall in **8 directions**. Each index covers a 90° slice of the scan centred on `idx × 45°`:

| idx | Direction | Detection box (points must fall inside) |
|---|---|---|
| 0 | front | `|x| < vertical_threshold`, `|y| < robot width` (narrow box straight ahead) |
| 1, 3, 5, 7 | front-left, back-left, back-right, front-right | `min < |x| < max` and `min < |y| < max` from the diagonal thresholds (box off the corner) |
| 2 | left | `|x| < robot length`, `|y| < 1.3 × diagonal max` (box beside the robot) |
| 4 | back | same as front |
| 6 | right | same as left |

A direction counts as having a wall if **more than 5 points** fall inside its box. The function returns an array such as `[1,0,0,0,0,1,1,0]`.

It reads `self.scan`, so in STRAFE mode "front" means the travel direction, not the robot's front.

> The extra back-right check at the end (using `back_right_lines`) can only set `wall_detected_arr[5]` when it's already 1, so it currently has no effect. It was left as is; see the earlier review notes.

## `move_2D(x, y, turn)`

The only function that sends commands to the robot. It clips each speed to `translate_limit` / `turn_limit` and publishes a `Twist`:

- `linear.x`: forward speed
- `linear.y`: sideways speed (positive = left). This only works on a base that can move sideways.
- `angular.z`: turning rate (positive = counter-clockwise, i.e. left)

## `move_relative(forward, slide, turn)`

Like `move_2D`, but `forward` and `slide` are measured **from the travel direction**. It rotates them by `travel_dir` to get robot speeds:

```
x = forward·cos(φ) − slide·sin(φ)
y = forward·sin(φ) + slide·cos(φ)
```

When `travel_dir = 0` (always the case in ROTATE and ARC), this is the same as `move_2D`.

## `move_forward(speed, turn_offset, align)`

Drives in the travel direction, with an optional steering correction (`turn_offset`).

With `align=True`, it also **keeps a set distance from the right wall** by sliding sideways. It looks at the closest reading in the 260–280° window (directly to the right):

| Right-wall distance | Action |
|---|---|
| between 1.2·b and 3·b (too far) | slide right |
| less than 0.8·b (too close) | slide left |
| between 0.8·b and 1.2·b | no correction. This dead band stops the robot from wobbling side to side. |

This is a simple on/off (bang-bang) distance controller.

## `set_travel_dir(delta)`  *(STRAFE)*

Changes `travel_dir` by `delta` degrees (+90 = left, −90 = right) **without rotating the robot**. It then pauses for 1 tick and clears the scan, so the next decision uses a fresh scan. Finally it calls `register_turn`.

## `register_turn(direction)`

Called after every completed 90° corner, in every mode (+1 = left, −1 = right). It:

- adds 1 to `turn_count`;
- adds `direction` to `heading_sum`;
- sets `heading = heading_sum mod 4`;
- logs the turn.

When `turn_count` reaches `max_turns`, it sets `finished = True` and stops the robot. See [algorithms.md](algorithms.md).

## `may_turn_right()`

Decides whether the robot should follow the wall around an outside corner:

- **PLAIN:** always yes.
- **HEADING:** yes unless `heading == 0`.
- **PLEDGE:** yes unless `heading_sum == 0`.

"No" means leave the wall and carry straight on. All three `step_*` functions check this before turning right.

## `turn_left_90deg()` / `turn_right_90deg()`  *(ROTATE)*

**Timed turns with no sensor feedback.** Each sends a turn command 50 times, 0.05 s apart (2.5 s in total). At 0.6 rad/s that's 1.5 rad, about 86°.

Afterwards each one sets `cooldown = 5` (a 0.5 s pause), clears `last_scan` so the next decision uses a scan taken after the turn, and calls `register_turn`.

## `arc_turn(direction, radius)`  *(ARC)*

Drives a **quarter circle** (+1 = left, −1 = right) of the given radius, moving forward and turning at the same time. It works out its speeds like this:

- Turn rate `ω = min(turn_limit, translate_velocity / radius)`, or `turn_limit` when the radius is 0 (turn on the spot).
- Number of 0.05 s steps = (π/2)/ω, **rounded up**. ω is then adjusted so the steps add up to exactly 90°, and it can only go down, so it stays within `turn_limit`.
- Forward speed `v = ω × radius`.

Like the 90° turns, it uses timing only, with no sensor feedback, and blocks until the arc is done. Afterwards it stops, sets `cooldown = 2` (so stale scans taken during the arc are thrown away) and calls `register_turn`.

## `check_arc_clearance()`  *(ARC)*

Runs once at startup in ARC mode. It warns if the geometry is unsafe, using only `robot_dimensions` (L, W) and b:

- `b > W/2`: the robot's inner side clears the wall end at outside corners.
- `(b + W/2)² + (L/2)² < (2b)²`: the robot's corners clear both walls at inside corners.

The derivation is in [mode_arc.md](mode_arc.md).

## `stop()`

Sends zero speed.

---

## `timer_callback()`: the main loop

Runs 10 times a second:

0. **Finished** (`max_turns` reached): send a stop command and do nothing else.
1. **No scan yet:** stop and wait.
2. **Cooldown after a turn:** stop, count down, and throw the scan away.
3. **Shift the scan to the travel direction:** `self.scan = np.roll(last_scan, −travel_dir)`, so index 0 is always the travel direction. This changes nothing when `travel_dir = 0`.
4. **Find the walls:** run RANSAC on the front-right (270–360°) and back-right (180–271°) ranges, then work out `angle = angle_right()`.
5. **Find obstacles:** `wall_array = wall_scan()`.
6. **Hand over to the current mode:** `step_rotate`, `step_strafe` or `step_arc`.

## `align_heading(angle, wall_array)`  *(shared by ROTATE and ARC)*

Keeps the robot parallel to the right wall. It returns `(handled, turn_offset)`:

- `turn_offset = −angle × turn_correction_factor`. This is proportional (P) control: the bigger the misalignment, the harder it steers back.
- **0° < |angle| ≤ `angle_tolerance`:** no stop. `turn_offset` is used as a small steering correction while driving.
- **`angle_tolerance` < |angle| ≤ 60°:** it **rotates on the spot** to re-align and returns `handled=True`.
- **|angle| > 60°:** it's probably not the right wall. It retries RANSAC over a wider range (270–440°). If there's still no usable wall:
  - **Nothing ahead:** it creeps forward slowly (`handled=True`).
  - **Wall ahead:** it returns `handled=False` so the corner logic can deal with it.

## `step_rotate(angle, wall_array)`

Decisions for ROTATE mode. See [mode_rotate.md](mode_rotate.md).

## `step_strafe(wall_array)`

Decisions for STRAFE mode. See [mode_strafe.md](mode_strafe.md).

## `step_arc(angle, wall_array)`

Decisions for ARC mode. See [mode_arc.md](mode_arc.md).

## `main()`

Starts ROS, creates the node, and runs it until it's stopped.
