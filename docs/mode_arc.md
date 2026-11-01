# ARC mode

```python
movement_mode = "ARC"
```

The robot drives **smooth quarter-circle arcs** around corners, moving forward and turning at the same time, instead of stopping to turn on the spot. The arc geometry depends only on the **robot's dimensions** and the **buffer distance**.

Related functions: `step_arc`, `arc_turn`, `check_arc_clearance`, `align_heading`, `move_forward` (see [functions.md](functions.md)).

---

## Symbols

| Symbol | Meaning | Simulation | Real robot |
|---|---|---|---|
| **b** | `right_wall_buffer_distance`: target distance from the robot's centre to the right wall | 0.20 m | 0.16 m |
| **L, W** | `robot_dimensions`: full length and width | 0.23, 0.10 m | 0.10, 0.05 m |
| **R** | arc radius | — | — |

## Geometry

### Inside corner (wall ahead → arc left)

A quarter-circle left arc of radius R moves the robot **R forward and R to the left**, finishing at 90°.

```
          ┃ wall ahead
          ┃
     ╭────┨  ← ends here, b from the wall (which is now on its right)
     │    ┃
  ───╯    ┃
  start   ┃
  ←  R+b →┃
```

If the wall ahead is D away, then after the arc it's `D − R` to the robot's right. To finish exactly b from it:

> **R = D − b**

- Normally the arc starts when **D = 2b**, which gives **R = b**.
- If the wall is detected when it's already closer, R gets smaller (a **tighter arc**), down to R = 0, which is a turn on the spot.

### Outside corner (right wall ends → arc right)

Ideally the robot pivots around the wall's end point with **R = b**. It then stays exactly b from the wall for the whole turn and finishes already following the new wall at b.

```
  ───────────●  ← wall end (pivot)
     robot → ╮│
             ╰┼  stays b from the wall end
              │
              │ new wall
```

At speed, the wall end is usually noticed slightly late, when it's already **e** behind the robot. A right arc of radius R then finishes `R + e` from the new wall, so:

> **R = b − e**

Clearance: the arc's centre is at (e, −R) from the robot. Measured from that centre, the wall end lies diagonally behind (at 225°), outside the quarter-circle the robot sweeps. So the robot is closest to the wall end at the start or end of the arc, at distance **√(b² + e²) ≥ b**.

### Clearance checks (`check_arc_clearance`)

These use only L, W and b (with R = b):

1. **Outside corner:** the robot's inner side passes b − W/2 from the wall end, so it needs **b > W/2**.
2. **Inside corner:** the outer-front corner swings towards the wall ahead, and the rear-inner corner swings towards the old right wall. Both stay clear if

   > **(R + W/2)² + (L/2)² < (R + b)²**

   The left side is the squared distance from the arc's centre to the robot's furthest corner. The right side is the squared distance from the arc's centre to each wall.

| | b > W/2 | Corner check (left < right) |
|---|---|---|
| Simulation | 0.20 > 0.05 ✓ | 0.076 < 0.16 ✓ |
| Real robot | 0.16 > 0.025 ✓ | 0.037 < 0.10 ✓ |

If either check fails, the node prints a warning at startup.

## Speed

`arc_turn` drives as fast as the limits allow:

- Turn rate `ω = min(turn_limit, translate_velocity / R)`, then forward speed `v = ω·R`.
- Duration = (π/2)/ω. It's split into 0.05 s steps, rounded **up** so ω never goes over `turn_limit` (which would make `move_2D` cut it back and stop short of 90°).

| | R = b arc | v | ω | Time |
|---|---|---|---|---|
| Simulation | 0.20 m | ~0.52 m/s | ~2.6 rad/s | 0.60 s |
| Real robot | 0.16 m | 0.16 m/s | ~0.98 rad/s | 1.60 s |

Like the 90° turns, arcs use **timing only, with no sensor feedback**. The arc blocks until it's done, then the robot stops and waits a 2-tick cooldown so stale scans taken during the arc are thrown away. This means a short pause (about 0.2–0.3 s) after each arc.

> A non-blocking version that kept moving after the arc was tried and then reverted: in Gazebo it caused the robot to miss an open corner and turn left at the next wall instead.

## Decision order (each 0.1 s tick)

Corners are checked **before** heading correction, so a corner is never noticed late because the robot was busy re-aligning. The right arc only happens if `may_turn_right()` allows it. Otherwise the robot carries on to the forward/alignment steps (see [algorithms.md](algorithms.md)).

```
front_dist ≤ 2b  or  wall_array[0]                     ──► arc_turn(left,  R = clip(front_dist − b, 0, b))
right_dist > 2b  and back_right_dist < 2b              ──► arc_turn(right, R = b − e)
align_heading() off by more than angle_tolerance       ──► rotate on the spot to re-align
wall on right (wall_array[6])                          ──► move_forward(align=True)
nothing                                                ──► move_forward()
```

| Reading | Taken from | Purpose |
|---|---|---|
| `front_dist` | nearest valid reading within ±10° of straight ahead | when to start the left arc, and how tight |
| `right_dist` | nearest valid reading at 260–280° (straight right) | has the wall end just passed? |
| `back_right_dist` | nearest valid reading at 225–260° | is the wall still behind-right? |
| `e` | −(largest x of the wall points at 180–270° within 2b) | how far behind the robot the wall end is |

Valid means not `inf`, not `nan` and not `0`.

The outside-corner trigger deliberately doesn't use `wall_array[5]`. That diagonal box only counts points more than 0.2 m to the side, and a wall exactly b = 0.2 m away sits right on that edge, so it might never register. The ±10° front cone doesn't pick up the wall being followed: at b = 0.2 m, the 350° reading only reaches it at about 1.15 m, well beyond 2b.

## Offline simulation results

These come from a 2D simulation that runs the real node code against fake walls and fake lidar scans, with the robot assumed to follow every command perfectly:

| Case | Simulation: distance to new wall | Real robot: distance to new wall | Smallest body-to-wall gap |
|---|---|---|---|
| Inside corner | 0.199 m (target 0.20) | 0.160 m (target 0.16) | ≥ 12 cm |
| Inside corner, wall already 0.3 m ahead (tight arc) | 0.199 m | 0.160 m | ≥ 11 cm |
| Outside corner | 0.200 m | 0.151 m | ≥ 12.6 cm |

Every arc ended at 90° ± 0.1°.

At simulation speed (0.8 m/s = 8 cm per tick), the outside-corner arc is usually the tighter version (R ≈ 0.06 m), because the wall end is noticed late. At the real robot's 0.2 m/s it's close to the full R = b.

## Pros and cons

| Pros | Cons |
|---|---|
| Smooth arcs instead of stopping to turn on the spot | Timed arcs with no feedback: wheel slip on the real robot will make them less accurate than the simulation |
| Finishes each corner already at distance b from the new wall | Blind during the arc (0.6 s in sim, 1.6 s on the real robot), then a short pause (~0.2–0.3 s) |
| Works on any base (only uses forward speed and turn rate) | The tighter arc at an inside corner relies on a reliable `front_dist` reading |
| Radius comes straight from b, with clearance guaranteed by L and W | |
