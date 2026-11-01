# ROTATE mode

```python
movement_mode = "ROTATE"
```

The original behaviour. The robot always **faces the direction it's moving**. At corners it stops and **turns 90° on the spot**.

Related functions: `step_rotate`, `align_heading`, `move_forward`, `turn_left_90deg`, `turn_right_90deg` (see [functions.md](functions.md)).

---

## Decision order (each 0.1 s tick)

```
align_heading()  ── off parallel by more than angle_tolerance? ──► rotate on the spot, done for this tick
      │
      ▼
wall ahead       (wall_array[0]) ──► turn_left_90deg()        inside corner
wall on right    (wall_array[6]) ──► move_forward(align=True)  normal wall following
wall back-right  (wall_array[5]) ──► turn_right_90deg()       outside corner (the wall just ended)
nothing                          ──► move_forward()           go forward until a wall is found
```

1. **Line up first.** `align_heading` uses the RANSAC wall angle. If the robot is more than `angle_tolerance` off parallel (and no more than 60°), it rotates on the spot and does nothing else this tick. Smaller errors become a steering correction (`turn_offset`) while driving.
2. **Wall ahead → turn left.** This is an inside corner. Under the right-hand rule, the only way on is left.
3. **Wall on the right → follow it.** Drive forward. `align=True` slides the robot sideways to stay about b from the wall.
4. **Only a back-right wall → turn right.** The right wall has just ended (an outside corner), so the robot turns right to follow the wall around. This only happens if `may_turn_right()` allows it. Under HEADING or PLEDGE the robot may instead leave the wall and go straight (see [algorithms.md](algorithms.md)).
5. **Nothing → go forward** until it finds a wall.

## Turning

`turn_left_90deg` / `turn_right_90deg` send `turn_velocity` (0.6 rad/s) for 50 × 0.05 s = 2.5 s, which is about 86°. They're **timed, with no sensor feedback**: the robot is effectively blind while turning. Afterwards a 5-tick cooldown (0.5 s) makes sure the next decision uses a fresh scan.

The small undershoot (86° instead of 90°) and any wheel slip are corrected afterwards by `align_heading`.

## Pros and cons

| Pros | Cons |
|---|---|
| Simple and predictable | Stops at every corner: 2.5 s turn plus 0.5 s cooldown |
| The lidar's front always matches the direction of travel, so detection is the same every time | Only reacts to an outside corner once the wall end is well behind (the right-hand box must clear first), so the robot ends up further from the new wall |
| Works on any base that can turn on the spot (differential or mecanum) | Blind for 2.5 s during each turn |
