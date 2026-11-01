# Algorithms: PLAIN, HEADING and PLEDGE

```python
algorithm = "PLEDGE"   # "PLAIN", "HEADING" or "PLEDGE"
max_turns = 12         # stop for good after this many 90° turns; None = no limit
```

Both are set at the top of `src/rb1101/rb1101/wall_following.py`. The algorithm decides **when the robot leaves the wall**. It's independent of `movement_mode`, which decides **how** the robot turns ([ROTATE](mode_rotate.md), [STRAFE](mode_strafe.md), [ARC](mode_arc.md)), so all 9 combinations work.

Related functions: `register_turn`, `may_turn_right` (see [functions.md](functions.md)).

---

## Tracking turns

Every completed 90° corner calls `register_turn(+1)` for left or `register_turn(−1)` for right, in every movement mode:

| Mode | Where the turn is recorded |
|---|---|
| ROTATE | `turn_left_90deg` / `turn_right_90deg` |
| STRAFE | `set_travel_dir` (the travel direction acts as the heading, since the body never rotates) |
| ARC | `arc_turn`, including the tight R = 0 version |

The small rotations `align_heading` makes to stay parallel to the wall **don't count**.

It keeps three numbers:

| Variable | Meaning | Used by |
|---|---|---|
| `turn_count` | Total number of turns; every left or right turn adds 1 | `max_turns` |
| `heading_sum` | Running total of turns: left = +1, right = −1 | PLEDGE |
| `heading` | `heading_sum mod 4`: which way the robot faces relative to its start (0 = start direction, 1 = 90° left, …) | HEADING |

Headings are relative to wherever the robot starts. `gz_heading.sh` (yaw 0) and `gz_pledge.sh` (yaw π) both count as "heading 0" at the start.

## The one rule that differs

All three algorithms use the same right-hand wall following:

- **Wall ahead:** always turn left. This (re)starts wall following.
- **Wall on the right:** follow it.
- **Right wall ends (outside corner):** turn right **only if `may_turn_right()` allows it**. Otherwise the robot leaves the wall and carries straight on.

| Algorithm | `may_turn_right()` | Leaves the wall when… |
|---|---|---|
| PLAIN | always True | never |
| HEADING | `heading != 0` | it's facing its starting direction (turn sum ≡ 0 mod 4) |
| PLEDGE | `heading_sum != 0` | the running total is **exactly** 0 |

With PLEDGE, the robot only follows walls after a left turn, and it never turns right when the total is 0. So the total never goes negative.

### `may_turn_right()` for each turn sum (from the offline test)

| turn sum | −1 | 0 | 1 | 2 | 3 | **4** | 5 |
|---|---|---|---|---|---|---|---|
| PLAIN | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| HEADING | ✓ | ✗ | ✓ | ✓ | ✓ | **✗** | ✓ |
| PLEDGE | ✓ | ✗ | ✓ | ✓ | ✓ | **✓** | ✓ |

**Turn sum 4 is the case the "G" test checks.** After a full circle to the left inside the G, the robot faces its starting direction again:

- **HEADING** sees "heading 0" and leaves the wall straight away. Depending on where that happens, it can end up back in the G.
- **PLEDGE** knows the robot has turned a full circle, so it keeps following until the total unwinds to exactly 0. This is what guarantees it escapes.

This matches the README objectives:
- `gz_heading.sh`: the robot should escape the G and carry straight on.
- `gz_pledge.sh`: the robot should escape the G and turn right after it.

## Stopping after N turns

When `turn_count` reaches `max_turns`, `register_turn`:

1. sets `self.finished = True`,
2. sends a stop command,
3. logs `Reached N turns, stopping`.

From then on, `timer_callback` only sends stop commands. Set `max_turns = None` to turn the limit off. It applies to every algorithm, PLAIN included.

Each turn is logged, so you can follow progress in the terminal:

```
Turn 3/12: left, turn sum = 3, heading = 270 deg
Right corner, but PLEDGE says go straight (turn sum = 0)
Reached 12 turns, stopping
```

## Offline test results

These come from the same 2D simulation as the ARC tests. It runs the real node code against fake walls and fake lidar, and assumes the robot follows every command perfectly. The robot follows a right wall that ends at an outside corner, starting with turn sum 0:

| Mode | PLAIN | PLEDGE | HEADING |
|---|---|---|---|
| ROTATE | turns right (heading −90°) | goes straight (heading 0°) | goes straight (heading 0°) |
| STRAFE | starts moving right (−y) | carries straight on (+x) | carries straight on (+x) |
| ARC | arcs right (heading −90°) | goes straight (heading 0°) | goes straight (heading 0°) |

The turn counter and stop limit were also checked in all three modes. After 3 turns (left, left, right) with `max_turns = 3`: count = 3, sum = +1, and the robot stays stopped.

The G mazes themselves haven't been tested yet; run `gz_heading.sh` / `gz_pledge.sh` in Gazebo.
