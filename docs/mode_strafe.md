# STRAFE mode

```python
movement_mode = "STRAFE"
```

The robot **never rotates**. It always faces the same way and moves forward, backward or sideways. At corners, instead of turning its body, it changes its **travel direction**.

This needs a base that can move sideways (for example mecanum wheels), because it uses `linear.y`.

Related functions: `step_strafe`, `set_travel_dir`, `move_relative`, `move_forward` (see [functions.md](functions.md)).

---

## Travel direction

`self.travel_dir` (φ) is the direction the robot is moving, measured from its front:

| φ | Moving towards | Robot speed sent |
|---|---|---|
| 0° | forward | +x |
| 90° | left | +y |
| 180° | backward | −x |
| 270° | right | −y |

"Turn left" becomes `φ += 90°` and "turn right" becomes `φ −= 90°`.

## How the same logic is reused

Instead of writing a second copy of the wall detection, the code **shifts the lidar scan** so it's seen from the travel direction:

```python
self.scan = np.roll(self.last_scan, -self.travel_dir)
```

After this, index 0 always points along the travel direction and index 270 is always on the travel-right side. `wall_scan`, `fitRansacLines` and the right-distance check in `move_forward` all read `self.scan`, so they work unchanged. "Front" now means "ahead in the travel direction".

Movement goes the opposite way. `move_relative(forward, slide)` takes speeds measured from the travel direction and rotates them by φ to get robot speeds:

```
x = forward·cos(φ) − slide·sin(φ)
y = forward·sin(φ) + slide·cos(φ)
```

For example, at φ = 270° (moving right), "forward 0.8" is sent as `y = −0.8`.

## Decision order (each 0.1 s tick)

```
wall ahead (travel direction)  ──► stop, set_travel_dir(+90)   start moving left
wall on travel-right           ──► move_forward(align=True)    move along it, keeping distance b
wall at travel-back-right only ──► stop, set_travel_dir(−90)   start moving right
nothing                        ──► move_forward()              keep going
```

These are the same rules as ROTATE mode, but seen from the travel direction. The "start moving right" step only happens if `may_turn_right()` allows it. Otherwise the robot keeps going in its current direction (see [algorithms.md](algorithms.md)). After a direction change there's a 1-tick pause so the next decision uses a fresh scan. **No turn command is ever sent**, and the "line up with the wall" step is skipped.

## Design notes

- **Detection box sizes aren't swapped when moving sideways.** Swapping length and width would make the "wall ahead" box ±0.23 m wide, which is wider than the 0.2 m distance to the wall being followed. That wall would then count as "ahead" and the robot would keep changing direction. The original box sizes work in every direction.
- **Clearance depends on direction.** Distances are measured from the lidar at the robot's centre. When moving sideways, the robot's longer side faces the wall: half-length is 0.115 m, compared with half-width 0.05 m. That leaves about 8.5 cm of clearance instead of 15 cm (simulation settings).

## Pros and cons

| Pros | Cons |
|---|---|
| Changes direction almost instantly (one 0.1 s tick) | Needs a base that can move sideways |
| No timed turns, so it's never blind | **No heading correction.** Fine in Gazebo, but on the real robot small turning errors build up over time |
| Simple: same rules as ROTATE, just seen from the travel direction | Less clearance when moving sideways (the robot's longer side faces the wall) |
| | Stops dead at each corner, which means sudden changes of direction |
