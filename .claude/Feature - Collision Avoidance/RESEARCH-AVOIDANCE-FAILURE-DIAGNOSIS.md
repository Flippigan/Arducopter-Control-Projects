---
feature: Collision Avoidance
date: 2026-07-04
phase: Research / Root-Cause Diagnosis
status: Complete
based_on: IMPLEMENTATION_LOG.md (iteration 3), REVIEW.md, Marek et al. 2025 paper
scope: Why the current implementation does not cause drones to avoid one another
---

# RESEARCH: Why the Drones Do Not Avoid One Another

## TL;DR

**The collision avoidance code never executes during the test.** The launch
sequence starts only `swarm_takeoff_commander.py` and `formation_commander.py`.
The formation commander contains no avoidance logic — it streams straight-line
GPS targets. The node that implements the paper's algorithm
(`collision_avoidance_controller.py`) is never launched, and even if it were,
it would sit idle because the topic it depends on has no publisher. The drones
fly directly at their sphere slots and pass through each other exactly as
commanded.

There are three independent breaks in the integration chain (any one alone is
fatal), plus five latent algorithm/frame bugs that will surface once the wiring
is fixed.

---

## Part 1: The Broken Integration Chain

### BREAK-1 (CRITICAL): `collision_avoidance_controller` is never launched

- The documented test sequence starts only the takeoff commander and formation
  commander. Nothing starts `collision_avoidance_controller.py`.
- The package has **no launch files** at all, and the controller must run as
  **four separate processes** — one per drone. Instance ID comes from argv
  (`collision_avoidance_controller.py:304`); running the file bare gives
  instance 1 only.
- `safety_monitor.py` is also never launched, so the emergency-stop backstop
  is dead too. This is the `"Safety system not detected"` warning spamming the
  formation commander log every second (`formation_commander.py:485`).
- Net effect: **zero avoidance code runs during the observed flights.**

### BREAK-2 (CRITICAL): `/formation/targets/drone_N` has no publisher

- The controller's only source of goal positions is a subscription to
  `/formation/targets/drone_{N}` (`collision_avoidance_controller.py:144-149`).
- A grep of the entire package shows this topic exists **only in tests and in
  that one subscription**. `formation_commander.py` never publishes it — the
  intended handoff (commander publishes local ENU targets → controller
  publishes safe GPS commands) was designed but never wired in.
- Consequence: `target_position` stays `None` forever, so
  `_avoidance_control_loop()` returns at Step 1 on every tick
  (`collision_avoidance_controller.py:250-251`) and **never publishes a single
  command**, even when properly launched.

### BREAK-3 (CRITICAL): Command channel conflict on `/ap_N/cmd_gps_pose`

- `formation_commander._formation_control_loop()` publishes direct-to-target
  GPS commands at 10 Hz (`formation_commander.py:314-357`) on the **same
  topic** the avoidance controller would publish to at 20 Hz
  (`collision_avoidance_controller.py:162-166`).
- ArduPilot GUIDED mode obeys the most recent position target. Interleaved
  direct commands continuously re-aim each drone onto its straight-line
  (collision) course, cancelling any avoidance detour.
- Fix requirement: when the avoidance layer is active, the formation commander
  must **stop publishing `cmd_gps_pose` entirely** and publish only local
  targets on `/formation/targets/drone_N`.

---

## Part 2: Latent Bugs (will bite after Part 1 is fixed)

### LATENT-1 (HIGH): Inter-drone positions compared across different frames

- Each drone's `/ap_N/pose/filtered` is relative to **its own EKF origin**
  (its spawn point). The world spawns drones in a ~2 m pentagon
  (`ardupilot_gazebo/worlds/iris_multiuav.sdf:120-153`), so drone local frames
  are mutually offset by ~2.8–4 m.
- The controller computes neighbor distances from raw local poses
  (`collision_avoidance_controller.py:258-265`). The error (2–4 m) is the same
  magnitude as `d_min = 3.0 m`: two drones physically touching can appear
  ~4 m apart (detector never fires), and vice versa. `safety_monitor.py` has
  the same flaw.
- Related: `formation_commander._gps_origin_callback` subscribes to all four
  drones' `gps_global_origin/filtered` topics with one callback and keeps
  whichever origin arrives **first** (race condition) — all targets are then
  computed from that arbitrary origin.
- **USER DECISION (2026-07-10): use `/ap_N/geopose/filtered`** as the common
  frame. This topic is confirmed present in this ArduPilot's DDS topic table
  (`src/ardupilot/libraries/AP_DDS/AP_DDS_Topic_Table.h:231`) and reports each
  drone's **global** pose (`geographic_msgs/GeoPoseStamped`:
  lat/lon/alt + orientation), so all drones already live in one frame and no
  per-drone origin bookkeeping is needed. Convert each drone's lat/lon/alt to a
  shared local ENU (e.g. relative to a single chosen reference origin) before
  computing inter-drone distances, then feed those common-frame positions to
  the collision predictor.
  - Implementation impact: replace the `PoseStamped` subscription on
    `/ap_N/pose/filtered` (`collision_avoidance_controller.py:127-132`) with a
    `GeoPoseStamped` subscription on `/ap_N/geopose/filtered`, and apply the
    same change in `safety_monitor.py`. `formation_commander.py` position
    callbacks (`_drone_position_callback`, line 227) and the assembly check
    (`_check_formation_status`) must consume the same common-frame positions so
    every node agrees on where each drone is.
  - (Alternative not chosen: transforming `/ap_N/pose/filtered` via each
    drone's own `/ap_N/gps_global_origin/filtered`.)

*(LATENT-2 removed 2026-07-10 — superseded by the decision to change `d_min`
from 3.0 m to 1.0 m, which sits below the 2.8–4 m pentagon spawn spacing, so
the takeoff hover-lock it described cannot occur. The `d_min` change itself is
tracked as Part 4 step 4. Numbering of the remaining LATENT items is
preserved.)*

### LATENT-3 (HIGH): "All primitives unsafe" fallback does not brake

- The fallback returns the zero-acceleration primitive
  (`collision_avoidance.py:358-364`), whose waypoint is
  `current + v·Δt` — i.e. **keep flying at current velocity straight into the
  predicted collision**.
- Paper Section 3.4 / Eq. 4: the drone should decelerate/stop at `p_safe`.
  Worst-case behavior must be a braking command toward `p_safe`, never
  continued drift.
- **USER DECISION (2026-07-10): keep it super simple — command a stop.** The
  all-primitives-unsafe case is very unlikely, and it firing at all likely
  means there are bigger problems; skip `p_safe` selection entirely and have
  the affected drone fire a stop command (hold current position). Because the
  `t_iter` prediction looks up to `t_horizon` (2.0 s) ahead, the drone stops
  before the situation cascades into a collision (worst-case head-on at
  2×5 m/s ≈ 20 m warning vs ≈ 12.5 m combined braking distance at a_max).
  - Implementation: `TrajectoryOptimizer.optimize()` returns `None` instead of
    the hover primitive when all primitives are rejected
    (`collision_avoidance.py:358-364`). The existing dead branch in
    `compute_safe_waypoint()` (`if best_primitive is None: return my_position`)
    then emits the hold waypoint; re-commanded at 20 Hz from the braking
    drone's current position, ArduPilot GUIDED decelerates to a clean stop.
  - Add a loud warning when this fallback engages so it is visible in logs.

### LATENT-4 (HIGH): The primitive-safety test doesn't test the primitive

- Collision predictions are computed **once from current velocity**, then each
  primitive is judged by whether its endpoint lies within `d_min` of `p_safe`
  (`collision_avoidance.py:337-347`).
- But `p_safe` is a stopping point along the drone's own current velocity —
  not an obstacle. A primitive accelerating **directly at the neighbor** can
  pass this test; a safe lateral escape can fail it.
- Per the paper, each candidate primitive should be **propagated against
  neighbors' predicted trajectories** over the horizon and rejected if it
  violates `d_min` anywhere along the way.

#### The exact equation from the paper (Section 3.4, "Collision Avoidance Prediction")

The phenomenon we need — *stepping through successive future timesteps `t_iter`
and, at each step, checking every other drone/obstacle's predicted position
against `d_min`* — is stated by the paper in **prose**, and the inequality
itself is **not assigned its own equation number**. Verbatim (p. 13):

> "The algorithm analyzes the future positions of drones over successive time
> steps `t_iter` and checks whether the distance between the future position of
> the analyzing drone `p_i(t + t_iter)` and the position of another drone in the
> swarm or an obstacle `p_j(t + t_iter)` is **not smaller than the defined
> minimum safe distance `d_min`**. If the boundary of this distance is breached,
> the analyzing drone calculates the distance to the potential collision
> position `d_col` and adjusts its position to maintain a safe distance, while
> staying on course toward its destination."

So the safety condition being checked at each `t_iter`, for every neighbor `j`,
is the inequality:

```
‖ p_i(t + t_iter) − p_j(t + t_iter) ‖  ≥  d_min          (unnumbered; Section 3.4 prose)
```

A breach (`< d_min`) is what triggers the **only numbered equation** in this
section, **Equation (4)** — the response that computes the safe stopping
position:

```
p_safe(t) = p_i(t) + v̂_d · (d_col − d_min)               (Eq. 4)
```

where `v̂_d` is the unit desired-velocity direction and `d_col` is the distance
along the path to the predicted collision point. `p_safe(t)` is then "used to
compute the control velocities of the drone."

**Important caveat — the paper is internally inconsistent here, and the code
inherited the wrong branch.** The `t_iter` forward-prediction check above
(Section 3.4) is the *predictive* mechanism the user is asking about. But the
paper's own pseudocode, **Algorithm 1**, does **not** contain a `t_iter` loop.
Algorithm 1 line 5 instead tests a single-step repulsion trigger:

```
‖ r_j(t) · Δt ‖ < d_min       where  r_j(t) = p_i(t) − p_j(t)   (Algorithm 1, line 5)
```

i.e. it only reacts to *current* proximity (scaled by Δt), not to *predicted
future* collisions. The current implementation blends these two: it computes a
single `p_safe` from the current velocity (partial Eq. 4 flavour) but never
performs the per-`t_iter`, per-neighbor forward check the Section 3.4 prose
prescribes. **The fix for LATENT-4 is to implement the Section 3.4 inequality
literally** — for each candidate primitive, march `t_iter` over the horizon and
reject the primitive if `‖p_i(t+t_iter) − p_j(t+t_iter)‖ < d_min` for any
neighbor `j` at any step — rather than the endpoint-vs-`p_safe` proxy currently
in `collision_avoidance.py:337-347`.

### LATENT-5 (MEDIUM): Actuation mismatch — GPS micro-waypoints vs velocity control

- Paper Algorithm 1 integrates the resultant vector R(t) as a velocity-like
  command each Δt (`p_i(t+Δt) = p_i(t) + R(t)Δt`).
- The implementation publishes a GUIDED global-position target 0.25–2.75 m
  ahead, 20×/s. ArduPilot's guided position controller replans on every new
  target and never exits its acceleration-limited ramp → evasive response is
  far too sluggish for closing speeds of several m/s.
- Recommended channel: `/ap_N/cmd_vel` (TwistStamped velocity control), which
  matches the paper's control model directly.
- Note: the cost function in `calculate_cost_with_repulsion()` (alignment
  heuristic) also deviates from paper Eq. 2
  (`Σ 1/‖p_i−p_j‖² + λ‖v_i−v_d‖²`). Not fatal, but unvalidated.

### LATENT-6 (MEDIUM): Velocity feed silently degrades to zero

- `/ap_N/twist/filtered` IS in this ArduPilot's DDS topic table
  (`src/ardupilot/libraries/AP_DDS/AP_DDS_Topic_Table.h:195`), so REVIEW.md
  concern SF-002 is likely moot — but verify at runtime:
  `ros2 topic hz /ap_1/twist/filtered`.
- If the topic doesn't arrive, the controller substitutes zero velocity
  (`collision_avoidance_controller.py:253-254, 260`), which blinds the
  predictor: static drones "never collide" until already inside `d_min`
  (and with LATENT-1 frame offsets, possibly never). Should be a loud
  warning, not a silent default.

---

## Part 3: Runtime Verification Steps

With the sim up, confirm the chain breaks empirically:

```bash
ros2 node list                              # no collision_avoidance / safety_monitor nodes
ros2 topic info /formation/targets/drone_1  # 0 publishers  → BREAK-2
ros2 topic info /ap_1/cmd_gps_pose          # shows who actually commands the drone
ros2 topic hz /ap_1/twist/filtered          # verify velocity feed (LATENT-6)
ros2 topic echo /ap_1/geopose/filtered --once   # confirm chosen common frame is published (LATENT-1)
ros2 topic hz /ap_1/geopose/filtered        # confirm geopose update rate is adequate for 20 Hz loop
```

Workflow note: the install tree is currently in sync with src, but the
commanders are run via `python3 <src file>` while their `formation_control.*`
imports resolve to the **installed** copy under
`install/formation_control/lib/python3.10/site-packages/`. Always
`colcon build` after edits (or use `--symlink-install` / `ros2 run`), or the
fix being tested is not the code actually imported.

---

## Part 4: Recommended Fix Order

1. **Wire the handoff (BREAK-2/3):** formation commander publishes local ENU
   targets on `/formation/targets/drone_N`; add a parameter/mode so it stops
   publishing `cmd_gps_pose` when the avoidance layer owns the channel.
2. **Launch the stack (BREAK-1):** create a launch file (or documented
   sequence) starting safety_monitor + 4× collision_avoidance_controller
   (instance IDs 1–4) alongside the commander.
3. **Unify frames (LATENT-1):** switch all position inputs to
   `/ap_N/geopose/filtered` (chosen approach) and convert to a shared local
   ENU before any inter-drone distance calculation.
4. **Set `d_min` = 1.0 m (was 3.0 m):** update
   `collision_avoidance_controller.py:61` (`declare_parameter('d_min', 3.0)`)
   and the three defaults in `collision_avoidance.py`
   (`CollisionPredictor.__init__`, `TrajectoryOptimizer.__init__`,
   `CollisionAvoidanceAlgorithm.__init__`); review `safety_monitor.py`
   separation thresholds for consistency. Must land in the same pass as
   step 3 — with per-drone frames offset up to ~4 m, a 1 m `d_min` is
   meaningless until positions share a common frame.
5. **Algorithm fixes (LATENT-3/4):** stop-command fallback (hold current
   position; see LATENT-3 decision); per-primitive trajectory propagation for
   the safety check.
6. **Actuation (LATENT-5):** migrate avoidance output to `/ap_N/cmd_vel`;
   revisit cost function against paper Eq. 2.
7. **Hardening (LATENT-6):** loud warning when velocity feed is absent.
