# Collision Avoidance Integration — Progress Ledger

**Plan:** `.claude/Plans/2026-07-23-collision-avoidance-integration.md` (13 tasks)
**Method:** Subagent-driven development (fresh implementer subagent per task → task reviewer → fix loop → mark complete)
**Branch:** `fix/avoidance-integration` (created Task 1, based off `4f01fa4` on `swarm`)
**Repo:** `/home/finn/Documents/ardu_ws/src/formation_control` (its own git repo)
**SDD workspace:** `.superpowers/sdd/` (task briefs, implementer reports, review diffs, ledger — git-ignored)

---

## Status Board

| Task | Title | Status | Commits |
|---|---|---|---|
| 1 | Shared geodesy module `geo_utils.py` | ✅ Complete | `7f1302b` |
| 2 | Commander target handoff + channel gating (BREAK-2, BREAK-3) | ✅ Complete | `6fb8d77` |
| 3 | Launch file for the avoidance stack (BREAK-1) | ✅ Complete | `852b9a2`, `da525b5` |
| 4 | Formation commander common frame (LATENT-1) | ✅ Complete | `c2d6228` |
| 5 | Controller common frame + d_min=1.0 (LATENT-1) | ✅ Complete | `eb8f52a`, `fc144b8` |
| 6 | Safety monitor common frame + thresholds (LATENT-1) | ✅ Complete | `6a794bc` |
| 7 | Per-primitive trajectory propagation (LATENT-4) | ⬜ Not started | — |
| 8 | Stop-command fallback with loud warning (LATENT-3) | ⬜ Not started | — |
| 9 | Paper Eq. 2 cost function (LATENT-5) | ⬜ Not started | — |
| 10 | Velocity-command actuation on `/ap_N/cmd_vel` (LATENT-5) | ⬜ Not started | — |
| 11 | Controller defers to emergency stops | ⬜ Not started | — |
| 12 | Loud warning when velocity feed is absent (LATENT-6) | ⬜ Not started | — |
| 13 | Runtime verification in simulation (spec Part 3) | ⬜ Not started | — |

**Final whole-branch code review:** pending (after Task 13).

---

## Task 1 — Shared geodesy module `geo_utils.py` ✅

**Completed:** 2026-07-23
**Commit:** `7f1302b` — "feat: add shared geo_utils module for GPS<->ENU conversion"
**Diff:** `4f01fa4..7f1302b` — 2 files, +99 / -0

### What was built
- **Created `formation_control/geo_utils.py`** (34 lines): the single source of truth for GPS↔ENU conversion.
  - `EARTH_RADIUS = 6378137.0`
  - `gps_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt) -> np.ndarray` → `[east, north, up]` metres
  - `enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt) -> (lat°, lon°, alt m)`
  - Pure module: `math` + `numpy` only, **no ROS imports** — so its tests run without a sourced ROS env.
- **Created `tests/unit/test_geo_utils.py`** (65 lines): imports the **real** module (not the inline-mock-copy pattern used by the legacy `tests/unit/test_phase*.py`).

### Verification
- TDD followed: RED (failing before implementation) → GREEN.
- New tests: **9/9 passing**. Full unit suite: **114 passing**, 3 pre-existing unrelated failures in `test_phase1_core_algorithms.py::TestTrajectoryOptimizer` (confirmed pre-existing — this diff adds two brand-new files and touches no trajectory code).
- Note: the brief states "Expected: 10 passed", but its own verbatim test code collects 9 items (4 + 1 + 4 parametrized). Independently recounted by the reviewer — **brief miscount, not a dropped requirement**. No action needed.

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** · No Critical or Important findings.

Reviewer verified by hand:
- Axis convention correct in **both** directions — x=East→longitude, y=North→latitude, z=Up (`geo_utils.py:38-42`, `geo_utils.py:47-50`).
- `cos(latitude)` scaling applied to the east/longitude axis only, using the **reference** latitude consistently in both directions (`geo_utils.py:40`, `geo_utils.py:49`) — the classic asymmetric-latitude bug is not present.
- Round-trip is exact (both functions linearize around the same fixed `ref_lat`); `atol=1e-6` reflects float error, not model error.
- Tests would actually catch an East/North swap — each axis test asserts the expected axis is non-zero **and** the orthogonal axis is zero.
- Scope discipline: `formation_commander.py` and `collision_avoidance_controller.py` untouched, as Task 1 requires.

### Controller resolution of reviewer ⚠️ items
1. **"Legacy formula test re-derives the formula inline rather than calling `formation_commander._local_to_gps`"** — Accepted, not a gap. The brief specifies this test verbatim, and the duplication it fails to guard is *transient*: Tasks 4, 5, and 6 replace those call sites with `geo_utils` imports, deleting the second copy entirely. Risk window closes by Task 6. No fix dispatched.
2. **"Branch name not verifiable from the diff"** — Verified directly by the controller: `git branch --show-current` → `fix/avoidance-integration`, HEAD `7f1302b`, parent `4f01fa4`. ✅

### Minor findings carried forward (for the final whole-branch review to triage)
- `geo_utils.py:40,49` — no guard/docstring caveat for `cos(ref_lat) == 0` (poles) or antimeridian wraparound. Out of scope for the brief (fleet operates at CMAC/Canberra), but worth a one-line docstring caveat since this module is now the fleet-wide single source of truth.
- `test_geo_utils.py:99-109` — "matches legacy formula" is asserted against a hand-copied formula rather than the real legacy method (see ⚠️ 1 above; self-resolving by Task 6).

---

## Task 2 — Commander target handoff + command-channel gating (BREAK-2, BREAK-3) ✅

**Completed:** 2026-07-23
**Commit:** `6fb8d77` — "feat: publish /formation/targets/drone_N and gate cmd_gps_pose on avoidance_active (BREAK-2/3)"
**Diff:** `7f1302b..6fb8d77` — 2 files, +31 in `formation_commander.py`, new test file

### What was built
- **`formation_commander.py`** — four edit sites, all verbatim per brief:
  - Parameter `avoidance_active` (bool, default `False`) → `self.avoidance_active`.
  - New dict `self.target_pubs: Dict[int, Publisher]`.
  - Per-instance publisher `/formation/targets/drone_{N}` (`PoseStamped`, depth 10) created in `_setup_drone_communications`.
  - `_formation_control_loop` now **always** publishes the local ENU target (`frame_id='map'`, metres, identity orientation), then `continue`s past the `cmd_gps_pose` block when `avoidance_active` — so exactly one node owns `/ap_N/*`.
- **`tests/unit/test_avoidance_wiring.py`** (new): real-node tests (`FormationCommander` with `MagicMock` publishers), 6 tests covering handoff in both modes, gating in both modes, and emergency bypass.
- `_local_to_gps` deliberately untouched — frame unification is Task 4.

### Verification
- TDD followed: RED (`AttributeError` on missing `avoidance_active`/`target_pubs`) → GREEN.
- New tests: **6/6 passing**. Full unit suite: **120 passed, 3 failed** — the same known `TestTrajectoryOptimizer` baseline, no new regressions.

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** · No Critical or Important findings.

Reviewer verified by hand:
- Emergency-authority invariant intact — traced both call sites of `_send_emergency_hold_commands()`; the `EMERGENCY_HOLD` branch `return`s before the gated block, so emergency commands are never suppressed.
- Target topic contract matches the interface spec exactly (type, name, depth, `frame_id='map'`, ENU metres).
- Scope discipline: controller, safety monitor, and algorithm files untouched.
- New tests import the **real** module — the inline-mock-copy pattern was not extended.

### Controller resolution of reviewer ⚠️ items
1. **"Cannot verify subscriber-side QoS compatibility on `/formation/targets/drone_N`"** — Verified directly by the controller. `collision_avoidance_controller.py:144-149` subscribes with plain depth `10`; the new publisher uses plain depth `10`. Both are rclpy defaults (RELIABLE / VOLATILE / KEEP_LAST(10)) — an exact match, so the pair is compatible. Not a gap; no fix dispatched. (The BEST_EFFORT constraint applies to `/ap_N/*` topics, not to this internal topic.)

### Minor findings carried forward (for the final whole-branch review to triage)
- `tests/unit/test_avoidance_wiring.py:151` — unused `import numpy as np` (plan-mandated, verbatim from the brief). Lint noise only.
- New tests assert only `pose.position.{x,y,z}`; they do not assert `header.frame_id == 'map'` or identity orientation, though the implementation sets both. Coverage gap worth closing given AP_DDS's frame-rule sensitivity.

---

## Task 3 — Launch file for the avoidance stack (BREAK-1) ✅

**Completed:** 2026-07-23
**Commits:** `852b9a2` — "feat: add collision_avoidance launch file starting full stack (BREAK-1)"
 · `da525b5` — "fix: rename controller status topic to drone_N (invalid ROS2 topic token crashed all controllers)"
**Diff:** `6fb8d77..da525b5` — 2 commits

### What was built
- **Created `launch/collision_avoidance.launch.py`**: starts `safety_monitor`, `formation_commander` (with `avoidance_active: True`, `num_drones: 4`), and 4 × `collision_avoidance_controller` (instance ID via `argv[1]`, node names `collision_avoidance_controller_1..4`).
- **`setup.py`**: added the `launch/` data_files entry so `glob('launch/*.launch.py')` installs to `share/formation_control/launch/`. Wholesale replacement verified to drop nothing — all 6 console_scripts entry points and both original data_files entries preserved.
- **`package.xml`**: declared `geographic_msgs` and `ardupilot_msgs`, which the code already imported but never declared.

### ⚠️ Plan gap found and fixed mid-task (user decision)
Task 3's smoke test exposed a **pre-existing crash the plan never covered**: `collision_avoidance_controller.py:171` published on `/collision_avoidance/status/{instance_id}` → `/collision_avoidance/status/1`. **ROS 2 rejects any topic token starting with a digit**, so `create_publisher` raised `InvalidTopicNameException` for every instance — *all 4 controllers crashed on startup*, 2/6 nodes up. Reproduced independently by the controller before acting.

Nothing subscribed to the topic (publish-only telemetry). The only test covering it — `test_phase2_controller_node.py:517` — asserts against an **inline mock copy**, which is exactly why the crash went undetected for so long.

**User decision (2026-07-23):** rename to `/collision_avoidance/status/drone_{N}`, matching the `/formation/targets/drone_{N}` convention already in the plan, and fold the fix into Task 3 so its launch file actually works. Applied in `da525b5`:
- `collision_avoidance_controller.py:171` — one-token rename, no other behaviour touched.
- `test_phase2_controller_node.py:127,515-517` — stale topic string updated (file left mock-based; restructuring it is out of scope).
- **New real regression test** `TestControllerTopicNames` in `tests/unit/test_avoidance_wiring.py` — constructs the REAL `CollisionAvoidanceController` (signature `__init__(self, instance_id: int = 1, num_drones: int = 4)`, verified) with no mocked `create_publisher`, parametrized over instances 1-4, so ROS 2's topic-name grammar genuinely gates it.

### Verification
- TDD followed on the fix: RED (`InvalidTopicNameException`) → GREEN.
- `colcon build --packages-select formation_control` succeeded; launch file confirmed installed.
- **Smoke test (no sim): 6/6 nodes up** — `/formation_commander`, `/safety_monitor`, `/collision_avoidance_controller_1..4`, clean `initialized for drone N` logs, no tracebacks. (Before the fix: 2/6.)
- `test_avoidance_wiring.py`: **10/10 passing**. Full unit suite: **124 passed, 3 failed** — the known `TestTrajectoryOptimizer` baseline, no new regressions.

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** · No Critical or Important findings.

Reviewer verified by hand:
- `setup.py` entry-point count (6) independently re-counted — nothing dropped by the wholesale replacement.
- Repo-wide grep for the **old** topic name → no stale references, no orphaned subscriber anywhere. The rename is self-contained.
- The regression test is not a string tautology — it exercises the real node's `create_publisher` against ROS 2's grammar.
- The pasted 6-node smoke-test output is genuine evidence, not a partial or doctored paste.

### Controller resolution of reviewer ⚠️ items
1. **"`CollisionAvoidanceController.__init__` accepting `instance_id=` is inferred from the test run, not visible in the diff"** — Verified directly by the controller: `collision_avoidance_controller.py:44` → `def __init__(self, instance_id: int = 1, num_drones: int = 4):`. ✅ Not a gap.

### Minor findings carried forward (for the final whole-branch review to triage)
- `tests/unit/test_avoidance_wiring.py:127` — the regression test's `assert node.status_pub is not None` is trivially true; the real signal is that construction did not raise. Harmless, but the assertion adds nothing — consider asserting the publisher's actual topic name instead, for clarity of intent.
- **`safety_monitor` emits CRITICAL log spam while idle** (no sim, no drone data). Pre-existing and out of scope for Task 3, but it will make Task 13's runtime verification noisy and could mask real emergencies. Task 6 owns this file — worth handling there or flagging at final review.

---

## Task 4 — Formation commander common frame (LATENT-1, commander half) ✅

**Completed:** 2026-07-23
**Commit:** `c2d6228` — "fix: commander uses shared ENU frame from geopose + single-origin subscription (LATENT-1)"
**Diff:** `da525b5..c2d6228`

### What was built
All six brief edit sites in `formation_commander.py`:
- Imports hoisted (`:28,32`); parameter `origin_instance` (int, default 1) declared and read (`:66,74`).
- **Single** origin subscription `/ap_{origin_instance}/gps_global_origin/filtered` created before the per-instance loop (`:126-132`); the old per-instance origin block deleted. This kills the **origin race** — previously all four drones' origin topics fed one callback and first-arrival non-deterministically defined the shared frame.
- Position input switched from per-drone-origin `/ap_N/pose/filtered` to global `/ap_N/geopose/filtered` (`:147-152`).
- `_drone_position_callback` → `_drone_geopose_callback` (`:186-199`): guards on `gps_origin_set` (a position without the shared origin cannot be placed in the common frame) and delegates to `geo_utils.gps_to_enu`.
- `_local_to_gps` body replaced by `enu_to_gps` delegation (`:216-237`), docstring and `RuntimeError` guard kept, manual math deleted.
- Dead code removed: `gps_origin_subs` dict, the `EARTH_RADIUS` instance attribute, a stale debug log referencing the deleted `gps_origin_topic`.
- 4 new tests appended to `test_avoidance_wiring.py` (`TestCommanderCommonFrame`); imports hoisted to the top block per controller direction.

### Verification
- TDD followed: RED 4/4 failing → GREEN. `test_avoidance_wiring.py`: **14/14 passing** (brief's "10 passed" was a stale count predating Task 3's additions — controller resolved this before dispatch). Full unit suite: **128 passed, 3 failed** (known `TestTrajectoryOptimizer` baseline).

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** · No Critical or Important findings.

Both pre-authorized cross-cutting risks checked and clean:
1. **`drone_positions` reader audit complete** — grep returns exactly 5 sites: writer at `:250`, `_check_formation_status` (`:387,392`), `_send_emergency_hold_commands` (`:464,474`), one comment. No missed reader. This mattered because `drone_positions` keeps its type (3-element list) while changing *meaning* to shared-frame ENU — a missed reader would fail silently, not loudly. Reviewer independently traced that both readers move from frame-**in**consistent to frame-consistent, i.e. Task 4 fixes two latent bugs as a side effect.
2. **No manual GPS/ENU formula survives** in `formation_commander.py` — grep for `EARTH_RADIUS`/`math.cos`/`math.radians`/degree conversions returns nothing. `geo_utils` is now genuinely the single implementation, closing the Task-1 duplication finding.

Also verified: `test_geopose_converted_to_shared_enu` re-derives the expected north offset independently (`math.radians(0.001) * EARTH_RADIUS`) rather than re-importing the implementation's own conversion — so it would catch an axis swap.

### Controller resolution of reviewer ⚠️ items
1. **"Cannot verify runtime behaviour against a live `/ap_N/geopose/filtered` publisher"** — Correctly deferred, not a gap. Runtime verification against the live topic is **Task 13**'s explicit job (spec Part 3); Task 4's scope is unit-level. Carried into the Task 13 checklist rather than fixed here.

### Minor findings carried forward (for the final whole-branch review to triage)
- `test_avoidance_wiring.py:307-318` — `test_geopose_converted_to_shared_enu` uses the origin's longitude, so `east` is trivially ~0. It exercises north/up only and would not catch an argument-order slip at the **call site** involving the `cos(ref_lat)` east-scaling term. Plan-mandated (verbatim from the brief). An east-axis case would close it.
- `_check_formation_status` and `_send_emergency_hold_commands` are now frame-corrected as a byproduct but have no regression test locking that in.

---

## Task 5 — Controller common frame + d_min=1.0 (LATENT-1 controller half) ✅

**Completed:** 2026-07-23
**Commits:** `eb8f52a` — "fix: controller shared ENU frame, d_min=1.0, GlobalPosition frame_id (LATENT-1)"
 · `fc144b8` — "test: assert frame_id='map' on controller GPS command and pin east axis" (review fix)
**Diff:** `c2d6228..fc144b8`

### What was built
Three coupled changes, which the plan requires in one pass:
1. **LATENT-1 controller half** — `collision_avoidance_controller.py` now reads global `/ap_N/geopose/filtered` and converts via `geo_utils` into the shared ENU frame; `_position_callback` → `_geopose_callback` with a `gps_origin_set` guard; single origin subscription anchored at `/ap_{origin_instance}/gps_global_origin/filtered`. Same shape as Task 4's commander.
2. **d_min 3.0 → 1.0 m** (USER DECISION 2026-07-10) — all three defaults in `collision_avoidance.py` (`CollisionPredictor`, `TrajectoryOptimizer`, `CollisionAvoidanceAlgorithm`) plus the controller's `d_min` parameter.
3. **`gps_cmd.header.frame_id = 'map'`** (`:281`) — without it `AP_DDS_ExternalControl.cpp:54-80` makes ArduPilot **silently drop** every `cmd_gps_pose` command. Interim; Task 10 replaces this GPS path with `cmd_vel`.

Dead code removed (`import math`, `self.EARTH_RADIUS`) orphaned by the `_local_to_gps` rewrite, mirroring Task 4.

### Verification
- TDD followed: RED → GREEN. New `tests/unit/test_avoidance_algorithm_fixes.py` (3 tests, real module) + 5 appended to `test_avoidance_wiring.py` → **22 + 3 passing**. Full unit suite: **136 passed, 3 failed** (known `TestTrajectoryOptimizer` baseline, **unshifted** — confirming those mock-copy tests are indeed blind to real-module changes).

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** (after one fix round) · No Critical findings.

All three pre-authorized cross-cutting risks checked clean:
1. **Reader audit complete** — one writer (`_geopose_callback`), four readers in `_avoidance_control_loop`; no stale reader.
2. **No manual GPS/ENU formula survives** in the controller.
3. **No production call site passes `d_min=3.0`** — remaining 3.0 occurrences are confined to out-of-scope mock-copy test fixtures.

Reviewer also confirmed `test_subscribes_geopose_and_single_origin` instantiates the controller as **instance 2** and still asserts `/ap_1/...` — so it genuinely catches a controller anchoring at its own instance instead of `origin_instance`.

### Review fix round (1 Important + 1 Minor, both closed by `fc144b8`)
- **Important: nothing tested the `frame_id='map'` fix** — the task's highest-severity behaviour change shipped with zero real-module assertions; a regression would have left the suite green while ArduPilot silently discarded every command. Closed by `test_gps_command_uses_map_frame`, which drives the REAL controller through its actual guards to the actual `publish()` with only the publisher mocked.
  - **Mutation-verified:** with `:281` commented out the test fails `AssertionError: assert '' == 'map'`; restored, it passes. Re-reviewer independently confirmed the claim is coherent — `GlobalPosition.header.frame_id` defaults to `''`, `:281` is the only place it is set, and `_avoidance_control_loop` swallows no exceptions, so nothing can mask the result.
  - Controller verified test-only scope by hand: `git diff --stat` shows one file (+21), and `:281` is present in the production file.
- **Minor: asymmetric axis assertion** — east axis now pinned (`== approx(0.0)`), matching the sibling commander test; verified correct against `gps_to_enu`'s `[east, north, up]` ordering.

### Controller resolution of reviewer ⚠️ items
1. **"Cannot verify `/formation/targets/drone_N` is in the same shared ENU frame as the controller's positions"** — Verified directly by the controller. Commander `target_positions` derive from the `formation_center` parameter (`formation_commander.py:94,105,304`) — a pure local ENU frame with no drone-specific origin — and since Task 4 that frame is anchored at `origin_instance`'s GPS origin. The controller's positions anchor at the same `/ap_{origin_instance}/gps_global_origin/filtered`. **Same frame ✅** — both nodes default `origin_instance` to 1 and the launch file overrides neither.

### Minor findings carried forward (for the final whole-branch review to triage)
- **Config divergence risk:** the shared frame is only shared because commander and controllers independently default `origin_instance` to 1. Setting it on one node and not the other would silently desync the frames with no error — exactly the LATENT-1 failure mode this plan exists to remove. Worth having `collision_avoidance.launch.py` set `origin_instance` explicitly on **all** nodes from one value.
- **Velocity-frame consistency unaudited:** `self.drone_velocities` (from `/ap_N/twist/filtered`, untouched) is now paired in `DroneState` with positions in the new shared ENU frame. Likely already EKF world-frame, but it was outside this task's audited scope. Relevant to Tasks 10/12.
- Legacy mock-copy fixtures still carry `d_min=3.0`. Harmless (they test inline copies) but misleading to a reader.

---

## Task 6 — Safety monitor common frame + thresholds (LATENT-1 monitor half) ✅

**Completed:** 2026-07-23
**Commit:** `6a794bc` — "fix: safety monitor shared ENU frame + thresholds for d_min=1.0 (LATENT-1)"
**Diff:** `fc144b8..6a794bc`

### What was built
The **third and final** application of the shared-ENU pattern (commander → Task 4, controller → Task 5, monitor → Task 6). LATENT-1 is now closed across all three nodes the plan covers.
- `safety_monitor.py` — thresholds aligned to `d_min=1.0`: `safety_radius` 3.0 → **1.0**, `warning_radius` 6.0 → **3.0**, changed in *both* the hardcoded default (`:48-49`) and the `declare_parameter` calls (`:74-75`).
- Parameter `origin_instance` (int, default 1) (`:78,87`); **single** shared-origin subscription `/ap_{origin_instance}/gps_global_origin/filtered` on `sensor_qos`, created before the per-drone loop (`:110-114`).
- Per-drone input switched `PoseStamped` on `/ap_N/pose/filtered` → `GeoPoseStamped` on `/ap_N/geopose/filtered` (`:118-124`).
- `_drone_position_callback` → `_gps_origin_callback` (idempotent latch) + `_drone_geopose_callback` (guards on `gps_origin_set`, delegates to `geo_utils.gps_to_enu`) (`:148-169`).
- Dead code removed, mirroring Tasks 4/5: the now-unused `PoseStamped` import (`TwistStamped` retained — still used at `:129`) and the never-read `self.drone_position_subs` dict. Per-drone position debug counters dropped per the brief.
- **Created `tests/unit/test_safety_monitor_frames.py`** — 5 real-node tests (thresholds, 0.5 m ⇒ risk 3, 2.0 m ⇒ not critical, geopose-before-origin skipped, subscribes-geopose-not-local-pose).

### Verification
- TDD followed: RED → GREEN. New file **5/5 passing**, output pristine. Full unit suite: **141 passed, 3 failed** — the known `TestTrajectoryOptimizer` baseline, no new regressions.
- Brief's "Expected: 6 passed" is a **miscount** (its verbatim code defines 5 tests) — resolved by the controller before dispatch and independently re-counted by the reviewer. Same miscount pattern as Task 1.

### Review outcome
**Spec compliance: ✅** · **Task quality: Approved** · No Critical or Important findings. First task in this plan to pass with zero fix rounds *and* zero Important findings.

Reviewer verified by hand:
- `gps_to_enu` call site (`:163-167`) passes `(lat, lon, alt, ref_lat, ref_lon, ref_alt)` in the exact order `geo_utils.py:15` defines — checked against the real `geo_utils` source, not the report's claim.
- **Reader audit independently re-run** (not taken on faith from the report): all readers of `drone_positions` are `:264,272,281-282,415`; each is either frame-agnostic (key/membership) or correctly consumes shared ENU. No missed reader — this mattered because the attribute keeps its type (3-element list) while changing *meaning*.
- Scope discipline: the debug-counter removal was authorized for position only; the analogous velocity/status counters were correctly left alone.

### Controller resolution of reviewer ⚠️ items
1. **"Cannot verify `/ap_N/geopose/filtered` and the origin topic publish the assumed field layout at runtime"** — Correctly deferred, not a gap. Identical to the ⚠️ raised on Tasks 4 and 5; runtime verification against live topics is **Task 13**'s explicit job (spec Part 3). Carried into the Task 13 checklist.

### Idle CRITICAL log spam — investigated, unchanged
Task 3 flagged `safety_monitor` CRITICAL spam while idle and suggested Task 6 as the place to fix it. Investigated and deliberately **not** fixed (outside the brief; fixing it would have been unrequested scope). The `gps_origin_set` guard does **not** incidentally silence it: with no simulator there is no origin message either, so the code path is unchanged. Source is `safety_monitor.py:239-240` (`total_offline >= num_drones // 2`). **Still open — carried to Task 13**, where it will make runtime verification noisy and could mask real emergencies.

### Minor findings carried forward (for the final whole-branch review to triage)
- `safety_monitor.py:8-10` — module docstring still advertises `/ap_N/pose/filtered` and omits the new `gps_global_origin` subscription. Stale after this change; a doc pass over all three converted nodes would be worthwhile.
- `test_safety_monitor_frames.py:56-58` — `offset_north` varies latitude only and all assertions are distance **magnitudes**, so the tests cannot distinguish an East/North axis swap. Plan-mandated (verbatim from the brief) and the same limitation already accepted in Tasks 4/5 — but it now stands unclosed in all three test files.

---

## ▶ Left off here

**Task 6 is complete and reviewed clean, with no fix round required. LATENT-1 (cross-frame distance comparison) is now fully closed — all three nodes share one ENU frame anchored at `origin_instance`.**

**Next action:** dispatch the Task 7 implementer.
- Task 7 = *Per-primitive trajectory propagation (LATENT-4)* — the first of the three algorithm-internals tasks (7, 8, 9) in `collision_avoidance.py`.
- Base commit for Task 7's review package is **`6a794bc`**. Brief already generated at `.superpowers/sdd/task-7-brief.md`.
- **Expect the pre-existing baseline to shift here.** The 3 known failures in `test_phase1_core_algorithms.py::TestTrajectoryOptimizer` test *inline mock copies* of exactly the code Tasks 7-9 rework. Their status changing is expected, not a regression — but each change must be explained, not waved through.

### Standing notes for every remaining task
- **Build gotcha:** imports resolve to the *installed* copy. After every source edit and before any runtime test: `colcon build --packages-select formation_control && source install/setup.bash` from `/home/finn/Documents/ardu_ws`.
- **Test gotcha:** `python3 -m pytest` from `src/formation_control` puts the package root first on `sys.path`, so `formation_control.*` resolves to **source** — this is what makes TDD possible here.
- **Do not extend the mock-copy pattern.** `tests/unit/test_phase*.py` and `tests/integration/*` test inline mock copies and will keep passing regardless of these changes. All new tests import the real modules.
- **3 pre-existing failures** in `test_phase1_core_algorithms.py::TestTrajectoryOptimizer` are the known baseline — Tasks 7-9 rework that code and should be expected to change their status.
- **Pre-flight plan conflict scan was skipped** at the user's direction. The per-task review loop remains the net for plan contradictions as they surface.
