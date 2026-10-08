# Valve Index Controller — Data Access Deep Dive
**Headset-free · Linux · ROS2 | Research date: 2026-07-10**
Method: multi-agent verified web research (18 sources, 89 claims extracted, 25 adversarially verified 3-vote, 23 confirmed) + direct source-code inspection of `cntools/libsurvive` master and `asymingt/libsurvive_ros2`.

---

## TL;DR for the purchase decision

Every physical input on the Index controller is decodable **headset-free and Steam-free** on Linux via libsurvive — including the channels the old issue #180 said were missing (grip squeeze force, finger capacitance, touch flags — all fixed 2020-08-29). The practical gaps are all in the last hop, the `libsurvive_ros2` wrapper:

- **Pose arrives on `/tf` only** — no `PoseStamped` topic. Your mapping node needs a tf2 lookup or a ~30-line relay.
- **Battery and haptics exist in the libsurvive C API but are not wired into the ROS2 node** — a small fork adds them.
- **Update rates were never established by any verifiable source** — bench-measure on day one.
- SteamVR/OpenVR headset-free is **effectively dead for controllers** (broken since 2017, issue still open). Monado/OpenXR uses libsurvive underneath anyway and adds constraints, not channels. **libsurvive_ros2 is the only sane path.**

---

## What the hardware itself contains

| Sensor/actuator | Notes |
|---|---|
| IR photodiode array | Lighthouse sweep timing → 6DoF optical pose |
| IMU (gyro + accel) | Fused with optical in libsurvive's Kalman tracker |
| Analog trigger | value + click + capacitive touch |
| Thumbstick | X/Y + click + touch |
| Trackpad (track button) | X/Y + touch + **force** |
| Grip force sensor | Analog squeeze (the "crush the rock" sensor) |
| A / B / System buttons | click + capacitive touch |
| 87 capacitive sensors | Exposed by libsurvive as 4 per-finger proximity channels |
| Battery + charge state | Percent + charging flag |
| Haptic LRA | Output: freq/amplitude/duration pulses |

## Path 1 — libsurvive / libsurvive_ros2 (recommended)

### Channel matrix (primary-source verified)

`driver_vive.c` `RFKnuckles()` maps **all 12 input axes**; `survive_types.h` enums confirmed:

| Channel | libsurvive core | Reaches ROS2 today via libsurvive_ros2 |
|---|---|---|
| 6DoF pose | ✅ Kalman, IMU-driven + light corrections | ⚠️ `/tf` only (`tracking_frame` → device serial). No PoseStamped topic |
| Raw IMU | ✅ | ✅ `/libsurvive/imu` (`sensor_msgs/Imu`; accel m/s², gyro passthrough, **orientation unpopulated**, frame `{serial}_imu`) |
| Trigger analog | ✅ axis 1 | ✅ `/libsurvive/joy` (`sensor_msgs/Joy`) |
| Trackpad X/Y | ✅ axes 2, 3 | ✅ Joy |
| Finger proximity (middle/ring/pinky/index) | ✅ axes 4–7, raw 0–1 capacitance (not calibrated curl) | ⚠️ Joy — delivery unverified; wrapper issue #11 showed ≥1 axis dropped (2024) |
| Grip squeeze force | ✅ axis 8 | ⚠️ same caveat |
| Trackpad force | ✅ axis 9 | ⚠️ same caveat |
| Thumbstick X/Y | ✅ axes 10, 11 | ✅ Joy |
| Button clicks (trigger, trackpad, thumbstick, system, A, B) | ✅ | ✅ Joy buttons (lower bits) |
| Capacitive touch flags (0x01 trigger, 0x10 A, 0x20 B, 0x40 thumbstick, 0x08 menu) | ✅ | ✅ Joy buttons — **touch states packed into upper bits** |
| Battery % + charging | ✅ `handle_battery()`; simple API: `survive_simple_object_charge_percent()` / `_charging()` | ❌ not republished — needs fork |
| Haptic output | ✅ `survive_vive_send_haptic()`; simple API: `survive_simple_object_haptic(freq, ampl, duration)` | ❌ no ROS subscriber — needs fork |

Also published: `/libsurvive/cfg` (`diagnostic_msgs/KeyValue` device config). Joy is event-driven (published on input change), not fixed-rate.

### Known risks on this path
1. **Maintainer caveat**: README roadmap still says button callback "only works on Tracker 2.0 wirelessly via watchman" (present as of 2025). Counter-evidence: the Joy handler is device-agnostic and wrapper issue #11 (closed Sep 2024) shows a real user receiving knuckles Joy messages — minus one expected axis. Status: **plausible-but-unvalidated; bench test mandatory.**
2. **Tracking jitter** reports against libsurvive v1.0/1.01 with Gen2 lighthouses (issues #263/#324) — noted, not independently adjudicated. Filter/validate before flying drones off this signal.
3. **Setup logistics**: wireless headset-free use needs a **Watchman dongle per controller** (the RF receiver normally lives in the HMD); one-time pairing + firmware updates typically still use SteamVR tooling (setup-time only, not runtime). Controller-only calibration is achievable (`--requiredtrackersforcal` accepts any device) but undocumented — official flows assume an HMD as the floor reference.
4. **Update rates: unverified everywhere.** No confirmed source states pose Hz, IMU Hz, or wrapper publish rates. Measure with `ros2 topic hz` on day one.

## Path 2 — SteamVR/OpenVR null-HMD: effectively dead for controllers

- `requireHmd: false` is the *officially intended* mechanism — a Valve engineer confirmed it in Jan 2017 and that it was broken; [openvr issue #383](https://github.com/ValveSoftware/openvr/issues/383) remains **open as of 2026**. Without an HMD, controllers don't even show as connected.
- The community null-driver recipe (`enableNullDriver` + `activateMultipleDrivers`) is unsupported by Valve, biased toward **trackers** rather than controllers in newer SteamVR builds, and has documented regressions (opentrack #1564: no poses; "No headset detected" popup blocking controller pairing). Evidence here ages badly — SteamVR keeps getting stricter.
- Even where it works, existing ROS wrappers (`vive_ros2` etc.) expose only pose + buttons.
- If it *did* work you'd gain SteamVR-exclusive extras (calibrated skeletal finger curl, battery, haptics via OpenVR API) — but do not build on this path.

## Path 3 — Monado/OpenXR: adds constraints, not channels

- Monado's 6DoF for Index hardware **comes from libsurvive underneath** (must be installed pre-build or Monado falls back to 3DoF). Runtime is Steam-free.
- But: no controller hotplugging (power-cycle a controller mid-session → restart the whole Monado session; confirmed current through mid-2026), and the documented calibration flow assumes an HMD (and optionally SteamVR import).
- For a ROS2 pipeline it's strictly an extra OpenXR layer on top of the same libsurvive data. **Skip.**

## Implications for the formation-control mapping node

- **Pose ingestion**: tf2 listener on `tracking_frame → {controller serial}`, or add a `PoseStamped` publisher to a fork of `component.cpp` (trivial — the pose is already in hand where `sendTransform()` is called).
- **Joy parsing**: axis indices follow the `SurviveAxis` enum (trigger=1 … thumbstick=10,11); buttons follow `SurviveButton` with touch flags shifted into upper bits. Pin exact layout on the bench (issue #11 axis-drop).
- **Clutch/deadman**: grip force (axis 8) with a threshold is now viable — contrary to the earlier assumption from stale issue #180. Trigger click or A/B remain simpler fallbacks.
- **Haptic feedback loop**: a fork exposing `survive_simple_object_haptic()` as a ROS2 subscriber enables collision-warning buzzes from safety_monitor — cheap and high-value for swarm teleop.
- **Battery**: same fork, publish `sensor_msgs/BatteryState` from `survive_simple_object_charge_percent()`.

## Day-one bench validation checklist

1. `survive-buttons` / `survive-cli`: confirm all 12 axes + touch flags stream from the knuckles over the dongle.
2. `ros2 topic hz` on `/libsurvive/imu`, `/libsurvive/joy`, `/tf` — record actual rates and Joy event latency.
3. Diff Joy axis count vs upstream events (issue #11 regression check).
4. Static jitter: log 60 s of stationary pose, compute stddev (targets: sub-mm good, >1 cm investigate calibration).
5. Haptic pulse via simple API over RF.
6. Attempt pairing without SteamVR (`lighthouse-console`/community tools) — currently an open question.

## Open questions (unresolved by any verified source)

1. Actual per-channel update rates on current hardware.
2. Whether current `libsurvive_ros2` forwards **all** 12 axes into `Joy.axes` for knuckles.
3. Whether dongle pairing/firmware can be done fully Steam-free.
4. Haptic-over-RF reliability headset-free (API exists; end-to-end unproven).

## Sources

- [libsurvive (collabora/cntools)](https://github.com/collabora/libsurvive) · [issue #180 (closed 2020-08-29)](https://github.com/collabora/libsurvive/issues/180) · [knuckles input commit 1ad0b16](https://github.com/collabora/libsurvive/commit/1ad0b16897c55121d98edc30619bb603332f2c53)
- [libsurvive_ros2](https://github.com/asymingt/libsurvive_ros2) · [issue #11 (knuckles Joy, axis drop)](https://github.com/asymingt/libsurvive_ros2/issues/11)
- Primary code reads: `driver_vive.c` (`RFKnuckles()`, `handle_battery()`, `survive_vive_send_haptic()`), `survive_types.h` (SurviveAxis/SurviveButton enums), `survive_api.h` (simple-API haptic/battery), `libsurvive_ros2/src/component.cpp` (publishers)
- [openvr issue #383 — requireHmd broken, open since 2017](https://github.com/ValveSoftware/openvr/issues/383) · [opentrack #1564 — null-driver regressions](https://github.com/opentrack/opentrack/issues/1564)
- [Monado Valve Index setup](https://monado.freedesktop.org/valve-index-setup.html) · [Monado libsurvive page](https://monado.freedesktop.org/libsurvive.html) · [Collabora: lighthouse tracking in Monado](https://www.collabora.com/news-and-blog/news-and-events/lighthouse-positional-tracking-in-monado-with-libsurvive.html)
- [NXP MR-B3RB libsurvive_ros2 deployment (topic surface)](https://nxp.gitbook.io/mr-b3rb/additional-b3rb-modules/vive-tracker-integration)
