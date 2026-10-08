# Hardware Implementation Summary — Operator Input Device

## Context
Deciding the physical control device a human operator will use to manipulate the
drone swarm formation in real time (entertainment/education). Requirement: hand/arm
motion **plus** physical joysticks, usable **independently** of any VR headset, and
compatible with the existing ROS2 + DDS stack. Operator input must eventually flow
**through collision avoidance** to the drones.

## Key finding — Oculus/Quest Touch is NOT viable
The Quest Touch controllers cannot work independently of the headset. Their 6DoF
**position is computed by the headset's cameras** (they carry IR LEDs; the headset
sees them). Remove the headset and only IMU orientation + buttons remain — and even
that requires the headset running + reverse-engineering (auto_oculus_touch via the
Meta runtime). Free-space arm position is impossible without the HMD. **Ruled out.**

## Recommended hardware — SteamVR "lighthouse" tracking (works headset-free)
Lighthouse = room-mounted IR laser base stations sweep the room; **sensors on the
tracked device** time the sweeps and compute their own 6DoF pose. Because the compute
lives in the device + room (not a headset), it runs with **no HMD in the loop**.

- **Valve Index controllers** — 6DoF + built-in thumbstick/trigger/grip in one
  handheld. Closest single-device match to "motion + physical joysticks."
- **2x SteamVR Base Stations 2.0** — the lighthouse emitters (room-mounted).
- **SteamVR USB dongle(s)** — pair controllers to the PC with no headset.
- Alt trackers: **Vive Tracker 3.0 / Tundra Tracker** + a separate gamepad for sticks.
- Desktop alternative / prototyping: **3Dconnexion SpaceMouse** — 6DoF puck + buttons,
  USB, zero room setup.

## Software / ingestion path (all native to ROS2 + DDS)
Universal pattern — device driver joins the ROS graph like any other node:
```
device driver -> ROS2 node -> sensor_msgs/Joy or geometry_msgs/PoseStamped (+ /tf)
   -> mapping node -> formation_msgs/FormationCommand (MOVE / SCALE / ROTATE)
   -> collision avoidance -> /ap_N/cmd_gps_pose -> drone
```
- **libsurvive_ros2** (recommended) — Steam-free, pure-Linux ROS2 driver for lighthouse
  hardware; publishes pose (/tf), IMU, button, and joystick data.
  github.com/asymingt/libsurvive_ros2  (also meco-group/ViveTrackerROS2)
- **Alt (OpenVR/SteamVR):** vive_ros2 / openvr_ros — needs SteamVR running; often split
  into a socket server/client (sometimes a Windows server). More friction.
- **SpaceMouse:** `spacenav` (ros-drivers/joystick_drivers) — publishes Twist + Joy.
- **Gamepad / cheap stand-in:** `joy` / `joy_linux` — publishes sensor_msgs/Joy.
- **Custom wireless (future):** micro-ROS over micro-XRCE-DDS — the same agent already
  running for ArduPilot.

## DDS compatibility — confirmed
ROS2 **is** DDS, so any driver node publishes on your existing bus automatically — no
bridging. Only standard hygiene to keep consistent across nodes: same `ROS_DOMAIN_ID`,
same rmw (Fast-DDS *or* Cyclone), matched QoS (best-effort + small depth for high-rate
teleop). `libsurvive_ros2` and `spacenav` are the cleanest fully-native paths.

## Integration seam (from the codebase)
- `formation_msgs/FormationCommand.msg` already defines **MOVE / SCALE / ROTATE**
  (`translation`, `target_center`, `target_scale`, `rotation_axis` + `rotation_angle`) —
  the ready-made operator message, currently un-wired to any node.
- Chosen operator scope: **whole-formation** manipulation (translate / rotate / scale the
  sphere as one rigid body).
- Designed injection point through collision avoidance:
  `/formation/targets/drone_N` (PoseStamped, ENU) -> CA controller -> `/ap_N/cmd_gps_pose`.
  (Note: the CA layer is currently un-launched / mid-redesign toward
  `/ap_N/geopose/filtered` as the common frame.)
