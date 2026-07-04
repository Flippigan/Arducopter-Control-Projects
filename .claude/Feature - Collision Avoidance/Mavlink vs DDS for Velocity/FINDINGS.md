# MAVLink vs DDS: Velocity & Position Command Handling

## Summary

The ROS2/DDS `GlobalPosition.msg` is modeled after MAVLink's `SET_POSITION_TARGET_GLOBAL_INT`, but the DDS code path in ArduPilot has **incomplete implementation** — only position fields are processed. Velocity, acceleration, and yaw are silently ignored.

---

## GlobalPosition.msg Type Mask

Defined in `ardupilot1/Tools/ros2/ardupilot_msgs/msg/GlobalPosition.msg`:

```
uint16 type_mask
IGNORE_LATITUDE  = 1
IGNORE_LONGITUDE = 2
IGNORE_ALTITUDE  = 4
IGNORE_VX        = 8
IGNORE_VY        = 16
IGNORE_VZ        = 32
IGNORE_AFX       = 64
IGNORE_AFY       = 128
IGNORE_AFZ       = 256
FORCE            = 512
IGNORE_YAW       = 1024
IGNORE_YAW_RATE  = 2048
```

A `type_mask` of `0` means **no fields are ignored** — all values (including zeros) are treated as intentional targets.

---

## Code Path Comparison

### MAVLink Path (Full Implementation)

**File**: `ArduCopter/GCS_MAVLink_Copter.cpp:1393`
**Handler**: `handle_message_set_position_target_global_int()`

Parses every type_mask bit independently:

```cpp
bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
```

Dispatches to different guided-mode sub-controllers based on which field combinations are active (pos+vel, pos-only, vel-only, vel+accel, etc.). Full velocity, acceleration, and yaw processing.

### DDS/ROS2 Path (Position Only)

**File**: `libraries/AP_DDS/AP_DDS_ExternalControl.cpp:18`
**Handler**: `handle_global_position_control()`

Only processes position ignore flags:

```cpp
constexpr uint32_t MASK_POS_IGNORE =
    TYPE_MASK_IGNORE_LATITUDE |
    TYPE_MASK_IGNORE_LONGITUDE |
    TYPE_MASK_IGNORE_ALTITUDE;

if (!(cmd_pos.type_mask & MASK_POS_IGNORE)) {
    Location loc(...);
    external_control->set_global_position(loc);
}

// TODO add velocity and accel handling   <-- LINE 46
```

**Velocity, acceleration, yaw, and yaw_rate fields are completely ignored.** The code has a `TODO` comment where that logic should be.

---

## Behavior When type_mask = 0 (All Zeros)

| Field | MAVLink Path | DDS/ROS2 Path |
|-------|-------------|---------------|
| lat/lon/alt | Processed as position target | Processed as position target |
| velocity (0,0,0) | Treated as "target velocity = 0" (hover) | **Ignored entirely** |
| acceleration (0,0,0) | Treated as "target accel = 0" | **Ignored entirely** |
| yaw = 0.0 | Actively rotates to face North | **Ignored entirely** |
| yaw_rate = 0.0 | Treated as "stop rotating" | **Ignored entirely** |

---

## Available ROS2 Topics for Control

Since `GlobalPosition` is position-only through DDS, velocity control requires a separate topic:

| Topic | Message Type | Purpose |
|-------|-------------|---------|
| `/ap_N/cmd_gps_pose` | `ardupilot_msgs/GlobalPosition` | GPS position targets (lat/lon/alt only) |
| `/ap_N/cmd_vel` | `geometry_msgs/TwistStamped` | Velocity commands (linear + yaw rate) |

The `cmd_vel` handler (`AP_DDS_ExternalControl.cpp:54`) **is fully implemented** and supports both body-frame (`base_link`) and world-frame (`map`) inputs, with proper ENU-to-NED conversion.

---

## Implications for Formation Control

1. **Cannot combine position + velocity in a single DDS message** the way MAVLink can.
2. **Position commands** (`cmd_gps_pose`): ArduPilot uses its internal WPNAV/S-curve profiling to fly to the target. Velocity/acceleration shaping is handled internally.
3. **Velocity commands** (`cmd_vel`): Direct velocity setpoints for real-time control loops (e.g., collision avoidance reactive maneuvers).
4. **Switching between modes**: Sending `cmd_vel` and `cmd_gps_pose` alternately may cause control mode conflicts in GUIDED mode. Design the controller to use one or the other at any given time, not both simultaneously.
