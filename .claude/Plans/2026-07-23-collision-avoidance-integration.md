# Collision Avoidance Integration & Algorithm Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the 4-drone swarm actually avoid collisions by wiring the never-executed avoidance stack into the command chain and fixing the five latent algorithm/frame bugs identified in `RESEARCH-AVOIDANCE-FAILURE-DIAGNOSIS.md`.

**Architecture:** The formation commander stops publishing GPS commands when the avoidance layer is active and instead hands off local ENU targets on `/formation/targets/drone_N`. Four `collision_avoidance_controller` instances (one per drone) consume those targets, run the Marek et al. 2025 motion-primitive algorithm in a **shared ENU frame** (all positions from `/ap_N/geopose/filtered`, anchored at drone 1's GPS origin), and output **velocity commands** on `/ap_N/cmd_vel`. A launch file starts the whole stack. The safety monitor keeps its emergency-stop authority; controllers defer to it.

**Tech Stack:** ROS2 Humble, Python 3.10, rclpy, ament_python, ArduPilot SITL DDS (`/ap_N/*` topics), numpy, pytest.

## Global Constraints

- **Spec:** `/home/finn/Documents/ardu_ws/src/formation_control/.claude/Feature - Collision Avoidance/RESEARCH-AVOIDANCE-FAILURE-DIAGNOSIS.md` — follow its Part 4 fix order.
- **d_min = 1.0 m** (USER DECISION 2026-07-10; was 3.0 m). Must land in the same pass as frame unification.
- **All-primitives-unsafe fallback = command a stop** (hold current position), never `p_safe` selection, never continued drift (USER DECISION 2026-07-10). Loud warning when it engages.
- **Common frame:** all inter-drone positions come from `/ap_N/geopose/filtered` (`geographic_msgs/GeoPoseStamped`), converted to a shared local ENU anchored at `/ap_{origin_instance}/gps_global_origin/filtered` (`origin_instance` parameter, default 1).
- **Channel ownership:** exactly one node commands `/ap_N/*` at a time. Commander owns it when `avoidance_active:=false` and during emergencies; controllers own it otherwise.
- **AP_DDS frame rules** (verified in `src/ardupilot/libraries/AP_DDS/AP_DDS_ExternalControl.cpp:54-80`): `cmd_vel` TwistStamped must have `header.frame_id = 'map'` → interpreted as ENU, converted to NED internally. `cmd_gps_pose` GlobalPosition **without** `header.frame_id = 'map'` is silently dropped by ArduPilot.
- **QoS:** all `/ap_N/*` subscriptions use BEST_EFFORT/VOLATILE/KEEP_LAST(10) (`sensor_qos` pattern already in each node).
- **4 drones, instances 1–4** (`/ap_1/`…`/ap_4/`).
- **Build gotcha (spec Part 3):** imports resolve to the *installed* copy. After every source edit and before any runtime test: `colcon build --packages-select formation_control && source install/setup.bash` from `/home/finn/Documents/ardu_ws`.
- **Test invocation:** from `/home/finn/Documents/ardu_ws/src/formation_control`, with ROS sourced (`source /opt/ros/humble/setup.bash && source /home/finn/Documents/ardu_ws/install/setup.bash`), run `python3 -m pytest tests/... -v`. `python3 -m pytest` puts the package root first on `sys.path`, so `formation_control.*` imports resolve to **source**, not install — this is what makes TDD possible here.
- **Existing test suite caveat:** `tests/unit/test_phase*.py` and `tests/integration/*` test *inline mock copies* of the algorithm classes, NOT the real modules. They will keep passing regardless of these changes. All NEW tests must import the real `formation_control.*` modules. Do not extend the mock-copy pattern.
- **Git:** `src/formation_control` is its own git repository. Do all work on branch `fix/avoidance-integration` (create in Task 1). All `git` commands below run from `/home/finn/Documents/ardu_ws/src/formation_control`.

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `formation_control/geo_utils.py` | **Create** | Single source of truth for GPS↔ENU conversion (pure functions, no ROS) |
| `formation_control/formation_commander.py` | Modify | Target handoff, `avoidance_active` gating, geopose positions, single-origin subscription |
| `formation_control/collision_avoidance_controller.py` | Modify | Geopose positions, `d_min=1.0`, cmd_vel output, emergency deference, velocity-feed warnings |
| `formation_control/collision_avoidance.py` | Modify | `d_min=1.0` defaults, per-primitive propagation (LATENT-4), stop fallback (LATENT-3), Eq. 2 cost |
| `formation_control/safety_monitor.py` | Modify | Geopose positions, thresholds consistent with `d_min=1.0` |
| `launch/collision_avoidance.launch.py` | **Create** | Starts commander (avoidance mode) + safety monitor + 4 controllers |
| `setup.py` | Modify | Install `launch/` directory |
| `package.xml` | Modify | Add missing `geographic_msgs`, `ardupilot_msgs` deps |
| `tests/unit/test_geo_utils.py` | **Create** | Conversion round-trip and axis-convention tests |
| `tests/unit/test_avoidance_wiring.py` | **Create** | Real-node tests: handoff, gating, frame conversion, cmd_vel output |
| `tests/unit/test_avoidance_algorithm_fixes.py` | **Create** | Real-module tests: propagation, stop fallback, Eq. 2 cost, d_min defaults |
| `tests/unit/test_safety_monitor_frames.py` | **Create** | Real-node tests: safety monitor common-frame distances |

Interface conventions used throughout (defined once, referenced by every task):

```python
# geo_utils (Task 1)
gps_to_enu(lat: float, lon: float, alt: float,
           ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray  # [east, north, up] m
enu_to_gps(x: float, y: float, z: float,
           ref_lat: float, ref_lon: float, ref_alt: float) -> tuple[float, float, float]  # (lat°, lon°, alt m)

# TrajectoryOptimizer after Task 7 (final form after Task 9 adds v_max):
TrajectoryOptimizer(d_min=1.0, urr=0.75, lambda_goal=1.0, t_horizon=2.0, dt_check=0.05, v_max=5.0)
optimize(primitives: List[MotionPrimitive], current_position: np.ndarray,
         current_velocity: np.ndarray,
         neighbor_states: Dict[int, Tuple[np.ndarray, np.ndarray]],  # id -> (position, velocity)
         target_position: np.ndarray) -> Optional[MotionPrimitive]   # None = all unsafe (after Task 8)
is_primitive_safe(primitive, current_position, current_velocity, neighbor_states) -> bool

# CollisionAvoidanceAlgorithm after Task 8:
algorithm.fallback_engaged: bool   # True when last compute_safe_waypoint hit the all-unsafe stop fallback
compute_safe_waypoint(...)         # signature unchanged: returns np.ndarray waypoint (== my_position on fallback)
```

The rclpy test fixture used by all node-level test files (repeat it in each new test file that needs it):

```python
import pytest
import rclpy

@pytest.fixture(scope='module')
def ros_init():
    """Initialize rclpy once per module; leave shutdown to process exit so
    other test modules sharing the process are unaffected."""
    if not rclpy.ok():
        rclpy.init()
    yield
```

---

### Task 1: Shared geodesy module `geo_utils.py`

Extract the GPS↔ENU math (currently duplicated in `formation_commander._local_to_gps` and `collision_avoidance_controller._local_to_gps`) into one pure module, and add the inverse (`gps_to_enu`) needed for LATENT-1. Axis convention per project memory: **x=East→longitude, y=North→latitude, z=Up**.

**Files:**
- Create: `formation_control/geo_utils.py`
- Test: `tests/unit/test_geo_utils.py`

**Interfaces:**
- Consumes: nothing (numpy only — no ROS imports, so these tests run without a sourced ROS env).
- Produces: `gps_to_enu(...) -> np.ndarray([east, north, up])`, `enu_to_gps(...) -> (lat, lon, alt)`, constant `EARTH_RADIUS = 6378137.0`. Tasks 4, 5, 6 import these.

- [ ] **Step 0: Create the working branch**

```bash
cd /home/finn/Documents/ardu_ws/src/formation_control
git checkout -b fix/avoidance-integration
```

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_geo_utils.py`:

```python
"""Tests for formation_control.geo_utils — shared GPS<->ENU conversion.

These import the REAL module (unlike tests/unit/test_phase*.py which test
inline copies).
"""
import math

import numpy as np
import pytest

from formation_control.geo_utils import EARTH_RADIUS, enu_to_gps, gps_to_enu

# ArduPilot SITL default home (CMAC, Canberra)
REF = (-35.363262, 149.165237, 584.0)


class TestGpsToEnu:
    def test_reference_point_maps_to_zero(self):
        enu = gps_to_enu(REF[0], REF[1], REF[2], *REF)
        np.testing.assert_allclose(enu, [0.0, 0.0, 0.0], atol=1e-9)

    def test_north_offset_maps_to_positive_y(self):
        enu = gps_to_enu(REF[0] + 0.001, REF[1], REF[2], *REF)
        expected_north = math.radians(0.001) * EARTH_RADIUS  # ~111.32 m
        assert enu[1] == pytest.approx(expected_north, abs=1e-6)
        assert enu[0] == pytest.approx(0.0, abs=1e-9)
        assert enu[2] == pytest.approx(0.0, abs=1e-9)

    def test_east_offset_maps_to_positive_x(self):
        enu = gps_to_enu(REF[0], REF[1] + 0.001, REF[2], *REF)
        expected_east = (math.radians(0.001) * EARTH_RADIUS
                         * math.cos(math.radians(REF[0])))
        assert enu[0] == pytest.approx(expected_east, abs=1e-6)
        assert enu[1] == pytest.approx(0.0, abs=1e-9)

    def test_altitude_offset_maps_to_z(self):
        enu = gps_to_enu(REF[0], REF[1], REF[2] + 12.5, *REF)
        assert enu[2] == pytest.approx(12.5)


class TestEnuToGps:
    def test_matches_legacy_local_to_gps_formula(self):
        """Must reproduce formation_commander._local_to_gps exactly
        (same small-angle equirectangular math)."""
        x, y, z = 30.0, -40.0, 15.0
        lat, lon, alt = enu_to_gps(x, y, z, *REF)
        d_lat = (y / EARTH_RADIUS) * (180.0 / math.pi)
        d_lon = (x / (EARTH_RADIUS
                      * math.cos(REF[0] * math.pi / 180.0))) * (180.0 / math.pi)
        assert lat == pytest.approx(REF[0] + d_lat, abs=1e-12)
        assert lon == pytest.approx(REF[1] + d_lon, abs=1e-12)
        assert alt == pytest.approx(REF[2] + z)


class TestRoundTrip:
    @pytest.mark.parametrize("point", [
        (10.0, 20.0, 5.0),
        (-3.7, 0.2, -1.0),
        (100.0, -250.0, 40.0),
        (0.0, 0.0, 0.0),
    ])
    def test_enu_gps_enu_round_trip(self, point):
        lat, lon, alt = enu_to_gps(*point, *REF)
        back = gps_to_enu(lat, lon, alt, *REF)
        np.testing.assert_allclose(back, point, atol=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/finn/Documents/ardu_ws/src/formation_control
python3 -m pytest tests/unit/test_geo_utils.py -v
```
Expected: FAIL with `ModuleNotFoundError: No module named 'formation_control.geo_utils'`

- [ ] **Step 3: Write the implementation**

Create `formation_control/geo_utils.py`:

```python
"""Shared GPS <-> local-ENU conversion for all formation_control nodes.

Single source of truth for the small-angle equirectangular conversion that
was previously duplicated in formation_commander and
collision_avoidance_controller. Axis convention: x = East (longitude),
y = North (latitude), z = Up. Pure functions, no ROS dependencies.
"""
import math

import numpy as np

EARTH_RADIUS = 6378137.0  # meters (WGS-84 equatorial)


def gps_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Convert GPS (deg, deg, m) to local ENU meters relative to a reference origin.

    Returns:
        np.ndarray [east, north, up]
    """
    north = math.radians(lat - ref_lat) * EARTH_RADIUS
    east = (math.radians(lon - ref_lon) * EARTH_RADIUS
            * math.cos(math.radians(ref_lat)))
    up = alt - ref_alt
    return np.array([east, north, up])


def enu_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """Convert local ENU meters to GPS (lat deg, lon deg, alt m). Inverse of gps_to_enu."""
    lat = ref_lat + math.degrees(y / EARTH_RADIUS)
    lon = ref_lon + math.degrees(
        x / (EARTH_RADIUS * math.cos(math.radians(ref_lat))))
    alt = ref_alt + z
    return lat, lon, alt
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_geo_utils.py -v
```
Expected: 10 passed

- [ ] **Step 5: Commit**

```bash
git add formation_control/geo_utils.py tests/unit/test_geo_utils.py
git commit -m "feat: add shared geo_utils module for GPS<->ENU conversion"
```

---

### Task 2: Formation commander target handoff + command-channel gating (BREAK-2, BREAK-3)

The commander gains an `avoidance_active` parameter. It **always** publishes each drone's local ENU target on `/formation/targets/drone_{N}` (the topic the controller already subscribes to at `collision_avoidance_controller.py:144-149`), and publishes direct `cmd_gps_pose` GPS commands **only when `avoidance_active` is false**. Emergency-hold commands remain unconditional — the commander stays the emergency authority in both modes (controllers learn to defer in Task 11).

**Files:**
- Modify: `formation_control/formation_commander.py` (init ~line 124, `_setup_drone_communications` ~line 188, `_formation_control_loop` ~line 331)
- Test: `tests/unit/test_avoidance_wiring.py`

**Interfaces:**
- Consumes: nothing new.
- Produces: `/formation/targets/drone_{N}` (`geometry_msgs/PoseStamped`, ENU meters in `pose.position`, QoS depth 10, `frame_id='map'`); parameter `avoidance_active` (bool, default `False`); instance attribute `self.avoidance_active`; dict `self.target_pubs: Dict[int, Publisher]`. Task 3's launch file sets `avoidance_active: True`.

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_avoidance_wiring.py`:

```python
"""Node-level wiring tests using the REAL nodes (rclpy, no spinning).

Publishers are replaced with MagicMock so publishes are captured without DDS.
Requires a sourced ROS environment (rclpy, ardupilot_msgs, geographic_msgs).
"""
from unittest.mock import MagicMock

import numpy as np
import pytest
import rclpy
from geographic_msgs.msg import GeoPointStamped

from formation_control.formation_commander import (
    FormationCommander,
    FormationState,
)


@pytest.fixture(scope='module')
def ros_init():
    """Initialize rclpy once per module; leave shutdown to process exit so
    other test modules sharing the process are unaffected."""
    if not rclpy.ok():
        rclpy.init()
    yield


REF = (-35.363262, 149.165237, 584.0)


def make_origin_msg(lat=REF[0], lon=REF[1], alt=REF[2]):
    msg = GeoPointStamped()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = alt
    return msg


def make_active_commander(avoidance_active):
    node = FormationCommander()
    node.avoidance_active = avoidance_active
    for iid in node.drone_command_pubs:
        node.drone_command_pubs[iid] = MagicMock()
    for iid in node.target_pubs:
        node.target_pubs[iid] = MagicMock()
    node._gps_origin_callback(make_origin_msg())
    node.formation_active = True
    node.formation_state = FormationState.ACTIVE
    return node


class TestTargetHandoff:
    def test_avoidance_active_parameter_defaults_false(self, ros_init):
        node = FormationCommander()
        assert node.avoidance_active is False
        node.destroy_node()

    def test_enu_targets_published_for_every_drone(self, ros_init):
        node = make_active_commander(avoidance_active=True)
        node._formation_control_loop()
        for iid in range(1, node.num_drones + 1):
            assert node.target_pubs[iid].publish.called, \
                f'/formation/targets/drone_{iid} never published (BREAK-2)'
            msg = node.target_pubs[iid].publish.call_args[0][0]
            expected = node.target_positions[iid]
            assert msg.pose.position.x == pytest.approx(expected[0])
            assert msg.pose.position.y == pytest.approx(expected[1])
            assert msg.pose.position.z == pytest.approx(expected[2])
        node.destroy_node()

    def test_targets_also_published_when_avoidance_inactive(self, ros_init):
        node = make_active_commander(avoidance_active=False)
        node._formation_control_loop()
        for iid in range(1, node.num_drones + 1):
            assert node.target_pubs[iid].publish.called
        node.destroy_node()


class TestCommandChannelGating:
    def test_cmd_gps_pose_suppressed_when_avoidance_active(self, ros_init):
        node = make_active_commander(avoidance_active=True)
        node._formation_control_loop()
        for iid in range(1, node.num_drones + 1):
            assert not node.drone_command_pubs[iid].publish.called, \
                f'cmd_gps_pose published for drone {iid} while avoidance owns the channel (BREAK-3)'
        node.destroy_node()

    def test_cmd_gps_pose_published_when_avoidance_inactive(self, ros_init):
        node = make_active_commander(avoidance_active=False)
        node._formation_control_loop()
        for iid in range(1, node.num_drones + 1):
            assert node.drone_command_pubs[iid].publish.called
        node.destroy_node()

    def test_emergency_hold_bypasses_gating(self, ros_init):
        """Commander remains the emergency authority even in avoidance mode."""
        node = make_active_commander(avoidance_active=True)
        node.drone_positions[1] = [1.0, 2.0, 20.0]
        node.emergency_drones = {1}
        node._send_emergency_hold_commands()
        assert node.drone_command_pubs[1].publish.called
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/finn/Documents/ardu_ws/src/formation_control
source /opt/ros/humble/setup.bash
source /home/finn/Documents/ardu_ws/install/setup.bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v
```
Expected: FAIL — `AttributeError: 'FormationCommander' object has no attribute 'avoidance_active'` (and no `target_pubs`).

- [ ] **Step 3: Implement in `formation_commander.py`**

3a. In `__init__`, after the existing `declare_parameter` block (after line `self.declare_parameter('formation_tolerance', 2.0)`):

```python
        # BREAK-3 fix: when true, the avoidance layer owns /ap_N/cmd_gps_pose;
        # this node publishes only /formation/targets/drone_N (plus emergency holds).
        self.declare_parameter('avoidance_active', False)
```

and after `self.formation_tolerance = self.get_parameter(...)`:

```python
        self.avoidance_active = self.get_parameter('avoidance_active').value
```

3b. In `__init__`, next to `self.drone_command_pubs = {}` (line 129), add:

```python
        # Local ENU target publishers (BREAK-2 fix: the collision avoidance
        # controller's only goal-position source)
        self.target_pubs = {}
```

3c. In `_setup_drone_communications`, inside the per-instance loop, right after the `cmd_topic` publisher creation:

```python
            # Local ENU target publisher consumed by collision_avoidance_controller
            target_topic = f'/formation/targets/drone_{instance_id}'
            self.target_pubs[instance_id] = self.create_publisher(
                PoseStamped, target_topic, 10)
```

3d. In `_formation_control_loop`, replace the command-sending block (lines 331-357, `for instance_id, target_pos in self.target_positions.items(): ...` through `self.drone_command_pubs[instance_id].publish(gps_cmd)`) with:

```python
        # Send target positions to all drones
        for instance_id, target_pos in self.target_positions.items():
            # Always publish the local ENU target for the avoidance layer (BREAK-2)
            if instance_id in self.target_pubs:
                target_msg = PoseStamped()
                target_msg.header.stamp = self.get_clock().now().to_msg()
                target_msg.header.frame_id = 'map'
                target_msg.pose.position.x = float(target_pos[0])
                target_msg.pose.position.y = float(target_pos[1])
                target_msg.pose.position.z = float(target_pos[2])
                target_msg.pose.orientation.w = 1.0
                self.target_pubs[instance_id].publish(target_msg)

            # Direct GPS commands only when the avoidance layer does NOT own
            # the channel (BREAK-3: interleaved direct commands cancel detours)
            if self.avoidance_active:
                continue

            if instance_id in self.drone_command_pubs:
                # Convert local ENU to GPS coordinates
                lat, lon, alt = self._local_to_gps(target_pos[0], target_pos[1], target_pos[2])

                # Create GlobalPosition command (format per ArduPilot ROS2 docs)
                gps_cmd = GlobalPosition()
                gps_cmd.header.stamp = self.get_clock().now().to_msg()
                gps_cmd.header.frame_id = "map"

                # Set GPS coordinates with coordinate_frame = 5 (as per ArduPilot documentation)
                gps_cmd.coordinate_frame = 5
                gps_cmd.latitude = lat
                gps_cmd.longitude = lon
                gps_cmd.altitude = alt

                # Debug: Log first command for each drone
                if not hasattr(self, '_logged_commands'):
                    self._logged_commands = set()
                if instance_id not in self._logged_commands:
                    self.get_logger().info(
                        f'UAV_{instance_id} GPS target: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m '
                        f'(local target: x={target_pos[0]:.2f}, y={target_pos[1]:.2f}, z={target_pos[2]:.2f})')
                    self._logged_commands.add(instance_id)

                self.drone_command_pubs[instance_id].publish(gps_cmd)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v
```
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add formation_control/formation_commander.py tests/unit/test_avoidance_wiring.py
git commit -m "feat: publish /formation/targets/drone_N and gate cmd_gps_pose on avoidance_active (BREAK-2/3)"
```

---

### Task 3: Launch file for the avoidance stack (BREAK-1)

Nothing ever started the avoidance nodes. Create `launch/collision_avoidance.launch.py` starting: formation_commander (with `avoidance_active: True`), safety_monitor, and 4 collision_avoidance_controller processes (instance IDs 1–4 via argv, matching `main()` at `collision_avoidance_controller.py:304`). Install the launch directory via `setup.py`. This launch file is run *in addition to* the sim launch and after takeoff (see Task 13 for the full sequence); `swarm_takeoff_commander` stays a manual step, as today.

**Files:**
- Create: `launch/collision_avoidance.launch.py`
- Modify: `setup.py`
- Modify: `package.xml` (declare the already-imported but undeclared deps)

**Interfaces:**
- Consumes: `avoidance_active` parameter from Task 2; existing console_scripts entry points in `setup.py`.
- Produces: `ros2 launch formation_control collision_avoidance.launch.py` — used by Task 13 runtime verification.

- [ ] **Step 1: Create the launch file**

Create `launch/collision_avoidance.launch.py`:

```python
"""Launch the full collision-avoidance stack (BREAK-1 fix).

Starts:
- formation_commander with avoidance_active:=True (publishes ENU targets only)
- safety_monitor (emergency-stop backstop, instances 1-4)
- 4x collision_avoidance_controller (one process per drone, instance ID via argv)

Run AFTER the simulation is up and drones have taken off:
    ros2 launch ardupilot_gz_bringup iris_multi_uav.launch.py
    ros2 run formation_control swarm_takeoff_commander
    ros2 launch formation_control collision_avoidance.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

NUM_DRONES = 4


def generate_launch_description():
    nodes = [
        Node(
            package='formation_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{'instance_ids': list(range(1, NUM_DRONES + 1))}],
        ),
        Node(
            package='formation_control',
            executable='formation_commander',
            name='formation_commander',
            output='screen',
            parameters=[{
                'num_drones': NUM_DRONES,
                'avoidance_active': True,
            }],
        ),
    ]

    for instance_id in range(1, NUM_DRONES + 1):
        nodes.append(Node(
            package='formation_control',
            executable='collision_avoidance_controller',
            name=f'collision_avoidance_controller_{instance_id}',
            output='screen',
            # main() reads the instance ID from argv[1]
            arguments=[str(instance_id)],
        ))

    return LaunchDescription(nodes)
```

- [ ] **Step 2: Install the launch directory**

Replace the full contents of `setup.py` with:

```python
import os
from glob import glob

from setuptools import setup

package_name = 'formation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='finn',
    maintainer_email='finnpicotoli@gmail.com',
    description='Multi-drone 3D formation control system with centralized safety monitoring',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'formation_commander = formation_control.formation_commander:main',
            'safety_monitor = formation_control.safety_monitor:main',
            'drone_interface = formation_control.drone_interface:main',
            'formation_visualizer = formation_control.formation_visualizer:main',
            'swarm_takeoff_commander = formation_control.swarm_takeoff_commander:main',
            'collision_avoidance_controller = formation_control.collision_avoidance_controller:main',
        ],
    },
)
```

- [ ] **Step 3: Declare missing dependencies in `package.xml`**

The code already imports `geographic_msgs` and `ardupilot_msgs` but never declared them. In `package.xml`, after `<depend>visualization_msgs</depend>` add:

```xml
  <depend>geographic_msgs</depend>
  <depend>ardupilot_msgs</depend>
```

- [ ] **Step 4: Build and verify installation**

```bash
cd /home/finn/Documents/ardu_ws
colcon build --packages-select formation_control
source install/setup.bash
ls install/formation_control/share/formation_control/launch/
```
Expected: `collision_avoidance.launch.py` listed.

- [ ] **Step 5: Smoke-test the launch file (no sim needed)**

```bash
timeout 10 ros2 launch formation_control collision_avoidance.launch.py &
sleep 6 && ros2 node list
```
Expected: `ros2 node list` shows `/safety_monitor`, `/formation_commander`, and `/collision_avoidance_controller_1` … `_4` (6 nodes). Nodes idle safely without a sim: the commander defers formation start until a GPS origin arrives, controllers skip their loop at Step 0. Wait for the `timeout` to reap the launch before proceeding.

- [ ] **Step 6: Commit**

```bash
cd /home/finn/Documents/ardu_ws/src/formation_control
git add launch/collision_avoidance.launch.py setup.py package.xml
git commit -m "feat: add collision_avoidance launch file starting full stack (BREAK-1)"
```

---

### Task 4: Formation commander common frame (LATENT-1, commander half)

Switch the commander's position inputs from per-drone-origin `/ap_N/pose/filtered` to global `/ap_N/geopose/filtered`, converted into the shared ENU frame. Fix the origin race (`_gps_origin_callback` currently subscribed to **all four** drones, keeping whichever arrives first) by subscribing only to `/ap_{origin_instance}/gps_global_origin/filtered`. Delegate `_local_to_gps` to `geo_utils`.

**Files:**
- Modify: `formation_control/formation_commander.py` (imports ~line 50, `__init__` params ~line 97, `_setup_drone_communications` lines 194-207, `_drone_position_callback` line 227, `_local_to_gps` lines 264-295)
- Test: `tests/unit/test_avoidance_wiring.py` (append)

**Interfaces:**
- Consumes: `gps_to_enu`, `enu_to_gps` from Task 1.
- Produces: `self.drone_positions[iid]` now holds **shared-frame** ENU `[e, n, u]` lists (same type as before, different frame); parameter `origin_instance` (int, default 1); callback `_drone_geopose_callback(msg: GeoPoseStamped, instance_id: int)`. Tasks 5/6 apply the identical pattern to the other two nodes.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_wiring.py`:

```python
from geographic_msgs.msg import GeoPoseStamped

from formation_control.geo_utils import EARTH_RADIUS
import math


def make_geopose_msg(lat, lon, alt):
    msg = GeoPoseStamped()
    msg.pose.position.latitude = lat
    msg.pose.position.longitude = lon
    msg.pose.position.altitude = alt
    return msg


class TestCommanderCommonFrame:
    def test_geopose_converted_to_shared_enu(self, ros_init):
        node = FormationCommander()
        node._gps_origin_callback(make_origin_msg())
        # Drone 2 reports a geopose 0.001 deg north of the shared origin, 10 m up
        node._drone_geopose_callback(
            make_geopose_msg(REF[0] + 0.001, REF[1], REF[2] + 10.0), 2)
        expected_north = math.radians(0.001) * EARTH_RADIUS
        pos = node.drone_positions[2]
        assert pos[0] == pytest.approx(0.0, abs=1e-6)
        assert pos[1] == pytest.approx(expected_north, abs=1e-3)
        assert pos[2] == pytest.approx(10.0, abs=1e-6)
        node.destroy_node()

    def test_geopose_before_origin_is_not_stored(self, ros_init):
        """Without the shared origin there is no shared frame yet."""
        node = FormationCommander()
        node._drone_geopose_callback(make_geopose_msg(*REF), 1)
        assert 1 not in node.drone_positions
        node.destroy_node()

    def test_single_origin_subscription_no_race(self, ros_init):
        """Origin must come only from /ap_{origin_instance}/... (default 1)."""
        node = FormationCommander()
        topics = [s.topic_name for s in node.subscriptions]
        origin_topics = [t for t in topics if 'gps_global_origin' in t]
        assert origin_topics == ['/ap_1/gps_global_origin/filtered']
        node.destroy_node()

    def test_positions_use_geopose_not_local_pose(self, ros_init):
        node = FormationCommander()
        topics = [s.topic_name for s in node.subscriptions]
        assert not any('pose/filtered' in t and 'geopose' not in t for t in topics), \
            'commander still subscribes to per-drone-origin /ap_N/pose/filtered (LATENT-1)'
        assert sum('geopose/filtered' in t for t in topics) == node.num_drones
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v -k CommanderCommonFrame
```
Expected: 4 FAIL (`AttributeError: _drone_geopose_callback`, origin topic list has 4 entries, `pose/filtered` subscriptions present).

- [ ] **Step 3: Implement in `formation_commander.py`**

3a. Update imports (line 50 area):

```python
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
```

and below the `formation_math` import:

```python
from formation_control.geo_utils import enu_to_gps, gps_to_enu
```

3b. In `__init__`, add the parameter next to `avoidance_active` (Task 2):

```python
        # Which drone's EKF origin anchors the shared ENU frame (LATENT-1)
        self.declare_parameter('origin_instance', 1)
```
```python
        self.origin_instance = self.get_parameter('origin_instance').value
```

3c. In `_setup_drone_communications`, **before** the per-instance loop, add the single origin subscription (and delete the per-instance `gps_origin_topic` subscription block, lines 201-207):

```python
        # Shared ENU origin: subscribe to ONE drone's EKF origin only.
        # (Previously all four origins raced into one callback; first-arrival won.)
        origin_topic = f'/ap_{self.origin_instance}/gps_global_origin/filtered'
        self.gps_origin_sub = self.create_subscription(
            GeoPointStamped, origin_topic,
            self._gps_origin_callback, qos_profile=sensor_qos)
        self.get_logger().info(f'Shared ENU origin source: {origin_topic}')
```

3d. In the per-instance loop, replace the position subscription block (lines 194-199) with:

```python
            # Global pose subscriber: all drones share this frame (LATENT-1 fix)
            pos_topic = f'/ap_{instance_id}/geopose/filtered'
            self.drone_position_subs[instance_id] = self.create_subscription(
                GeoPoseStamped, pos_topic,
                lambda msg, iid=instance_id: self._drone_geopose_callback(msg, iid),
                qos_profile=sensor_qos)
```

3e. Replace `_drone_position_callback` (lines 227-230) with:

```python
    def _drone_geopose_callback(self, msg, instance_id):
        """Store drone position in the shared ENU frame (anchored at origin_instance).

        Skips storage until the shared origin arrives - a position without the
        shared origin cannot be placed in the common frame.
        """
        if not self.gps_origin_set:
            return
        enu = gps_to_enu(
            msg.pose.position.latitude,
            msg.pose.position.longitude,
            msg.pose.position.altitude,
            self.gps_origin_lat, self.gps_origin_lon, self.gps_origin_alt)
        self.drone_positions[instance_id] = enu.tolist()
```

3f. Replace the body of `_local_to_gps` (keep the docstring and the `RuntimeError` guard, delete the manual math, lines 281-295):

```python
        if not self.gps_origin_set or self.gps_origin_lat is None:
            raise RuntimeError(
                'GPS origin not set - cannot convert local to GPS coordinates. '
                'Wait for GPS origin from ArduPilot before calling _local_to_gps().')

        return enu_to_gps(x, y, z,
                          self.gps_origin_lat, self.gps_origin_lon, self.gps_origin_alt)
```

- [ ] **Step 4: Run the full wiring test file (old + new tests must pass)**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v
```
Expected: 10 passed

- [ ] **Step 5: Commit**

```bash
git add formation_control/formation_commander.py tests/unit/test_avoidance_wiring.py
git commit -m "fix: commander uses shared ENU frame from geopose + single-origin subscription (LATENT-1)"
```

---

### Task 5: Controller common frame + d_min=1.0 (LATENT-1 controller half + Part 4 step 4)

Same geopose/shared-origin pattern in the controller, plus the `d_min` change from 3.0 → 1.0 **in the same pass** (spec requirement: a 1 m `d_min` is meaningless while frames disagree by up to 4 m, and a 3 m `d_min` with unified frames would hover-lock the 2.8 m pentagon spawn). Also fix the controller's `GlobalPosition` commands, which are currently **silently dropped by ArduPilot** because `header.frame_id` is never set (`AP_DDS_ExternalControl.cpp` requires `"map"`).

**Files:**
- Modify: `formation_control/collision_avoidance_controller.py` (imports line 26, params line 61, `_setup_subscriptions` lines 126-157, `_position_callback` lines 175-185, `_local_to_gps` lines 219-240, `_avoidance_control_loop` lines 282-288)
- Modify: `formation_control/collision_avoidance.py` (three `d_min: float = 3.0` defaults: lines 171, 289, 479)
- Test: `tests/unit/test_avoidance_wiring.py` (append), `tests/unit/test_avoidance_algorithm_fixes.py` (create)

**Interfaces:**
- Consumes: `gps_to_enu`, `enu_to_gps` (Task 1).
- Produces: controller state `self.drone_positions` / `self.my_position` in shared ENU; parameter `origin_instance` (default 1); callback `_geopose_callback(msg, instance_id)`; all `d_min` defaults = 1.0. Tasks 7-10 build on this module state.

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_avoidance_algorithm_fixes.py`:

```python
"""Tests for the REAL formation_control.collision_avoidance module.

(The legacy tests/unit/test_phase1_core_algorithms.py tests inline copies of
these classes; this file tests the actual shipped code.)
"""
import numpy as np
import pytest

from formation_control.collision_avoidance import (
    CollisionAvoidanceAlgorithm,
    CollisionPredictor,
    MotionPrimitiveGenerator,
    TrajectoryOptimizer,
)


class TestDminDefaults:
    """USER DECISION 2026-07-10: d_min = 1.0 m (below the 2.8-4 m pentagon
    spawn spacing, so takeoff cannot hover-lock)."""

    def test_collision_predictor_default(self):
        assert CollisionPredictor().d_min == 1.0

    def test_trajectory_optimizer_default(self):
        assert TrajectoryOptimizer().d_min == 1.0

    def test_algorithm_default(self):
        assert CollisionAvoidanceAlgorithm().d_min == 1.0
```

Append to `tests/unit/test_avoidance_wiring.py`:

```python
from formation_control.collision_avoidance_controller import (
    CollisionAvoidanceController,
)


class TestControllerCommonFrame:
    def test_d_min_parameter_default_is_one_meter(self, ros_init):
        node = CollisionAvoidanceController(instance_id=1)
        assert node.d_min == pytest.approx(1.0)
        node.destroy_node()

    def test_geopose_converted_to_shared_enu(self, ros_init):
        node = CollisionAvoidanceController(instance_id=1)
        node._gps_origin_callback(make_origin_msg())
        node._geopose_callback(
            make_geopose_msg(REF[0] + 0.001, REF[1], REF[2] + 10.0), 1)
        expected_north = math.radians(0.001) * EARTH_RADIUS
        assert node.my_position[1] == pytest.approx(expected_north, abs=1e-3)
        assert node.my_position[2] == pytest.approx(10.0, abs=1e-6)
        assert node.drone_positions[1][1] == pytest.approx(expected_north, abs=1e-3)
        node.destroy_node()

    def test_geopose_before_origin_is_skipped(self, ros_init):
        node = CollisionAvoidanceController(instance_id=1)
        node._geopose_callback(make_geopose_msg(*REF), 1)
        assert node.my_position is None
        node.destroy_node()

    def test_subscribes_geopose_and_single_origin(self, ros_init):
        node = CollisionAvoidanceController(instance_id=2)
        topics = [s.topic_name for s in node.subscriptions]
        assert sum('geopose/filtered' in t for t in topics) == node.num_drones
        assert not any('pose/filtered' in t and 'geopose' not in t for t in topics)
        assert [t for t in topics if 'gps_global_origin' in t] == \
            ['/ap_1/gps_global_origin/filtered']
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py tests/unit/test_avoidance_wiring.py -v -k "Dmin or ControllerCommonFrame"
```
Expected: FAIL — d_min defaults are 3.0; `_geopose_callback` missing; origin topic is `/ap_2/...` for instance 2.

- [ ] **Step 3: Implement**

3a. `collision_avoidance.py`: change the three defaults (and their docstring mentions) — `CollisionPredictor.__init__` line 171, `TrajectoryOptimizer.__init__` line 289, `CollisionAvoidanceAlgorithm.__init__` line 479:

```python
    def __init__(self, d_min: float = 1.0, t_horizon: float = 2.0, dt: float = 0.05):
```
```python
    def __init__(self, d_min: float = 1.0, urr: float = 0.75, lambda_goal: float = 1.0):
```
```python
        d_min: float = 1.0,
```

3b. `collision_avoidance_controller.py` imports (line 26):

```python
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
```
and after the `collision_avoidance` import:
```python
from formation_control.geo_utils import enu_to_gps, gps_to_enu
```

3c. Parameters (line 61 and after line 74):

```python
        self.declare_parameter('d_min', 1.0)
```
```python
        # Which drone's EKF origin anchors the shared ENU frame (LATENT-1)
        self.declare_parameter('origin_instance', 1)
        self.origin_instance = self.get_parameter('origin_instance').value
```

3d. `_setup_subscriptions`: replace the `PoseStamped` position loop (lines 126-132) with:

```python
        # Global pose subscriptions: all drones in one frame (LATENT-1 fix)
        for i in range(1, self.num_drones + 1):
            self.create_subscription(
                GeoPoseStamped,
                f'/ap_{i}/geopose/filtered',
                lambda msg, iid=i: self._geopose_callback(msg, iid),
                self.sensor_qos
            )
```

and replace the GPS-origin subscription (lines 151-157) with:

```python
        # Shared ENU origin (single source - same origin_instance for every node)
        self.create_subscription(
            GeoPointStamped,
            f'/ap_{self.origin_instance}/gps_global_origin/filtered',
            self._gps_origin_callback,
            self.sensor_qos
        )
```

3e. Replace `_position_callback` (lines 175-185) with:

```python
    def _geopose_callback(self, msg: GeoPoseStamped, instance_id: int):
        """Convert a drone's global pose into the shared ENU frame."""
        if not self.gps_origin_set:
            return  # no shared frame until the origin arrives
        pos = gps_to_enu(
            msg.pose.position.latitude,
            msg.pose.position.longitude,
            msg.pose.position.altitude,
            self.gps_origin_lat, self.gps_origin_lon, self.gps_origin_alt)
        self.drone_positions[instance_id] = pos

        if instance_id == self.instance_id:
            self.my_position = pos
```

3f. Replace the body of `_local_to_gps` (lines 231-240) keeping the guard:

```python
        return enu_to_gps(x, y, z,
                          self.gps_origin_lat, self.gps_origin_lon, self.gps_origin_alt)
```

3g. In `_avoidance_control_loop` Step 5 (lines 282-288), set the header so ArduPilot stops dropping the command (interim fix until Task 10 replaces this with cmd_vel):

```python
        # Step 5: Publish GPS command
        gps_cmd = GlobalPosition()
        gps_cmd.header.stamp = self.get_clock().now().to_msg()
        gps_cmd.header.frame_id = 'map'  # required: AP_DDS drops frameless commands
        gps_cmd.latitude = lat
        gps_cmd.longitude = lon
        gps_cmd.altitude = alt
        gps_cmd.coordinate_frame = 5
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py tests/unit/test_avoidance_wiring.py -v
```
Expected: all pass (3 + 14)

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance_controller.py formation_control/collision_avoidance.py \
        tests/unit/test_avoidance_algorithm_fixes.py tests/unit/test_avoidance_wiring.py
git commit -m "fix: controller shared ENU frame, d_min=1.0, GlobalPosition frame_id (LATENT-1)"
```

---

### Task 6: Safety monitor common frame + consistent thresholds (LATENT-1 monitor half)

The safety monitor has the same cross-frame distance flaw. Apply the geopose/shared-origin pattern and align thresholds with `d_min = 1.0`: `safety_radius` 3.0 → **1.0** (emergency trigger), `warning_radius` 6.0 → **3.0**.

**Files:**
- Modify: `formation_control/safety_monitor.py` (imports lines 25-29, defaults lines 45-46 and 74-75, `_setup_drone_monitoring` lines 101-109, `_drone_position_callback` lines 133-152)
- Test: `tests/unit/test_safety_monitor_frames.py`

**Interfaces:**
- Consumes: `gps_to_enu` (Task 1).
- Produces: `self.drone_positions` in shared ENU; parameter `origin_instance` (default 1); attributes `gps_origin_lat/lon/alt`, `gps_origin_set`, callbacks `_gps_origin_callback`, `_drone_geopose_callback`.

- [ ] **Step 1: Write the failing tests**

Create `tests/unit/test_safety_monitor_frames.py`:

```python
"""Safety monitor common-frame tests against the REAL node."""
import math

import pytest
import rclpy
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped

from formation_control.safety_monitor import SafetyMonitor


@pytest.fixture(scope='module')
def ros_init():
    """Initialize rclpy once per module; leave shutdown to process exit so
    other test modules sharing the process are unaffected."""
    if not rclpy.ok():
        rclpy.init()
    yield


REF = (-35.363262, 149.165237, 584.0)


def make_origin_msg():
    msg = GeoPointStamped()
    msg.position.latitude = REF[0]
    msg.position.longitude = REF[1]
    msg.position.altitude = REF[2]
    return msg


def make_geopose_msg(lat, lon, alt):
    msg = GeoPoseStamped()
    msg.pose.position.latitude = lat
    msg.pose.position.longitude = lon
    msg.pose.position.altitude = alt
    return msg


def offset_north(meters):
    """GPS latitude offset for a given north displacement."""
    return REF[0] + math.degrees(meters / 6378137.0)


class TestThresholds:
    def test_defaults_consistent_with_dmin_one_meter(self, ros_init):
        node = SafetyMonitor()
        assert node.safety_radius == pytest.approx(1.0)
        assert node.warning_radius == pytest.approx(3.0)
        node.destroy_node()


class TestCommonFrameDistances:
    def _monitor_with_two_drones(self, separation_m):
        node = SafetyMonitor()
        node._gps_origin_callback(make_origin_msg())
        node._drone_geopose_callback(
            make_geopose_msg(REF[0], REF[1], REF[2] + 20.0), 1)
        node._drone_geopose_callback(
            make_geopose_msg(offset_north(separation_m), REF[1], REF[2] + 20.0), 2)
        return node

    def test_half_meter_separation_is_critical(self, ros_init):
        node = self._monitor_with_two_drones(0.5)
        pairs = node._detect_collisions()
        assert pairs, 'physically-touching drones not detected (LATENT-1)'
        assert max(risk for _, _, risk in pairs) == 3
        node.destroy_node()

    def test_two_meter_separation_is_not_critical(self, ros_init):
        """2 m > d_min=1.0: must NOT fire an emergency (would hover-lock the
        2.8-4 m pentagon spawn)."""
        node = self._monitor_with_two_drones(2.0)
        pairs = node._detect_collisions()
        assert all(risk < 3 for _, _, risk in pairs)
        node.destroy_node()

    def test_geopose_before_origin_is_skipped(self, ros_init):
        node = SafetyMonitor()
        node._drone_geopose_callback(make_geopose_msg(*REF), 1)
        assert 1 not in node.drone_positions
        node.destroy_node()

    def test_subscribes_geopose_not_local_pose(self, ros_init):
        node = SafetyMonitor()
        topics = [s.topic_name for s in node.subscriptions]
        assert sum('geopose/filtered' in t for t in topics) == node.num_drones
        assert not any('pose/filtered' in t and 'geopose' not in t for t in topics)
        assert [t for t in topics if 'gps_global_origin' in t] == \
            ['/ap_1/gps_global_origin/filtered']
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_safety_monitor_frames.py -v
```
Expected: FAIL — thresholds 3.0/6.0, `_gps_origin_callback`/`_drone_geopose_callback` missing.

- [ ] **Step 3: Implement in `safety_monitor.py`**

3a. Imports — replace line 27 block additions:

```python
from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
from ardupilot_msgs.msg import Status
```
and after the other imports:
```python
from formation_control.geo_utils import gps_to_enu
```

3b. Defaults — lines 45-46 and the matching `declare_parameter` lines 74-75:

```python
        self.safety_radius = 1.0  # meters - minimum safe distance (= d_min)
        self.warning_radius = 3.0  # meters - collision warning distance
```
```python
        self.declare_parameter('safety_radius', 1.0)
        self.declare_parameter('warning_radius', 3.0)
        self.declare_parameter('origin_instance', 1)
```
and with the other parameter reads:
```python
        self.origin_instance = self.get_parameter('origin_instance').value
```

3c. In `__init__` before `_setup_drone_monitoring()` add the origin state:

```python
        # Shared ENU frame origin (LATENT-1)
        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self.gps_origin_alt = None
        self.gps_origin_set = False
```

3d. In `_setup_drone_monitoring`, before the loop:

```python
        # Shared ENU origin: one source for every node in the system
        self.create_subscription(
            GeoPointStamped,
            f'/ap_{self.origin_instance}/gps_global_origin/filtered',
            self._gps_origin_callback,
            self.sensor_qos)
```

and replace the pose subscription (lines 102-109) with:

```python
            # Subscribe to global pose (shared frame for all drones, LATENT-1 fix)
            pose_topic = f'/ap_{instance_id}/geopose/filtered'
            self.create_subscription(
                GeoPoseStamped,
                pose_topic,
                lambda msg, iid=instance_id: self._drone_geopose_callback(msg, iid),
                self.sensor_qos
            )
```

3e. Replace `_drone_position_callback` (lines 133-152) with:

```python
    def _gps_origin_callback(self, msg):
        """Latch the shared ENU origin."""
        if not self.gps_origin_set:
            self.gps_origin_lat = msg.position.latitude
            self.gps_origin_lon = msg.position.longitude
            self.gps_origin_alt = msg.position.altitude
            self.gps_origin_set = True
            self.get_logger().info(
                f'Shared ENU origin set: lat={self.gps_origin_lat:.7f}, '
                f'lon={self.gps_origin_lon:.7f}, alt={self.gps_origin_alt:.2f}')

    def _drone_geopose_callback(self, msg, instance_id):
        """Callback for drone global pose - stored in the shared ENU frame."""
        if not self.gps_origin_set:
            return
        enu = gps_to_enu(
            msg.pose.position.latitude,
            msg.pose.position.longitude,
            msg.pose.position.altitude,
            self.gps_origin_lat, self.gps_origin_lon, self.gps_origin_alt)
        self.drone_positions[instance_id] = enu.tolist()
        self.drone_last_update[instance_id] = self.get_clock().now()
```
(The per-drone debug counters may be dropped — the origin log line replaces them.)

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_safety_monitor_frames.py -v
```
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add formation_control/safety_monitor.py tests/unit/test_safety_monitor_frames.py
git commit -m "fix: safety monitor shared ENU frame + thresholds for d_min=1.0 (LATENT-1)"
```

---

### Task 7: Per-primitive trajectory propagation (LATENT-4)

The current safety test judges primitives by endpoint-distance to `p_safe` (a stopping point along the *current* velocity — not an obstacle), so a primitive accelerating **directly at a neighbor** can pass. Implement the Section 3.4 inequality literally: for each candidate primitive, march `t_iter` from 0 to `t_horizon` in `dt_check` steps and reject the primitive if `‖p_i(t+t_iter) − p_j(t+t_iter)‖ < d_min` for **any** neighbor at **any** step. My trajectory: kinematic under the primitive's acceleration for `[0, dt]`, then constant `final_velocity`. Neighbors: constant current velocity.

`optimize()` changes signature (it now needs my velocity and neighbor velocities); `compute_safe_waypoint` rewires to match. `CollisionPredictor` stays in the module (exported, still constructed) but the pipeline no longer uses its predictions for primitive rejection.

**Files:**
- Modify: `formation_control/collision_avoidance.py` (`TrajectoryOptimizer.__init__` line 289, `optimize` lines 303-366, `CollisionAvoidanceAlgorithm.__init__` line 499-501, `compute_safe_waypoint` lines 526-556)
- Test: `tests/unit/test_avoidance_algorithm_fixes.py` (append)

**Interfaces:**
- Consumes: `MotionPrimitive`, `MotionPrimitiveGenerator` (unchanged).
- Produces:
  - `TrajectoryOptimizer(d_min=1.0, urr=0.75, lambda_goal=1.0, t_horizon=2.0, dt_check=0.05)`
  - `is_primitive_safe(primitive, current_position, current_velocity, neighbor_states) -> bool` where `neighbor_states: Dict[int, Tuple[np.ndarray, np.ndarray]]` maps id → (position, velocity)
  - `optimize(primitives, current_position, current_velocity, neighbor_states, target_position) -> Optional[MotionPrimitive]` (still returns the hover primitive when all are unsafe — Task 8 changes that to `None`)
  - Tasks 8 and 9 edit these same methods; their tests assume this signature.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_algorithm_fixes.py`:

```python
HOVER_ALT = 20.0


def hover_primitive(primitives):
    return next(p for p in primitives if np.allclose(p.acceleration, [0, 0, 0]))


def plus_x_primitive(primitives):
    return next(p for p in primitives
                if p.acceleration[0] > 0 and p.acceleration[1] == 0
                and p.acceleration[2] == 0)


class TestIsPrimitiveSafe:
    """Section 3.4 inequality: reject if ||p_i(t) - p_j(t)|| < d_min anywhere
    on the horizon."""

    def setup_method(self):
        self.gen = MotionPrimitiveGenerator(v_max=5.0, a_max=2.0, dt=0.5)
        self.opt = TrajectoryOptimizer(d_min=1.0, t_horizon=2.0, dt_check=0.05)

    def test_hover_into_head_on_neighbor_is_unsafe(self):
        """Hovering while a neighbor closes at 2 m/s from 4 m away breaches
        d_min at t=1.5s - AFTER the 0.5s primitive window. Catching this
        requires propagation past the primitive end."""
        primitives = self.gen.generate_primitives(np.zeros(3))
        neighbor = {2: (np.array([4.0, 0.0, HOVER_ALT]),
                        np.array([-2.0, 0.0, 0.0]))}
        assert not self.opt.is_primitive_safe(
            hover_primitive(primitives),
            np.array([0.0, 0.0, HOVER_ALT]), np.zeros(3), neighbor)

    def test_retreat_from_closing_neighbor_is_safe(self):
        primitives = self.gen.generate_primitives(np.zeros(3))
        minus_x = next(p for p in primitives if p.acceleration[0] < 0)
        neighbor = {2: (np.array([4.0, 0.0, HOVER_ALT]),
                        np.array([-2.0, 0.0, 0.0]))}
        assert self.opt.is_primitive_safe(
            minus_x, np.array([0.0, 0.0, HOVER_ALT]), np.zeros(3), neighbor)

    def test_parallel_neighbor_at_two_meters_is_safe(self):
        primitives = self.gen.generate_primitives(np.zeros(3))
        neighbor = {2: (np.array([0.0, 2.0, HOVER_ALT]), np.zeros(3))}
        assert self.opt.is_primitive_safe(
            hover_primitive(primitives),
            np.array([0.0, 0.0, HOVER_ALT]), np.zeros(3), neighbor)


class TestOptimizePropagation:
    """LATENT-4 core failure: a primitive accelerating directly at a
    stationary neighbor used to pass the old endpoint-vs-p_safe test
    because zero current velocity produced no collision prediction."""

    def setup_method(self):
        self.gen = MotionPrimitiveGenerator(v_max=5.0, a_max=2.0, dt=0.5)
        self.opt = TrajectoryOptimizer(d_min=1.0, t_horizon=2.0, dt_check=0.05)
        self.my_pos = np.array([0.0, 0.0, HOVER_ALT])
        # Stationary neighbor 1.6 m ahead on the path to the goal
        self.neighbors = {2: (np.array([1.6, 0.0, HOVER_ALT]), np.zeros(3))}
        self.target = np.array([10.0, 0.0, HOVER_ALT])

    def test_never_selects_accelerate_at_neighbor(self):
        primitives = self.gen.generate_primitives(np.zeros(3))
        best = self.opt.optimize(
            primitives, self.my_pos, np.zeros(3), self.neighbors, self.target)
        assert best is not None
        assert not np.allclose(best.acceleration,
                               plus_x_primitive(primitives).acceleration), \
            'optimizer chose the primitive that flies straight at the neighbor'

    def test_safe_escape_still_available(self):
        primitives = self.gen.generate_primitives(np.zeros(3))
        best = self.opt.optimize(
            primitives, self.my_pos, np.zeros(3), self.neighbors, self.target)
        assert best is not None  # only +X is rejected; six primitives remain
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v -k "IsPrimitiveSafe or OptimizePropagation"
```
Expected: FAIL — `TypeError` (no `t_horizon`/`dt_check` kwargs, no `is_primitive_safe`, `optimize()` argument mismatch).

- [ ] **Step 3: Implement in `collision_avoidance.py`**

3a. `TrajectoryOptimizer.__init__` (line 289) becomes:

```python
    def __init__(self, d_min: float = 1.0, urr: float = 0.75,
                 lambda_goal: float = 1.0, t_horizon: float = 2.0,
                 dt_check: float = 0.05):
        """
        Initialize trajectory optimizer.

        Args:
            d_min: Minimum safe distance in meters
            urr: Urgency response ratio (0-1), higher = earlier response
            lambda_goal: Weight for goal attraction vs collision avoidance
            t_horizon: Propagation horizon for the per-primitive safety
                check in seconds (Paper Section 3.4)
            dt_check: Time step for the safety-check march in seconds
        """
        self.d_min = d_min
        self.urr = urr
        self.lambda_goal = lambda_goal
        self.t_horizon = t_horizon
        self.dt_check = dt_check
        self.dt = 0.5  # Primitive duration
```

3b. Add `is_primitive_safe` to `TrajectoryOptimizer` (new method, before `optimize`):

```python
    def is_primitive_safe(
        self,
        primitive: MotionPrimitive,
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        neighbor_states: Dict[int, Tuple[np.ndarray, np.ndarray]]
    ) -> bool:
        """
        Propagate this primitive against neighbors' predicted trajectories
        (Paper Section 3.4, unnumbered inequality):

            || p_i(t + t_iter) - p_j(t + t_iter) || >= d_min   for all j, t_iter

        My trajectory: kinematic under the primitive's acceleration during
        [0, dt], then constant final_velocity out to t_horizon. Neighbors fly
        their current velocity. Returns False on any breach.
        """
        current_position = np.asarray(current_position, dtype=float)
        current_velocity = np.asarray(current_velocity, dtype=float)
        accel = np.asarray(primitive.acceleration, dtype=float)
        p_end = current_position + primitive.position_delta
        v_end = np.asarray(primitive.final_velocity, dtype=float)

        t_iter = 0.0
        while t_iter <= self.t_horizon:
            if t_iter <= self.dt:
                p_i = (current_position + current_velocity * t_iter
                       + 0.5 * accel * t_iter * t_iter)
            else:
                p_i = p_end + v_end * (t_iter - self.dt)

            for neighbor_pos, neighbor_vel in neighbor_states.values():
                p_j = (np.asarray(neighbor_pos, dtype=float)
                       + np.asarray(neighbor_vel, dtype=float) * t_iter)
                if np.linalg.norm(p_i - p_j) < self.d_min:
                    return False

            t_iter += self.dt_check

        return True
```

3c. Replace `optimize` (lines 303-366) with:

```python
    def optimize(
        self,
        primitives: List[MotionPrimitive],
        current_position: np.ndarray,
        current_velocity: np.ndarray,
        neighbor_states: Dict[int, Tuple[np.ndarray, np.ndarray]],
        target_position: np.ndarray
    ) -> Optional[MotionPrimitive]:
        """
        Select the best motion primitive.

        Each candidate is propagated against every neighbor's predicted
        trajectory over the horizon (Section 3.4) and rejected on any d_min
        breach. Survivors are ranked by the cost function.

        Args:
            primitives: List of candidate primitives
            current_position: Current position [x, y, z]
            current_velocity: Current velocity [vx, vy, vz]
            neighbor_states: Dict of neighbor_id -> (position, velocity)
            target_position: Goal position

        Returns:
            Best motion primitive, or the hover primitive if all are unsafe
        """
        current_position = np.asarray(current_position, dtype=float)
        current_velocity = np.asarray(current_velocity, dtype=float)
        target_position = np.asarray(target_position, dtype=float)

        neighbor_positions = {nid: pv[0] for nid, pv in neighbor_states.items()}

        best_primitive = None
        min_cost = float('inf')

        for primitive in primitives:
            # Section 3.4 check: the primitive itself is propagated (LATENT-4)
            if not self.is_primitive_safe(
                    primitive, current_position, current_velocity,
                    neighbor_states):
                continue

            future_pos = current_position + primitive.position_delta
            cost = self.calculate_cost_with_repulsion(
                future_pos, primitive.final_velocity,
                neighbor_positions, target_position
            )

            if cost < min_cost:
                min_cost = cost
                best_primitive = primitive

        if best_primitive is None:
            # All primitives unsafe - return hover primitive
            # (Task 8 / LATENT-3 replaces this with `return None` -> stop command)
            for p in primitives:
                if np.allclose(p.acceleration, [0, 0, 0]):
                    return p
            return primitives[0] if primitives else None

        return best_primitive
```

3d. `CollisionAvoidanceAlgorithm.__init__` (lines 499-501): thread the horizon into the optimizer:

```python
        self.primitive_generator = MotionPrimitiveGenerator(v_max, a_max, dt_primitive)
        self.collision_predictor = CollisionPredictor(d_min, t_horizon, dt_prediction)
        self.trajectory_optimizer = TrajectoryOptimizer(
            d_min, urr, lambda_goal,
            t_horizon=t_horizon, dt_check=dt_prediction)
```

3e. Replace the body of `compute_safe_waypoint` (lines 526-556; docstring unchanged):

```python
        my_position = np.asarray(my_position, dtype=float)
        my_velocity = np.asarray(my_velocity, dtype=float)

        # Step 1: Generate motion primitives
        primitives = self.primitive_generator.generate_primitives(my_velocity)

        # Step 2: Build neighbor (position, velocity) pairs
        neighbor_pos_vel = {
            nid: (state.position, state.velocity)
            for nid, state in neighbor_states.items() if nid != my_id
        }

        # Step 3: Select optimal primitive (each candidate propagated against
        # neighbors' predicted trajectories - Paper Section 3.4)
        best_primitive = self.trajectory_optimizer.optimize(
            primitives,
            my_position,
            my_velocity,
            neighbor_pos_vel,
            target_position
        )

        if best_primitive is None:
            # Fallback: stay in place
            return my_position.copy()

        # Step 4: Return waypoint (current position + delta)
        return my_position + best_primitive.position_delta
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v
```
Expected: all pass (3 d_min + 5 propagation)

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance.py tests/unit/test_avoidance_algorithm_fixes.py
git commit -m "fix: propagate each primitive against neighbor trajectories per Section 3.4 (LATENT-4)"
```

---

### Task 8: Stop-command fallback with loud warning (LATENT-3)

When ALL primitives are unsafe, the current fallback returns the zero-acceleration primitive whose waypoint is `current + v·Δt` — keep flying into the predicted collision. USER DECISION: command a stop instead. `optimize()` returns `None`; `compute_safe_waypoint` returns `my_position` (re-commanded at 20 Hz → clean deceleration) and raises a `fallback_engaged` flag; the controller logs a loud warning.

**Files:**
- Modify: `formation_control/collision_avoidance.py` (`optimize` tail from Task 7 step 3c; `CollisionAvoidanceAlgorithm.__init__` and `compute_safe_waypoint`)
- Modify: `formation_control/collision_avoidance_controller.py` (`_avoidance_control_loop`, after the `compute_safe_waypoint` call)
- Test: `tests/unit/test_avoidance_algorithm_fixes.py` (append)

**Interfaces:**
- Consumes: Task 7's `optimize` signature.
- Produces: `optimize(...) -> Optional[MotionPrimitive]` returning `None` when nothing is safe; `CollisionAvoidanceAlgorithm.fallback_engaged: bool` (True right after a fallback tick, False otherwise). Task 10's velocity conversion turns the held waypoint into a zero-velocity command automatically.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_algorithm_fixes.py`:

```python
from formation_control.collision_avoidance import DroneState


class TestAllPrimitivesUnsafeFallback:
    """LATENT-3 / USER DECISION 2026-07-10: worst case = command a stop,
    never continued drift toward the collision."""

    def setup_method(self):
        # Head-on at 2 m/s each from 4 m apart with d_min=1.0:
        # every one of the 7 primitives breaches within the 2 s horizon.
        self.gen = MotionPrimitiveGenerator(v_max=5.0, a_max=2.0, dt=0.5)
        self.opt = TrajectoryOptimizer(d_min=1.0, t_horizon=2.0, dt_check=0.05)
        self.my_pos = np.array([0.0, 0.0, HOVER_ALT])
        self.my_vel = np.array([2.0, 0.0, 0.0])
        self.neighbors = {2: (np.array([4.0, 0.0, HOVER_ALT]),
                              np.array([-2.0, 0.0, 0.0]))}
        self.target = np.array([20.0, 0.0, HOVER_ALT])

    def test_every_primitive_is_rejected_in_this_scenario(self):
        primitives = self.gen.generate_primitives(self.my_vel)
        assert not any(
            self.opt.is_primitive_safe(p, self.my_pos, self.my_vel, self.neighbors)
            for p in primitives)

    def test_optimize_returns_none_not_hover(self):
        primitives = self.gen.generate_primitives(self.my_vel)
        assert self.opt.optimize(
            primitives, self.my_pos, self.my_vel,
            self.neighbors, self.target) is None

    def test_compute_safe_waypoint_holds_position_and_sets_flag(self):
        algo = CollisionAvoidanceAlgorithm(d_min=1.0)
        neighbor_states = {2: DroneState(
            instance_id=2,
            position=self.neighbors[2][0],
            velocity=self.neighbors[2][1])}
        waypoint = algo.compute_safe_waypoint(
            my_id=1, my_position=self.my_pos, my_velocity=self.my_vel,
            target_position=self.target, neighbor_states=neighbor_states)
        np.testing.assert_allclose(waypoint, self.my_pos)
        assert algo.fallback_engaged is True

    def test_flag_clears_on_safe_tick(self):
        algo = CollisionAvoidanceAlgorithm(d_min=1.0)
        algo.fallback_engaged = True  # pretend last tick was a fallback
        waypoint = algo.compute_safe_waypoint(
            my_id=1,
            my_position=np.array([0.0, 0.0, HOVER_ALT]),
            my_velocity=np.zeros(3),
            target_position=np.array([10.0, 0.0, HOVER_ALT]),
            neighbor_states={})  # nobody around
        assert algo.fallback_engaged is False
        assert not np.allclose(waypoint, [0.0, 0.0, HOVER_ALT])  # progresses
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v -k Fallback
```
Expected: `test_every_primitive_is_rejected_in_this_scenario` PASSES (Task 7 made it true); the other three FAIL (optimize returns hover primitive; no `fallback_engaged` attribute).

- [ ] **Step 3: Implement**

3a. In `TrajectoryOptimizer.optimize`, replace the fallback tail (the `if best_primitive is None:` block from Task 7) with:

```python
        # None = all primitives unsafe. Caller must command a stop (LATENT-3:
        # the old hover-primitive fallback meant "keep flying at current
        # velocity straight into the predicted collision").
        return best_primitive
```

3b. In `CollisionAvoidanceAlgorithm.__init__`, after `self.d_min = d_min`:

```python
        # True right after a compute_safe_waypoint tick in which every
        # primitive was unsafe and the drone was commanded to hold (LATENT-3).
        self.fallback_engaged = False
```

3c. In `compute_safe_waypoint`, replace the tail (from `if best_primitive is None:`) with:

```python
        if best_primitive is None:
            # All primitives unsafe -> stop command: hold current position.
            # Re-commanded at 20 Hz this decelerates the drone to a clean stop
            # (USER DECISION 2026-07-10: no p_safe selection).
            self.fallback_engaged = True
            return my_position.copy()

        self.fallback_engaged = False
        # Step 4: Return waypoint (current position + delta)
        return my_position + best_primitive.position_delta
```

3d. In `collision_avoidance_controller.py` `_avoidance_control_loop`, immediately after the `compute_safe_waypoint(...)` call (Step 3):

```python
        if self.collision_avoidance.fallback_engaged:
            self.get_logger().warning(
                f'ALL PRIMITIVES UNSAFE for drone {self.instance_id} - '
                f'commanding STOP at current position (LATENT-3 fallback). '
                f'If this repeats, inspect neighbor states and d_min.',
                throttle_duration_sec=1.0)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance.py formation_control/collision_avoidance_controller.py \
        tests/unit/test_avoidance_algorithm_fixes.py
git commit -m "fix: all-primitives-unsafe fallback commands a stop with loud warning (LATENT-3)"
```

---

### Task 9: Paper Eq. 2 cost function (LATENT-5, cost half)

Replace the unvalidated alignment-heuristic cost (`calculate_cost_with_repulsion` + `calculate_repulsion_weight`) with paper Eq. 2:

```
J = Σ_j 1/‖p_i − p_j‖²  +  λ‖v_i − v_d‖²
```

where `v_d` is the desired velocity toward the goal: `v_d = clamp((target − current_position)/dt, v_max)` — full speed when far, naturally decaying near the goal.

**Files:**
- Modify: `formation_control/collision_avoidance.py` (`TrajectoryOptimizer.__init__`, `optimize` cost call, delete `calculate_cost_with_repulsion` lines 368-425 and `calculate_repulsion_weight` lines 427-449, add `calculate_cost`; `CollisionAvoidanceAlgorithm.__init__` optimizer construction)
- Test: `tests/unit/test_avoidance_algorithm_fixes.py` (append)

**Interfaces:**
- Consumes: Task 7/8 `optimize`.
- Produces: `TrajectoryOptimizer(..., v_max=5.0)`; `calculate_cost(future_position, future_velocity, neighbor_positions: Dict[int, np.ndarray], desired_velocity: np.ndarray) -> float`. `calculate_cost_with_repulsion` and `calculate_repulsion_weight` are **removed** (only the legacy tests' inline copies reference those names — verify with the grep in Step 3d).

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_algorithm_fixes.py`:

```python
class TestEquation2Cost:
    """Paper Eq. 2: J = sum_j 1/||p_i-p_j||^2 + lambda*||v_i-v_d||^2."""

    def setup_method(self):
        self.opt = TrajectoryOptimizer(
            d_min=1.0, lambda_goal=1.0, v_max=5.0)

    def test_exact_value_single_neighbor(self):
        cost = self.opt.calculate_cost(
            future_position=np.array([0.0, 0.0, 20.0]),
            future_velocity=np.array([1.0, 0.0, 0.0]),
            neighbor_positions={2: np.array([2.0, 0.0, 20.0])},
            desired_velocity=np.array([3.0, 0.0, 0.0]))
        # 1/2^2 + 1.0 * ||(1,0,0)-(3,0,0)||^2 = 0.25 + 4.0
        assert cost == pytest.approx(4.25)

    def test_no_neighbors_pure_velocity_tracking(self):
        cost = self.opt.calculate_cost(
            future_position=np.array([0.0, 0.0, 20.0]),
            future_velocity=np.array([5.0, 0.0, 0.0]),
            neighbor_positions={},
            desired_velocity=np.array([5.0, 0.0, 0.0]))
        assert cost == pytest.approx(0.0)

    def test_closer_neighbor_costs_more(self):
        common = dict(
            future_velocity=np.zeros(3),
            desired_velocity=np.zeros(3))
        near = self.opt.calculate_cost(
            future_position=np.array([0.0, 0.0, 20.0]),
            neighbor_positions={2: np.array([1.5, 0.0, 20.0])}, **common)
        far = self.opt.calculate_cost(
            future_position=np.array([0.0, 0.0, 20.0]),
            neighbor_positions={2: np.array([5.0, 0.0, 20.0])}, **common)
        assert near > far

    def test_optimize_prefers_goal_directed_primitive_in_open_space(self):
        gen = MotionPrimitiveGenerator(v_max=5.0, a_max=2.0, dt=0.5)
        primitives = gen.generate_primitives(np.zeros(3))
        best = self.opt.optimize(
            primitives,
            current_position=np.array([0.0, 0.0, 20.0]),
            current_velocity=np.zeros(3),
            neighbor_states={},
            target_position=np.array([10.0, 0.0, 20.0]))
        assert best.acceleration[0] > 0, 'should accelerate toward +X goal'
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v -k Equation2
```
Expected: FAIL — no `v_max` kwarg, no `calculate_cost` method.

- [ ] **Step 3: Implement in `collision_avoidance.py`**

3a. Add `v_max` to `TrajectoryOptimizer.__init__` (extends Task 7's version):

```python
    def __init__(self, d_min: float = 1.0, urr: float = 0.75,
                 lambda_goal: float = 1.0, t_horizon: float = 2.0,
                 dt_check: float = 0.05, v_max: float = 5.0):
```
with the added line in the body:
```python
        self.v_max = v_max
```

3b. In `optimize`, compute the desired velocity once before the primitive loop, and use the new cost:

```python
        # Desired velocity toward the goal (v_d in Eq. 2): full speed when far,
        # decaying inside one primitive-length of the target.
        goal_vec = target_position - current_position
        desired_velocity = goal_vec / self.dt
        v_d_mag = np.linalg.norm(desired_velocity)
        if v_d_mag > self.v_max:
            desired_velocity = desired_velocity * (self.v_max / v_d_mag)
```

and replace the cost call inside the loop:

```python
            future_pos = current_position + primitive.position_delta
            cost = self.calculate_cost(
                future_pos, primitive.final_velocity,
                neighbor_positions, desired_velocity
            )
```

3c. Delete `calculate_cost_with_repulsion` (lines 368-425) and `calculate_repulsion_weight` (lines 427-449); add:

```python
    def calculate_cost(
        self,
        future_position: np.ndarray,
        future_velocity: np.ndarray,
        neighbor_positions: Dict[int, np.ndarray],
        desired_velocity: np.ndarray
    ) -> float:
        """
        Paper Equation 2:  J = sum_j 1/||p_i - p_j||^2 + lambda*||v_i - v_d||^2

        First term: inverse-square proximity penalty per neighbor (distance
        floored at 0.1 m to bound the cost). Second term: quadratic penalty
        for deviating from the desired (goal-directed) velocity.
        """
        future_position = np.asarray(future_position, dtype=float)
        future_velocity = np.asarray(future_velocity, dtype=float)
        desired_velocity = np.asarray(desired_velocity, dtype=float)

        cost = 0.0
        for neighbor_pos in neighbor_positions.values():
            distance = np.linalg.norm(
                future_position - np.asarray(neighbor_pos, dtype=float))
            cost += 1.0 / max(distance, 0.1) ** 2

        velocity_error = future_velocity - desired_velocity
        cost += self.lambda_goal * float(np.dot(velocity_error, velocity_error))
        return cost
```

3d. `CollisionAvoidanceAlgorithm.__init__`: pass `v_max` through:

```python
        self.trajectory_optimizer = TrajectoryOptimizer(
            d_min, urr, lambda_goal,
            t_horizon=t_horizon, dt_check=dt_prediction, v_max=v_max)
```

Then verify nothing outside the legacy inline-copy tests references the deleted names:

```bash
grep -rn "calculate_cost_with_repulsion\|calculate_repulsion_weight" formation_control/
```
Expected: no matches.

- [ ] **Step 4: Run the full algorithm test file**

```bash
python3 -m pytest tests/unit/test_avoidance_algorithm_fixes.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance.py tests/unit/test_avoidance_algorithm_fixes.py
git commit -m "feat: replace alignment-heuristic cost with paper Eq. 2 (LATENT-5 cost half)"
```

---

### Task 10: Velocity-command actuation on /ap_N/cmd_vel (LATENT-5, actuation half)

GPS micro-waypoints keep ArduPilot's guided position controller in a perpetual acceleration-limited replan — far too sluggish for multi-m/s closing speeds. Switch the controller output to `/ap_N/cmd_vel` (`geometry_msgs/TwistStamped`), which matches the paper's Algorithm 1 control model (`p_i(t+Δt) = p_i(t) + R(t)Δt`). Velocity command: `v_cmd = clamp((safe_waypoint − my_position)/dt_primitive, v_max)`. On the stop fallback (`waypoint == my_position`) this is exactly zero → clean stop. **`header.frame_id` must be `'map'`** (ENU; verified against `AP_DDS_ExternalControl.cpp:75-80`).

**Files:**
- Modify: `formation_control/collision_avoidance_controller.py` (params ~line 67, `_setup_publishers` lines 159-173, `_avoidance_control_loop` steps 4-5, delete `_local_to_gps` and the now-unused `GlobalPosition` import)
- Test: `tests/unit/test_avoidance_wiring.py` (append)

**Interfaces:**
- Consumes: `compute_safe_waypoint` (Tasks 7-9); `TwistStamped` (already imported).
- Produces: `/ap_{N}/cmd_vel` TwistStamped at 20 Hz, ENU linear velocity, `frame_id='map'`; parameter `dt_primitive` (default 0.5) also passed into `CollisionAvoidanceAlgorithm`; publisher attribute `self.cmd_vel_pub` (replaces `self.gps_cmd_pub`). The GPS origin subscription **stays** — it anchors the shared ENU frame (Task 5), even though no GPS commands are sent anymore.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_wiring.py`:

```python
def make_ready_controller(instance_id=1):
    """Controller with origin, own position, neighbor set, and target — ready
    for one _avoidance_control_loop tick. cmd_vel publisher mocked."""
    node = CollisionAvoidanceController(instance_id=instance_id)
    node._gps_origin_callback(make_origin_msg())
    node.cmd_vel_pub = MagicMock()
    node.status_pub = MagicMock()
    node.my_position = np.array([0.0, 0.0, 20.0])
    node.my_velocity = np.array([0.0, 0.0, 0.0])
    node.drone_positions = {instance_id: node.my_position.copy()}
    node.target_position = np.array([50.0, 0.0, 20.0])  # far away on +X
    return node


class TestCmdVelActuation:
    def test_publishes_twist_stamped_with_map_frame(self, ros_init):
        node = make_ready_controller()
        assert node._avoidance_control_loop() is True
        assert node.cmd_vel_pub.publish.called
        msg = node.cmd_vel_pub.publish.call_args[0][0]
        assert msg.header.frame_id == 'map'  # ENU per AP_DDS_ExternalControl
        node.destroy_node()

    def test_velocity_toward_target_clamped_to_v_max(self, ros_init):
        node = make_ready_controller()
        node._avoidance_control_loop()
        msg = node.cmd_vel_pub.publish.call_args[0][0]
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        assert np.linalg.norm(v) <= node.v_max + 1e-6
        assert v[0] > 0.0, 'should command velocity toward the +X target'
        node.destroy_node()

    def test_stop_fallback_publishes_zero_velocity(self, ros_init):
        node = make_ready_controller()
        # Head-on trap: all primitives unsafe (same scenario as Task 8)
        node.my_velocity = np.array([2.0, 0.0, 0.0])
        node.drone_positions[2] = np.array([4.0, 0.0, 20.0])
        node.drone_velocities[2] = np.array([-2.0, 0.0, 0.0])
        node._avoidance_control_loop()
        msg = node.cmd_vel_pub.publish.call_args[0][0]
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        np.testing.assert_allclose(v, [0.0, 0.0, 0.0], atol=1e-9)
        node.destroy_node()

    def test_no_gps_pose_publisher_remains(self, ros_init):
        node = CollisionAvoidanceController(instance_id=1)
        pub_topics = [p.topic_name for p in node.publishers]
        assert not any('cmd_gps_pose' in t for t in pub_topics), \
            'controller still owns the GPS channel (LATENT-5)'
        assert any(t == '/ap_1/cmd_vel' for t in pub_topics)
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v -k CmdVel
```
Expected: FAIL — no `cmd_vel_pub` attribute; `cmd_gps_pose` publisher still present.

- [ ] **Step 3: Implement in `collision_avoidance_controller.py`**

3a. Parameters — add with the others and read it; pass into the algorithm:

```python
        self.declare_parameter('dt_primitive', 0.5)
```
```python
        self.dt_primitive = self.get_parameter('dt_primitive').value
```
```python
        self.collision_avoidance = CollisionAvoidanceAlgorithm(
            v_max=self.v_max,
            a_max=self.a_max,
            d_min=self.d_min,
            t_horizon=self.t_horizon,
            urr=self.urr,
            lambda_goal=self.lambda_goal,
            dt_primitive=self.dt_primitive
        )
```

3b. `_setup_publishers` — replace the GPS command publisher (lines 161-166) with:

```python
        # Velocity command publisher (LATENT-5: matches the paper's control
        # model; frame_id 'map' => ENU, converted to NED inside ArduPilot)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            f'/ap_{self.instance_id}/cmd_vel',
            10
        )
```

3c. `_avoidance_control_loop` — replace Steps 4-5 (the `_local_to_gps` call through `self.gps_cmd_pub.publish(gps_cmd)`) with:

```python
        # Step 4: Convert waypoint to a velocity command. On the stop
        # fallback (waypoint == position) this is exactly zero -> clean stop.
        v_cmd = (safe_waypoint - self.my_position) / self.dt_primitive
        v_mag = np.linalg.norm(v_cmd)
        if v_mag > self.v_max:
            v_cmd = v_cmd * (self.v_max / v_mag)

        # Step 5: Publish ENU velocity command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'  # ENU; AP_DDS converts to NED
        cmd.twist.linear.x = float(v_cmd[0])
        cmd.twist.linear.y = float(v_cmd[1])
        cmd.twist.linear.z = float(v_cmd[2])
        self.cmd_vel_pub.publish(cmd)
        self.last_command_time = self.get_clock().now()
```

3d. Delete the now-dead `_local_to_gps` method and the `from ardupilot_msgs.msg import GlobalPosition` import (geo conversion for *positions* still comes from `geo_utils` via `_geopose_callback`; the origin subscription stays).

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance_controller.py tests/unit/test_avoidance_wiring.py
git commit -m "feat: avoidance output on /ap_N/cmd_vel velocity control (LATENT-5)"
```

---

### Task 11: Controller defers to emergency stops

With the commander gated (Task 2) and the controller streaming cmd_vel at 20 Hz (Task 10), an emergency would be a channel fight: the commander's GPS hold vs. the controller's velocity stream. Extension of BREAK-3's single-owner principle: on `/safety/emergency_stop` naming this instance, the controller halts its output (the commander's hold takes over); it resumes when `/safety/status` returns to `SAFE`.

**Files:**
- Modify: `formation_control/collision_avoidance_controller.py` (`__init__` state, `_setup_subscriptions`, `_avoidance_control_loop` step 0)
- Test: `tests/unit/test_avoidance_wiring.py` (append)

**Interfaces:**
- Consumes: `/safety/emergency_stop` (`std_msgs/String`, format `"EMERGENCY_STOP: 1,2,3"` per `safety_monitor.py:340`); `/safety/status` (`std_msgs/String`, `"SAFE"|"WARNING"|"EMERGENCY"`).
- Produces: attribute `self.emergency_halted: bool`; callbacks `_emergency_stop_callback`, `_safety_status_callback`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_wiring.py`:

```python
from std_msgs.msg import String


def string_msg(data):
    msg = String()
    msg.data = data
    return msg


class TestEmergencyDeference:
    def test_halts_when_own_instance_in_emergency_stop(self, ros_init):
        node = make_ready_controller(instance_id=2)
        node._emergency_stop_callback(string_msg('EMERGENCY_STOP: 1,2'))
        assert node.emergency_halted is True
        assert node._avoidance_control_loop() is False
        assert not node.cmd_vel_pub.publish.called, \
            'controller kept streaming cmd_vel during an emergency (channel fight)'
        node.destroy_node()

    def test_ignores_emergency_for_other_instances(self, ros_init):
        node = make_ready_controller(instance_id=2)
        node._emergency_stop_callback(string_msg('EMERGENCY_STOP: 1,3'))
        assert node.emergency_halted is False
        assert node._avoidance_control_loop() is True
        node.destroy_node()

    def test_resumes_when_safety_restored(self, ros_init):
        node = make_ready_controller(instance_id=2)
        node._emergency_stop_callback(string_msg('EMERGENCY_STOP: 2'))
        node._safety_status_callback(string_msg('SAFE'))
        assert node.emergency_halted is False
        assert node._avoidance_control_loop() is True
        node.destroy_node()

    def test_does_not_resume_while_still_emergency(self, ros_init):
        node = make_ready_controller(instance_id=2)
        node._emergency_stop_callback(string_msg('EMERGENCY_STOP: 2'))
        node._safety_status_callback(string_msg('EMERGENCY'))
        assert node.emergency_halted is True
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v -k EmergencyDeference
```
Expected: FAIL — `AttributeError: _emergency_stop_callback`.

- [ ] **Step 3: Implement in `collision_avoidance_controller.py`**

3a. In `__init__`, with the other state (~line 91):

```python
        # True while the safety monitor has this drone in emergency: the
        # formation commander's GPS hold owns the channel, we stay silent.
        self.emergency_halted = False
```

3b. In `_setup_subscriptions`, at the end:

```python
        # Safety system integration: defer to emergency stops
        self.create_subscription(
            String, '/safety/emergency_stop',
            self._emergency_stop_callback, 10)
        self.create_subscription(
            String, '/safety/status',
            self._safety_status_callback, 10)
```

3c. New callbacks (after `_gps_origin_callback`):

```python
    def _emergency_stop_callback(self, msg: String):
        """Halt output when the safety monitor puts THIS drone in emergency."""
        if 'EMERGENCY_STOP:' not in msg.data:
            return
        try:
            id_str = msg.data.split('EMERGENCY_STOP:')[1].strip()
            emergency_ids = [int(x.strip()) for x in id_str.split(',') if x.strip()]
        except ValueError:
            return
        if self.instance_id in emergency_ids:
            self.emergency_halted = True
            self.get_logger().error(
                f'EMERGENCY STOP for drone {self.instance_id} - avoidance '
                f'output HALTED; formation commander owns the channel')

    def _safety_status_callback(self, msg: String):
        """Resume output once the safety monitor reports SAFE again."""
        if self.emergency_halted and msg.data == 'SAFE':
            self.emergency_halted = False
            self.get_logger().info(
                'Safety restored - avoidance output resumed')
```

3d. In `_avoidance_control_loop`, right after the Step 0 origin check:

```python
        # Step 0.5: During an emergency the commander's hold owns the channel
        if self.emergency_halted:
            return False
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance_controller.py tests/unit/test_avoidance_wiring.py
git commit -m "feat: controller halts cmd_vel output during safety-monitor emergencies"
```

---

### Task 12: Loud warning when the velocity feed is absent (LATENT-6)

`/ap_N/twist/filtered` IS in this ArduPilot's DDS topic table, but if it doesn't arrive the controller silently substitutes zero velocity — blinding the predictor. Warn loudly (throttled) instead, and stop the sticky `self.my_velocity = zeros` assignment which permanently masks the missing feed after the first tick.

**Files:**
- Modify: `formation_control/collision_avoidance_controller.py` (`_avoidance_control_loop` steps 1-3)
- Test: `tests/unit/test_avoidance_wiring.py` (append)

**Interfaces:**
- Consumes: nothing new.
- Produces: `self.my_velocity` stays `None` until a real message arrives (a local zero is used per-tick); throttled warnings for own and neighbor missing feeds.

- [ ] **Step 1: Write the failing tests**

Append to `tests/unit/test_avoidance_wiring.py`:

```python
class TestVelocityFeedWarning:
    def test_missing_own_velocity_warns_and_stays_none(self, ros_init):
        node = make_ready_controller()
        node.my_velocity = None  # feed never arrived
        mock_logger = MagicMock()
        node.get_logger = lambda: mock_logger
        assert node._avoidance_control_loop() is True  # still commands
        assert node.my_velocity is None, \
            'zero-substitution must not be cached (masks the dead feed forever)'
        warnings = ' '.join(
            str(c.args[0]) for c in mock_logger.warning.call_args_list)
        assert 'twist/filtered' in warnings, 'LATENT-6: silent degradation'
        node.destroy_node()

    def test_missing_neighbor_velocity_warns(self, ros_init):
        node = make_ready_controller()
        node.drone_positions[3] = np.array([10.0, 10.0, 20.0])
        # note: no drone_velocities[3]
        mock_logger = MagicMock()
        node.get_logger = lambda: mock_logger
        node._avoidance_control_loop()
        warnings = ' '.join(
            str(c.args[0]) for c in mock_logger.warning.call_args_list)
        assert '3' in warnings and 'velocity' in warnings.lower()
        node.destroy_node()

    def test_no_warning_when_feeds_present(self, ros_init):
        node = make_ready_controller()
        mock_logger = MagicMock()
        node.get_logger = lambda: mock_logger
        node._avoidance_control_loop()
        assert not mock_logger.warning.called
        node.destroy_node()
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
python3 -m pytest tests/unit/test_avoidance_wiring.py -v -k VelocityFeedWarning
```
Expected: `test_missing_own_velocity_warns_and_stays_none` FAILS (`my_velocity` gets overwritten with zeros, no warning). `test_no_warning_when_feeds_present` may already pass.

- [ ] **Step 3: Implement in `_avoidance_control_loop`**

Replace Steps 1-3 (from `# Step 1: Check for valid data` through the `compute_safe_waypoint(...)` call) with:

```python
        # Step 1: Check for valid data
        if self.my_position is None or self.target_position is None:
            return False  # Skip iteration

        my_velocity = self.my_velocity
        if my_velocity is None:
            # LATENT-6: a missing velocity feed blinds the collision
            # predictor. Warn loudly; use zero for this tick only (never
            # cached, so the warning persists until the feed appears).
            self.get_logger().warning(
                f'No velocity feed on /ap_{self.instance_id}/twist/filtered - '
                f'collision prediction is BLIND (assuming zero velocity). '
                f'Verify with: ros2 topic hz /ap_{self.instance_id}/twist/filtered',
                throttle_duration_sec=5.0)
            my_velocity = np.zeros(3)

        # Step 2: Build neighbor states
        missing_velocity_ids = []
        neighbor_states: Dict[int, DroneState] = {}
        for iid, pos in self.drone_positions.items():
            if iid != self.instance_id:
                if iid not in self.drone_velocities:
                    missing_velocity_ids.append(iid)
                vel = self.drone_velocities.get(iid, np.array([0.0, 0.0, 0.0]))
                neighbor_states[iid] = DroneState(
                    instance_id=iid,
                    position=pos,
                    velocity=vel
                )

        if missing_velocity_ids:
            self.get_logger().warning(
                f'No velocity feed for neighbor drones {missing_velocity_ids} - '
                f'assuming zero velocity (predictions unreliable, LATENT-6)',
                throttle_duration_sec=5.0)

        # Step 3: Compute safe waypoint using collision avoidance
        safe_waypoint = self.collision_avoidance.compute_safe_waypoint(
            my_id=self.instance_id,
            my_position=self.my_position,
            my_velocity=my_velocity,
            target_position=self.target_position,
            neighbor_states=neighbor_states
        )
```

(The Task 8 `fallback_engaged` warning block stays immediately after, then Task 10's Steps 4-5.)

- [ ] **Step 4: Run the complete unit suite (all new files)**

```bash
python3 -m pytest tests/unit/test_geo_utils.py tests/unit/test_avoidance_wiring.py \
    tests/unit/test_avoidance_algorithm_fixes.py tests/unit/test_safety_monitor_frames.py -v
```
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add formation_control/collision_avoidance_controller.py tests/unit/test_avoidance_wiring.py
git commit -m "fix: loud throttled warning when velocity feed is absent (LATENT-6)"
```

---

### Task 13: Runtime verification in simulation (spec Part 3)

Empirically confirm every chain-break is fixed with the real sim. This task produces a short results log, not code.

**Files:**
- Create: `docs/superpowers/Log/2026-XX-XX-avoidance-runtime-verification.md` (in the **workspace** docs dir, dated on the day of execution; record each check's output)

- [ ] **Step 1: Build and source (non-negotiable — see spec Part 3 workflow note)**

```bash
cd /home/finn/Documents/ardu_ws
colcon build --packages-select formation_control
source install/setup.bash
```

- [ ] **Step 2: Start the sim and take off (three terminals, each sourced the same way)**

```bash
# T1: simulation (5 SITL instances + Gazebo)
ros2 launch ardupilot_gz_bringup iris_multi_uav.launch.py

# T2: wait for GPS lock (~1-2 min), then take off
ros2 run formation_control swarm_takeoff_commander

# T3: after takeoff completes, start the avoidance stack
ros2 launch formation_control collision_avoidance.launch.py
```

- [ ] **Step 3: Verify the integration chain (T4, sourced)**

```bash
ros2 node list
# EXPECT: /formation_commander, /safety_monitor, /collision_avoidance_controller_1..4  (BREAK-1 fixed)

ros2 topic info /formation/targets/drone_1
# EXPECT: Publisher count: 1, Subscription count: 1                                    (BREAK-2 fixed)

ros2 topic hz /ap_1/cmd_vel
# EXPECT: ~20 Hz from the controller                                                   (LATENT-5 active)

ros2 topic info /ap_1/cmd_gps_pose --verbose
# EXPECT: formation_commander listed but publishing nothing while avoidance_active     (BREAK-3 fixed)

ros2 topic hz /ap_1/twist/filtered
# EXPECT: ~10-30 Hz (LATENT-6: if absent, controller logs BLIND warnings instead of silence)

ros2 topic echo /ap_1/geopose/filtered --once
# EXPECT: global lat/lon/alt (LATENT-1 common frame source present)
ros2 topic hz /ap_1/geopose/filtered
# EXPECT: rate adequate for the 20 Hz loop (>= ~10 Hz)
```

- [ ] **Step 4: Observe avoidance behavior**

Write this monitor to the scratchpad (NOT the repo) as `min_dist.py` and run it:

```python
"""Print the live minimum pairwise inter-drone distance (shared ENU frame)."""
import math

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

R = 6378137.0
positions = {}


class MinDist(Node):
    def __init__(self):
        super().__init__('min_dist_monitor')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for i in range(1, 5):
            self.create_subscription(
                GeoPoseStamped, f'/ap_{i}/geopose/filtered',
                lambda m, iid=i: self.cb(m, iid), qos)
        self.create_timer(0.5, self.report)
        self.min_seen = float('inf')

    def cb(self, m, iid):
        lat, lon, alt = (m.pose.position.latitude,
                         m.pose.position.longitude, m.pose.position.altitude)
        positions[iid] = (math.radians(lat) * R,
                          math.radians(lon) * R * math.cos(math.radians(lat)), alt)

    def report(self):
        ids = sorted(positions)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                pa, pb = positions[ids[a]], positions[ids[b]]
                d = math.dist(pa, pb)
                self.min_seen = min(self.min_seen, d)
        if len(ids) >= 2:
            print(f'drones={ids} min_pairwise_ever={self.min_seen:.2f} m')


rclpy.init()
rclpy.spin(MinDist())
```

Acceptance criteria while the formation assembles:
- Drones visibly detour around each other in Gazebo instead of flying straight through.
- `min_pairwise_ever` never drops below **0.9 m** (d_min 1.0 m with 10% tolerance).
- Formation completes: commander logs `Formation assembly completed!`.
- No repeating `ALL PRIMITIVES UNSAFE` warnings after assembly (occasional during close crossings is acceptable).
- No `Safety system not detected` spam (safety monitor heartbeat present).
- Emergency path: run `ros2 topic pub -1 /safety/emergency_stop std_msgs/String "{data: 'EMERGENCY_STOP: 1'}"` → controller 1 logs HALTED and `/ap_1/cmd_vel` goes silent; commander sends hold.

- [ ] **Step 5: Record results and commit the log**

Write the observed outputs/criteria pass-fail into `docs/superpowers/Log/2026-XX-XX-avoidance-runtime-verification.md` (workspace docs — not the package repo). Then in the package repo:

```bash
cd /home/finn/Documents/ardu_ws/src/formation_control
git log --oneline fix/avoidance-integration
```
Confirm all task commits are present. Use superpowers:verification-before-completion before claiming done; then superpowers:finishing-a-development-branch to decide merge/PR.

---

## Self-Review

**Spec coverage** (against RESEARCH-AVOIDANCE-FAILURE-DIAGNOSIS.md Part 4):
1. Wire the handoff (BREAK-2/3) → Task 2 ✓
2. Launch the stack (BREAK-1) → Task 3 ✓
3. Unify frames (LATENT-1, geopose decision) → Tasks 1, 4, 5, 6 ✓ (commander race fix in Task 4)
4. d_min 1.0 (same pass as frames) → Task 5 (+ monitor thresholds Task 6) ✓
5. Algorithm fixes (LATENT-3/4) → Tasks 7, 8 ✓ (LATENT-4 first so LATENT-3 tests target the final `optimize` signature; loud warning included)
6. Actuation cmd_vel + Eq. 2 cost (LATENT-5) → Tasks 9, 10 ✓
7. Hardening (LATENT-6) → Task 12 ✓
Part 3 runtime verification → Task 13 ✓. Additions beyond the spec, justified inline: `GlobalPosition.header.frame_id` fix (Task 5 — ArduPilot silently drops frameless commands), emergency deference (Task 11 — required so the Phase-0 emergency system still works once the controller streams at 20 Hz), missing `package.xml` deps (Task 3).

**Type consistency:** `optimize(primitives, current_position, current_velocity, neighbor_states, target_position)` is used identically in Tasks 7, 8, 9, 10 tests; `neighbor_states` is `Dict[int, Tuple[position, velocity]]` at optimizer level and `Dict[int, DroneState]` at algorithm level (converted in `compute_safe_waypoint`); `fallback_engaged` defined in Task 8, consumed in Tasks 8/10; `gps_to_enu`/`enu_to_gps` signatures match across Tasks 1/4/5/6.
