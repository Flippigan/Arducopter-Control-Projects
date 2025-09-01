#!/usr/bin/env python3
"""
Safety Monitor Node

Centralized collision detection and prevention system for multi-drone formations.
Monitors all drone positions and triggers emergency stops when collision risks are detected.

UPDATED FOR MULTI-DRONE INTEGRATION:
- Uses actual drone names: iris_9002, iris_9012, iris_9022, iris_9032, iris_9042
- Configurable drone list via parameters
- Enhanced communication health monitoring
- Ready for DDS integration when drone_interface.py provides position data

Author: Finn Picotoli
Date: 2025-08-22
Updated: 2025-08-27 (Multi-drone naming integration)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Safety parameters
        self.safety_radius = 3.0  # meters - minimum safe distance
        self.warning_radius = 6.0  # meters - collision warning distance
        self.prediction_time = 2.0  # seconds - trajectory prediction horizon
        self.max_velocity = 5.0  # m/s - maximum expected drone velocity
        
        # Monitoring state - will be configured from parameters
        self.drone_names = []  # Actual drone names from simulation
        self.num_drones = 0  # Will be set from drone_names length
        self.drone_positions = {}  # drone_id: [x, y, z]
        self.drone_velocities = {}  # drone_id: [vx, vy, vz]
        self.drone_last_update = {}  # drone_id: timestamp
        self.collision_pairs = []  # [(drone1, drone2, risk_level), ...]
        
        # Safety status
        self.system_safe = True
        self.emergency_active = False
        
        # Publishers
        self.safety_status_pub = self.create_publisher(String, '/safety/status', 10)
        self.collision_warning_pub = self.create_publisher(String, '/safety/collision_warning', 10)
        self.emergency_stop_pub = self.create_publisher(String, '/safety/emergency_stop', 10)
        self.safety_visualization_pub = self.create_publisher(MarkerArray, '/safety/visualization', 10)
        
        # Subscribers for drone positions (will be created dynamically)
        self.drone_position_subs = {}
        
        # Parameters - use actual drone names from simulation
        self.declare_parameter('drone_names', ['iris_9002', 'iris_9012', 'iris_9022', 'iris_9032', 'iris_9042'])
        self.declare_parameter('safety_radius', 3.0)
        self.declare_parameter('warning_radius', 6.0)
        self.declare_parameter('prediction_time', 2.0)
        self.declare_parameter('max_velocity', 5.0)
        
        # Configure from parameters
        self.drone_names = self.get_parameter('drone_names').value
        self.num_drones = len(self.drone_names)
        self.safety_radius = self.get_parameter('safety_radius').value
        self.warning_radius = self.get_parameter('warning_radius').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Setup drone position monitoring
        self._setup_drone_monitoring()
        
        # Timers
        self.safety_check_timer = self.create_timer(0.05, self._safety_check_loop)  # 20Hz
        self.status_timer = self.create_timer(0.5, self._publish_safety_status)
        self.visualization_timer = self.create_timer(0.1, self._publish_safety_visualization)
        
        self.get_logger().info(f'Safety Monitor initialized for {self.num_drones} drones: {self.drone_names}')
        self.get_logger().info(f'Safety parameters: radius={self.safety_radius}m, warning={self.warning_radius}m')
        self.get_logger().info(f'Prediction parameters: horizon={self.prediction_time}s, max_vel={self.max_velocity}m/s')

    def _setup_drone_monitoring(self):
        """Setup subscribers for all drone position data using actual drone names"""
        for drone_id in self.drone_names:
            # Subscribe to drone odometry (NOTE: depends on drone_interface.py providing /odom topics)
            topic_name = f'/{drone_id}/odom'
            self.drone_position_subs[drone_id] = self.create_subscription(
                Odometry, topic_name,
                lambda msg, drone=drone_id: self._drone_position_callback(msg, drone), 10)
            
            self.get_logger().info(f'Monitoring safety for drone {drone_id} on topic {topic_name}')
    
    def _drone_position_callback(self, msg, drone_id):
        """Callback for drone position and velocity updates"""
        # Extract position
        position = msg.pose.pose.position
        self.drone_positions[drone_id] = [position.x, position.y, position.z]
        
        # Extract velocity
        velocity = msg.twist.twist.linear
        self.drone_velocities[drone_id] = [velocity.x, velocity.y, velocity.z]
        
        # Update timestamp
        self.drone_last_update[drone_id] = self.get_clock().now()
        
        # Debug logging for first few position updates per drone
        if not hasattr(self, '_position_debug_counter'):
            self._position_debug_counter = {}
        if drone_id not in self._position_debug_counter:
            self._position_debug_counter[drone_id] = 0
        
        if self._position_debug_counter[drone_id] < 3:  # Log first 3 position updates
            vel_magnitude = np.linalg.norm(self.drone_velocities[drone_id])
            self.get_logger().info(f'Safety monitoring {drone_id}: pos=[{position.x:.2f}, {position.y:.2f}, {position.z:.2f}], '
                                 f'vel_mag={vel_magnitude:.2f}m/s')
            self._position_debug_counter[drone_id] += 1
    
    def _safety_check_loop(self):
        """Main safety monitoring loop - runs at high frequency"""
        # Check for stale data
        self._check_communication_health()
        
        # Perform collision detection
        self.collision_pairs = self._detect_collisions()
        
        # Assess overall system safety
        previous_safe = self.system_safe
        self.system_safe = len(self.collision_pairs) == 0
        
        # Handle safety state changes
        if not self.system_safe and previous_safe:
            self._handle_collision_risk()
        elif self.system_safe and not previous_safe:
            self._handle_safety_restored()
    
    def _check_communication_health(self):
        """Enhanced communication health monitoring with critical failure detection"""
        current_time = self.get_clock().now()
        timeout_threshold = 1.0  # seconds
        critical_threshold = 3.0  # seconds - trigger emergency protocols
        
        unhealthy_drones = []
        critical_drones = []
        
        for drone_id in self.drone_names:
            if drone_id in self.drone_last_update:
                time_since_update = (current_time - self.drone_last_update[drone_id]).nanoseconds / 1e9
                
                if time_since_update > critical_threshold:
                    critical_drones.append(drone_id)
                    self.get_logger().error(f'CRITICAL: Communication lost with {drone_id}: {time_since_update:.1f}s')
                elif time_since_update > timeout_threshold:
                    unhealthy_drones.append(drone_id)
                    self.get_logger().warning(f'Communication timeout with {drone_id}: {time_since_update:.1f}s')
            else:
                # No communication ever received from this drone
                unhealthy_drones.append(drone_id)
                self.get_logger().warning(f'No communication received from {drone_id}')
        
        # Critical system check: too many drones offline
        total_offline = len(unhealthy_drones) + len(critical_drones)
        if total_offline >= self.num_drones // 2:
            self.get_logger().error(f'CRITICAL: Communication lost with {total_offline}/{self.num_drones} drones')
            # Could trigger formation-wide emergency protocols here
    
    def _detect_collisions(self):
        """Detect potential collisions between all drone pairs"""
        collision_risks = []
        
        drone_ids = list(self.drone_positions.keys())
        
        # Check all pairs of drones
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                drone1_id = drone_ids[i]
                drone2_id = drone_ids[j]
                
                if drone1_id in self.drone_positions and drone2_id in self.drone_positions:
                    risk = self._calculate_collision_risk(drone1_id, drone2_id)
                    if risk > 0:
                        collision_risks.append((drone1_id, drone2_id, risk))
        
        return collision_risks
    
    def _calculate_collision_risk(self, drone1_id, drone2_id):
        """Calculate collision risk between two drones"""
        pos1 = np.array(self.drone_positions[drone1_id])
        pos2 = np.array(self.drone_positions[drone2_id])
        
        # Current distance
        current_distance = np.linalg.norm(pos2 - pos1)
        
        # Immediate collision risk
        if current_distance < self.safety_radius:
            return 3  # CRITICAL - immediate collision risk
        
        # Check predicted collision based on velocities
        if drone1_id in self.drone_velocities and drone2_id in self.drone_velocities:
            vel1 = np.array(self.drone_velocities[drone1_id])
            vel2 = np.array(self.drone_velocities[drone2_id])
            
            # Predict future positions
            predicted_distance = self._predict_minimum_distance(pos1, vel1, pos2, vel2)
            
            if predicted_distance < self.safety_radius:
                return 2  # HIGH - predicted collision
            elif predicted_distance < self.warning_radius:
                return 1  # MEDIUM - collision warning
        
        # Warning based on current distance only
        if current_distance < self.warning_radius:
            return 1  # MEDIUM - proximity warning
        
        return 0  # No collision risk
    
    def _predict_minimum_distance(self, pos1, vel1, pos2, vel2):
        """Predict minimum distance between two drones over prediction time horizon"""
        # Relative position and velocity
        rel_pos = pos2 - pos1
        rel_vel = vel2 - vel1
        
        # If relative velocity is very small, use current distance
        if np.linalg.norm(rel_vel) < 0.1:
            return np.linalg.norm(rel_pos)
        
        # Time when drones are closest (derivative of distance = 0)
        t_closest = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        
        # Clamp to prediction horizon
        t_closest = max(0, min(t_closest, self.prediction_time))
        
        # Calculate minimum distance
        future_rel_pos = rel_pos + rel_vel * t_closest
        min_distance = np.linalg.norm(future_rel_pos)
        
        return min_distance
    
    def _handle_collision_risk(self):
        """Handle detected collision risks"""
        # Find highest risk level
        max_risk = max(risk for _, _, risk in self.collision_pairs)
        
        if max_risk >= 3:  # CRITICAL
            self._trigger_emergency_stop()
        elif max_risk >= 2:  # HIGH
            self._trigger_collision_warning()
        else:  # MEDIUM
            self._trigger_proximity_warning()
    
    def _trigger_emergency_stop(self):
        """Trigger immediate emergency stop for at-risk drones"""
        if not self.emergency_active:
            self.emergency_active = True
            
            # Get all drones involved in critical collisions
            critical_drones = set()
            for drone1, drone2, risk in self.collision_pairs:
                if risk >= 3:
                    critical_drones.add(drone1)
                    critical_drones.add(drone2)
            
            stop_msg = String()
            stop_msg.data = f"EMERGENCY_STOP: {','.join(critical_drones)}"
            self.emergency_stop_pub.publish(stop_msg)
            
            self.get_logger().error(f'EMERGENCY STOP triggered for drones: {list(critical_drones)}')
    
    def _trigger_collision_warning(self):
        """Trigger collision warning for high-risk situations"""
        warning_drones = set()
        for drone1, drone2, risk in self.collision_pairs:
            if risk >= 2:
                warning_drones.add(drone1)
                warning_drones.add(drone2)
        
        warning_msg = String()
        warning_msg.data = f"COLLISION_WARNING: {','.join(warning_drones)}"
        self.collision_warning_pub.publish(warning_msg)
        
        self.get_logger().warning(f'Collision warning for drones: {list(warning_drones)}')
    
    def _trigger_proximity_warning(self):
        """Trigger proximity warning for medium-risk situations"""
        proximity_drones = set()
        for drone1, drone2, risk in self.collision_pairs:
            if risk >= 1:
                proximity_drones.add(drone1)
                proximity_drones.add(drone2)
        
        self.get_logger().info(f'Proximity warning for drones: {list(proximity_drones)}')
    
    def _handle_safety_restored(self):
        """Handle restoration of safe conditions"""
        if self.emergency_active:
            self.emergency_active = False
            self.get_logger().info('Safety restored - emergency conditions cleared')
    
    def _publish_safety_status(self):
        """Publish overall safety status"""
        status_msg = String()
        
        if self.emergency_active:
            status_msg.data = "EMERGENCY"
        elif not self.system_safe:
            status_msg.data = "WARNING"
        else:
            status_msg.data = "SAFE"
        
        self.safety_status_pub.publish(status_msg)
    
    def _publish_safety_visualization(self):
        """Publish safety visualization markers for RViz"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add safety radius visualization for each drone
        for i, (drone_id, position) in enumerate(self.drone_positions.items()):
            # Safety radius marker
            safety_marker = Marker()
            safety_marker.header.frame_id = "map"
            safety_marker.header.stamp = self.get_clock().now().to_msg()
            safety_marker.id = i * 2
            safety_marker.type = Marker.SPHERE
            safety_marker.action = Marker.ADD
            
            safety_marker.pose.position.x = position[0]
            safety_marker.pose.position.y = position[1]
            safety_marker.pose.position.z = position[2]
            safety_marker.pose.orientation.w = 1.0
            
            safety_marker.scale.x = self.safety_radius * 2
            safety_marker.scale.y = self.safety_radius * 2
            safety_marker.scale.z = self.safety_radius * 2
            
            # Color based on safety status
            if any(drone_id in [d1, d2] for d1, d2, r in self.collision_pairs if r >= 3):
                # Critical - Red
                safety_marker.color.r = 1.0
                safety_marker.color.a = 0.5
            elif any(drone_id in [d1, d2] for d1, d2, r in self.collision_pairs if r >= 2):
                # Warning - Orange
                safety_marker.color.r = 1.0
                safety_marker.color.g = 0.5
                safety_marker.color.a = 0.3
            elif any(drone_id in [d1, d2] for d1, d2, r in self.collision_pairs if r >= 1):
                # Proximity - Yellow
                safety_marker.color.r = 1.0
                safety_marker.color.g = 1.0
                safety_marker.color.a = 0.2
            else:
                # Safe - Green
                safety_marker.color.g = 1.0
                safety_marker.color.a = 0.1
            
            marker_array.markers.append(safety_marker)
        
        self.safety_visualization_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    safety_monitor = SafetyMonitor()
    
    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()