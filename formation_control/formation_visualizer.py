#!/usr/bin/env python3
"""
Formation Visualizer Node

RViz visualization for multi-drone formation control system.
Displays formation status, drone positions, safety zones, and trajectories.

MIGRATED TO ARDUPILOT DDS:
- Uses ArduPilot DDS topics: /{drone_id}/ap/pose/filtered
- Subscribes to PoseStamped messages instead of Odometry
- Updated drone naming: iris_9002, iris_9012, iris_9022, iris_9032, iris_9042
- No longer requires MAVROS middleware

Author: Finn Picotoli
Date: 2025-08-22
Updated: 2025-08-27 (ArduPilot DDS migration)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Point, Vector3, PoseStamped
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from formation_control.formation_math import SphereFormation


class FormationVisualizer(Node):
    def __init__(self):
        super().__init__('formation_visualizer')
        
        # Visualization state
        self.num_drones = 5
        self.drone_positions = {}  # drone_id: [x, y, z]
        self.target_positions = {}  # drone_id: [x, y, z]
        self.formation_center = [0.0, 0.0, 20.0]
        self.formation_radius = 10.0
        self.formation_status = "IDLE"
        
        # Visualization settings
        self.drone_marker_scale = 1.0
        self.safety_radius_scale = 3.0
        self.show_trajectories = True
        self.show_safety_zones = True
        self.show_formation_structure = True
        
        # Publishers
        self.formation_markers_pub = self.create_publisher(
            MarkerArray, '/formation/visualization/markers', 10)
        
        self.drone_markers_pub = self.create_publisher(
            MarkerArray, '/formation/visualization/drones', 10)
        
        self.safety_markers_pub = self.create_publisher(
            MarkerArray, '/formation/visualization/safety', 10)
        
        # Subscribers
        self.formation_status_sub = self.create_subscription(
            String, '/formation/status', self._formation_status_callback, 10)
        
        # Drone position subscribers (will be created dynamically)
        self.drone_position_subs = {}
        
        # Parameters
        self.declare_parameter('num_drones', 5)
        self.declare_parameter('drone_marker_scale', 1.0)
        self.declare_parameter('show_trajectories', True)
        self.declare_parameter('show_safety_zones', True)
        
        self.num_drones = self.get_parameter('num_drones').value
        self.drone_marker_scale = self.get_parameter('drone_marker_scale').value
        self.show_trajectories = self.get_parameter('show_trajectories').value
        self.show_safety_zones = self.get_parameter('show_safety_zones').value
        
        # Setup drone monitoring
        self._setup_drone_monitoring()
        
        # Initialize target positions for visualization
        self._generate_target_positions()
        
        # Timers
        self.visualization_timer = self.create_timer(0.1, self._publish_visualizations)  # 10Hz
        
        self.get_logger().info(f'Formation Visualizer initialized for {self.num_drones} drones')

    def _setup_drone_monitoring(self):
        """Setup subscribers for all drone position data using ArduPilot DDS topics"""
        # Use proper drone naming convention from multi-drone setup
        drone_names = ['iris_9002', 'iris_9012', 'iris_9022', 'iris_9032', 'iris_9042']
        
        for i in range(self.num_drones):
            drone_id = drone_names[i] if i < len(drone_names) else f"iris_90{i}2"
            topic_name = f'/{drone_id}/ap/pose/filtered'
            
            # Subscribe to ArduPilot DDS pose topic (CORRECTED: no /rt/ prefix)
            self.drone_position_subs[drone_id] = self.create_subscription(
                PoseStamped, topic_name,
                lambda msg, drone=drone_id: self._drone_position_callback(msg, drone), 10)
            
            self.get_logger().info(f'Subscribed to DDS topic: {topic_name} for drone {drone_id}')
    
    def _drone_position_callback(self, msg, drone_id):
        """Callback for drone position updates from ArduPilot DDS PoseStamped"""
        position = msg.pose.position  # PoseStamped has different structure than Odometry
        self.drone_positions[drone_id] = [position.x, position.y, position.z]
        
        # Debug logging (first few messages only to avoid spam)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = {}
        if drone_id not in self._debug_counter:
            self._debug_counter[drone_id] = 0
        
        if self._debug_counter[drone_id] < 3:  # Log first 3 position updates per drone
            self.get_logger().info(f'DDS position update for {drone_id}: '
                                 f'x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}')
            self._debug_counter[drone_id] += 1
    
    def _formation_status_callback(self, msg):
        """Callback for formation status updates"""
        self.formation_status = msg.data
    
    def _generate_target_positions(self):
        """Generate target positions for formation visualization"""
        sphere_gen = SphereFormation()
        positions = sphere_gen.fibonacci_spiral_distribution(
            self.num_drones, self.formation_radius, np.array(self.formation_center))
        
        # Use proper drone naming convention from multi-drone setup
        drone_names = ['iris_9002', 'iris_9012', 'iris_9022', 'iris_9032', 'iris_9042']
        
        for i, pos in enumerate(positions):
            drone_id = drone_names[i] if i < len(drone_names) else f"iris_90{i}2"
            self.target_positions[drone_id] = pos.tolist()
    
    def _publish_visualizations(self):
        """Publish all visualization markers"""
        # Formation structure visualization
        if self.show_formation_structure:
            self._publish_formation_markers()
        
        # Drone position and status visualization
        self._publish_drone_markers()
        
        # Safety zone visualization
        if self.show_safety_zones:
            self._publish_safety_markers()
    
    def _publish_formation_markers(self):
        """Publish formation structure visualization"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Formation center marker
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        
        center_marker.pose.position.x = self.formation_center[0]
        center_marker.pose.position.y = self.formation_center[1]
        center_marker.pose.position.z = self.formation_center[2]
        center_marker.pose.orientation.w = 1.0
        
        center_marker.scale.x = 2.0
        center_marker.scale.y = 2.0
        center_marker.scale.z = 2.0
        
        center_marker.color.r = 0.0
        center_marker.color.g = 0.0
        center_marker.color.b = 1.0
        center_marker.color.a = 0.7
        
        marker_array.markers.append(center_marker)
        
        # Formation sphere wireframe
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        sphere_marker.pose.position.x = self.formation_center[0]
        sphere_marker.pose.position.y = self.formation_center[1]
        sphere_marker.pose.position.z = self.formation_center[2]
        sphere_marker.pose.orientation.w = 1.0
        
        sphere_marker.scale.x = self.formation_radius * 2
        sphere_marker.scale.y = self.formation_radius * 2
        sphere_marker.scale.z = self.formation_radius * 2
        
        sphere_marker.color.r = 0.5
        sphere_marker.color.g = 0.5
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 0.1
        
        marker_array.markers.append(sphere_marker)
        
        # Target position markers
        for i, (drone_id, target_pos) in enumerate(self.target_positions.items()):
            target_marker = Marker()
            target_marker.header.frame_id = "map"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.id = 10 + i
            target_marker.type = Marker.CUBE
            target_marker.action = Marker.ADD
            
            target_marker.pose.position.x = target_pos[0]
            target_marker.pose.position.y = target_pos[1]
            target_marker.pose.position.z = target_pos[2]
            target_marker.pose.orientation.w = 1.0
            
            target_marker.scale.x = 0.5
            target_marker.scale.y = 0.5
            target_marker.scale.z = 0.5
            
            target_marker.color.r = 1.0
            target_marker.color.g = 1.0
            target_marker.color.b = 0.0
            target_marker.color.a = 0.6
            
            marker_array.markers.append(target_marker)
        
        self.formation_markers_pub.publish(marker_array)
    
    def _publish_drone_markers(self):
        """Publish drone position and status visualization"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, (drone_id, current_pos) in enumerate(self.drone_positions.items()):
            # Drone position marker
            drone_marker = Marker()
            drone_marker.header.frame_id = "map"
            drone_marker.header.stamp = self.get_clock().now().to_msg()
            drone_marker.id = i * 3
            drone_marker.type = Marker.MESH_RESOURCE
            drone_marker.action = Marker.ADD
            
            drone_marker.mesh_resource = "package://ardupilot_gazebo/models/iris_with_gimbal/meshes/iris.dae"
            
            drone_marker.pose.position.x = current_pos[0]
            drone_marker.pose.position.y = current_pos[1]
            drone_marker.pose.position.z = current_pos[2]
            drone_marker.pose.orientation.w = 1.0
            
            drone_marker.scale.x = self.drone_marker_scale
            drone_marker.scale.y = self.drone_marker_scale
            drone_marker.scale.z = self.drone_marker_scale
            
            # Color based on formation status
            if self.formation_status == "ASSEMBLED":
                drone_marker.color.r = 0.0
                drone_marker.color.g = 1.0
                drone_marker.color.b = 0.0
            elif self.formation_status == "ASSEMBLING":
                drone_marker.color.r = 1.0
                drone_marker.color.g = 1.0
                drone_marker.color.b = 0.0
            else:
                drone_marker.color.r = 0.5
                drone_marker.color.g = 0.5
                drone_marker.color.b = 0.5
            
            drone_marker.color.a = 1.0
            
            marker_array.markers.append(drone_marker)
            
            # Drone ID text
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = i * 3 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = current_pos[0]
            text_marker.pose.position.y = current_pos[1]
            text_marker.pose.position.z = current_pos[2] + 2.0  # Above drone
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 1.5  # Text height
            text_marker.text = drone_id  # Already using correct iris_90XX names
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
            
            # Connection line to target position
            if drone_id in self.target_positions and self.show_trajectories:
                target_pos = self.target_positions[drone_id]
                
                line_marker = Marker()
                line_marker.header.frame_id = "map"
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.id = i * 3 + 2
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                
                # Start point (current position)
                start_point = Point()
                start_point.x = current_pos[0]
                start_point.y = current_pos[1]
                start_point.z = current_pos[2]
                
                # End point (target position)
                end_point = Point()
                end_point.x = target_pos[0]
                end_point.y = target_pos[1]
                end_point.z = target_pos[2]
                
                line_marker.points = [start_point, end_point]
                
                line_marker.scale.x = 0.1  # Line width
                
                line_marker.color.r = 1.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
                line_marker.color.a = 0.5
                
                marker_array.markers.append(line_marker)
        
        self.drone_markers_pub.publish(marker_array)
    
    def _publish_safety_markers(self):
        """Publish safety zone visualization"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Safety radius visualization for each drone
        for i, (drone_id, position) in enumerate(self.drone_positions.items()):
            safety_marker = Marker()
            safety_marker.header.frame_id = "map"
            safety_marker.header.stamp = self.get_clock().now().to_msg()
            safety_marker.id = i
            safety_marker.type = Marker.SPHERE
            safety_marker.action = Marker.ADD
            
            safety_marker.pose.position.x = position[0]
            safety_marker.pose.position.y = position[1]
            safety_marker.pose.position.z = position[2]
            safety_marker.pose.orientation.w = 1.0
            
            safety_marker.scale.x = self.safety_radius_scale * 2
            safety_marker.scale.y = self.safety_radius_scale * 2
            safety_marker.scale.z = self.safety_radius_scale * 2
            
            # Check if this drone has any collision warnings
            # (This would be integrated with safety monitor in full implementation)
            safety_marker.color.r = 0.0
            safety_marker.color.g = 1.0
            safety_marker.color.b = 0.0
            safety_marker.color.a = 0.1
            
            marker_array.markers.append(safety_marker)
        
        self.safety_markers_pub.publish(marker_array)
    
    def update_formation_parameters(self, center, radius):
        """Update formation parameters for visualization"""
        self.formation_center = center
        self.formation_radius = radius
        self._generate_target_positions()
        
        self.get_logger().info(f'Formation visualization updated: center={center}, radius={radius}')
    
    def set_visualization_options(self, show_trajectories=None, show_safety_zones=None, 
                                show_formation_structure=None):
        """Update visualization options"""
        if show_trajectories is not None:
            self.show_trajectories = show_trajectories
        if show_safety_zones is not None:
            self.show_safety_zones = show_safety_zones
        if show_formation_structure is not None:
            self.show_formation_structure = show_formation_structure
        
        self.get_logger().info(f'Visualization options updated: trajectories={self.show_trajectories}, '
                              f'safety={self.show_safety_zones}, structure={self.show_formation_structure}')


def main(args=None):
    rclpy.init(args=args)
    
    formation_visualizer = FormationVisualizer()
    
    try:
        rclpy.spin(formation_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        formation_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()