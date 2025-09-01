#!/usr/bin/env python3
"""
Formation Commander Node

Centralized formation planning and coordination for multi-drone systems.
Generates sphere formations and manages fleet-wide mission execution.

Author: Finn Picotoli
Date: 2025-08-22
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import String, Float32, Int32
from nav_msgs.msg import Odometry


class FormationCommander(Node):
    def __init__(self):
        super().__init__('formation_commander')
        
        # Formation parameters (will be set from parameters)
        self.formation_speed = 2.0  # m/s
        self.golden_ratio = (1 + math.sqrt(5)) / 2
        
        # Formation state
        self.formation_active = False
        self.formation_assembled = False
        self.drone_positions = {}  # drone_id: [x, y, z]
        self.target_positions = {}  # drone_id: [x, y, z]
        
        # Publishers
        self.formation_status_pub = self.create_publisher(
            String, '/formation/status', 10)
        
        # Drone command publishers (will be created dynamically)
        self.drone_command_pubs = {}
        
        # Subscribers for drone positions
        self.drone_position_subs = {}
        
        # Service clients for emergency control
        # (Will be implemented when emergency_controller is ready)
        
        # Parameters
        self.declare_parameter('drone_names', ["iris_9002", "iris_9012", "iris_9022", "iris_9032", "iris_9042"])
        self.declare_parameter('sphere_radius', 10.0)
        self.declare_parameter('formation_height', 20.0)
        self.declare_parameter('formation_center', [0.0, 0.0, 20.0])
        self.declare_parameter('formation_tolerance', 2.0)
        
        # Get configured drone names
        self.drone_names = self.get_parameter('drone_names').value
        self.num_drones = len(self.drone_names)
        self.sphere_radius = self.get_parameter('sphere_radius').value
        self.sphere_center = self.get_parameter('formation_center').value
        self.formation_tolerance = self.get_parameter('formation_tolerance').value
        
        # Initialize drone communication
        self._setup_drone_communications()
        
        # Generate initial formation
        self._generate_sphere_formation()
        
        # Timers
        self.formation_timer = self.create_timer(0.1, self._formation_control_loop)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        self.get_logger().info(f'Formation Commander initialized for {self.num_drones} drones')
        self.get_logger().info(f'Sphere formation: radius={self.sphere_radius}m, center={self.sphere_center}')

    def _setup_drone_communications(self):
        """Setup publishers and subscribers for each configured drone"""
        for drone_id in self.drone_names:
            # Command publisher for each drone
            self.drone_command_pubs[drone_id] = self.create_publisher(
                PoseStamped, f'/{drone_id}/formation/target_pose', 10)
            
            # Position subscriber for each drone
            self.drone_position_subs[drone_id] = self.create_subscription(
                Odometry, f'/{drone_id}/odom', 
                lambda msg, drone=drone_id: self._drone_position_callback(msg, drone), 10)
    
    def _drone_position_callback(self, msg, drone_id):
        """Callback for drone position updates"""
        position = msg.pose.pose.position
        self.drone_positions[drone_id] = [position.x, position.y, position.z]
    
    def _generate_sphere_formation(self):
        """Generate optimal sphere formation using Fibonacci spiral distribution"""
        self.target_positions = {}
        
        for i, drone_id in enumerate(self.drone_names):
            # Fibonacci spiral distribution for uniform sphere coverage
            theta = 2 * math.pi * i / self.golden_ratio
            phi = math.acos(1 - 2 * i / self.num_drones)
            
            # Convert spherical to Cartesian coordinates
            x = self.sphere_center[0] + self.sphere_radius * math.sin(phi) * math.cos(theta)
            y = self.sphere_center[1] + self.sphere_radius * math.sin(phi) * math.sin(theta)
            z = self.sphere_center[2] + self.sphere_radius * math.cos(phi)
            
            self.target_positions[drone_id] = [x, y, z]
            
            self.get_logger().debug(f'{drone_id} target position: [{x:.2f}, {y:.2f}, {z:.2f}]')
    
    def _formation_control_loop(self):
        """Main formation control loop"""
        if not self.formation_active:
            return
        
        # Send target positions to all drones
        for drone_id, target_pos in self.target_positions.items():
            if drone_id in self.drone_command_pubs:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                
                pose_msg.pose.position.x = target_pos[0]
                pose_msg.pose.position.y = target_pos[1]
                pose_msg.pose.position.z = target_pos[2]
                
                # Simple orientation (facing center of sphere)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                
                self.drone_command_pubs[drone_id].publish(pose_msg)
        
        # Check formation assembly status
        self._check_formation_status()
    
    def _check_formation_status(self):
        """Check if drones have reached their target positions"""
        if len(self.drone_positions) < self.num_drones:
            return  # Not all drones reporting positions yet
        
        drones_in_position = 0
        
        for drone_id, current_pos in self.drone_positions.items():
            if drone_id in self.target_positions:
                target_pos = self.target_positions[drone_id]
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, target_pos)))
                
                if distance < self.formation_tolerance:
                    drones_in_position += 1
        
        # Update formation status
        old_status = self.formation_assembled
        self.formation_assembled = (drones_in_position >= self.num_drones * 0.8)  # 80% threshold
        
        if self.formation_assembled and not old_status:
            self.get_logger().info('Formation assembly completed!')
        elif not self.formation_assembled and old_status:
            self.get_logger().warning('Formation integrity lost!')
    
    def _publish_status(self):
        """Publish formation status"""
        status_msg = String()
        if not self.formation_active:
            status_msg.data = "IDLE"
        elif self.formation_assembled:
            status_msg.data = "ASSEMBLED"
        else:
            status_msg.data = "ASSEMBLING"
        
        self.formation_status_pub.publish(status_msg)
    
    def start_formation(self):
        """Start formation assembly"""
        self.formation_active = True
        self.formation_assembled = False
        self._generate_sphere_formation()
        self.get_logger().info('Formation assembly started')
    
    def stop_formation(self):
        """Stop formation and enter idle state"""
        self.formation_active = False
        self.formation_assembled = False
        self.get_logger().info('Formation stopped')
    
    def update_formation_parameters(self, radius=None, center=None, drone_names=None):
        """Update formation parameters and regenerate formation"""
        if radius is not None:
            self.sphere_radius = radius
        if center is not None:
            self.sphere_center = center
        if drone_names is not None and drone_names != self.drone_names:
            self.drone_names = drone_names
            self.num_drones = len(self.drone_names)
            self._setup_drone_communications()
        
        if self.formation_active:
            self._generate_sphere_formation()
            self.get_logger().info(f'Formation updated: radius={self.sphere_radius}, center={self.sphere_center}, drones={self.drone_names}')
    
    def move_formation(self, delta_x, delta_y, delta_z):
        """Move entire formation by specified offset"""
        self.sphere_center[0] += delta_x
        self.sphere_center[1] += delta_y
        self.sphere_center[2] += delta_z
        
        if self.formation_active:
            self._generate_sphere_formation()
            self.get_logger().info(f'Formation moved to center: {self.sphere_center}')


def main(args=None):
    rclpy.init(args=args)
    
    formation_commander = FormationCommander()
    
    # Start formation automatically (for testing)
    formation_commander.create_timer(2.0, lambda: formation_commander.start_formation())
    
    try:
        rclpy.spin(formation_commander)
    except KeyboardInterrupt:
        pass
    finally:
        formation_commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()