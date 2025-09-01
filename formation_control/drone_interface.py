#!/usr/bin/env python3
"""
Drone Interface Node

Simple telemetry and command interface for individual drones.
Handles position reporting to ground station and waypoint following.

Author: Finn Picotoli
Date: 2025-08-22
"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
from ardupilot_msgs.srv import ArmMotors, ModeSwitch
from ardupilot_msgs.msg import Status, GlobalPosition


class DroneInterface(Node):
    def __init__(self, drone_id="drone_0"):
        super().__init__(f'{drone_id}_interface')
        
        self.drone_id = drone_id
        
        # Drone state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.target_position = [0.0, 0.0, 0.0]
        self.battery_level = 100.0
        self.connection_status = "DISCONNECTED"
        self.flight_mode = "UNKNOWN"
        self.armed = False
        
        # Control parameters
        self.position_tolerance = 1.0  # meters
        self.max_velocity = 3.0  # m/s
        self.emergency_mode = False
        
        # Publishers - send data to ground station
        self.odom_pub = self.create_publisher(
            Odometry, f'/{drone_id}/odom', 10)
        
        self.status_pub = self.create_publisher(
            String, f'/{drone_id}/status', 10)
        
        self.battery_pub = self.create_publisher(
            BatteryState, f'/{drone_id}/battery', 10)
        
        # Publishers - send commands to ArduPilot
        self.position_cmd_pub = self.create_publisher(
            GlobalPosition, f'/{drone_id}/ap/cmd_gps_pose', 10)
        
        self.velocity_cmd_pub = self.create_publisher(
            TwistStamped, f'/{drone_id}/ap/cmd_vel', 10)
        
        # Subscribers - receive from ground station
        self.formation_target_sub = self.create_subscription(
            PoseStamped, f'/{drone_id}/formation/target_pose',
            self._formation_target_callback, 10)
        
        self.emergency_stop_sub = self.create_subscription(
            TwistStamped, f'/{drone_id}/emergency/stop',
            self._emergency_stop_callback, 10)
        
        self.emergency_hover_sub = self.create_subscription(
            PoseStamped, f'/{drone_id}/emergency/hover',
            self._emergency_hover_callback, 10)
        
        # Subscribers - receive from ArduPilot
        self.ardupilot_pose_sub = self.create_subscription(
            PoseStamped, f'/{drone_id}/ap/pose/filtered',
            self._ardupilot_pose_callback, 10)
        
        self.ardupilot_velocity_sub = self.create_subscription(
            TwistStamped, f'/{drone_id}/ap/twist/filtered',
            self._ardupilot_velocity_callback, 10)
        
        self.ardupilot_state_sub = self.create_subscription(
            Status, f'/{drone_id}/ap/status',
            self._ardupilot_status_callback, 10)
        
        self.ardupilot_battery_sub = self.create_subscription(
            BatteryState, f'/{drone_id}/ap/battery',
            self._ardupilot_battery_callback, 10)
        
        # Service clients for ArduPilot control
        self.arming_client = self.create_client(
            ArmMotors, f'/{drone_id}/ap/arm_motors')
        
        self.mode_client = self.create_client(
            ModeSwitch, f'/{drone_id}/ap/mode_switch')
        
        # Parameters
        self.declare_parameter('drone_id', drone_id)
        self.declare_parameter('position_tolerance', 1.0)
        self.declare_parameter('max_velocity', 3.0)
        
        self.drone_id = self.get_parameter('drone_id').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Timers
        self.telemetry_timer = self.create_timer(0.1, self._publish_telemetry)  # 10Hz
        self.command_timer = self.create_timer(0.05, self._execute_commands)  # 20Hz
        self.status_timer = self.create_timer(1.0, self._publish_status)  # 1Hz
        
        self.get_logger().info(f'Drone Interface initialized for {self.drone_id}')

    def _formation_target_callback(self, msg):
        """Receive formation target position from ground station"""
        if not self.emergency_mode:
            self.target_position = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
            
            self.get_logger().debug(f'New formation target: {self.target_position}')
    
    def _emergency_stop_callback(self, msg):
        """Receive emergency stop command"""
        self.emergency_mode = True
        
        # Immediately publish stop command to ArduPilot
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = "base_link"
        
        # Zero velocity
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.linear.y = 0.0
        stop_cmd.twist.linear.z = 0.0
        stop_cmd.twist.angular.x = 0.0
        stop_cmd.twist.angular.y = 0.0
        stop_cmd.twist.angular.z = 0.0
        
        self.velocity_cmd_pub.publish(stop_cmd)
        
        self.get_logger().error(f'{self.drone_id} - EMERGENCY STOP EXECUTED')
    
    def _emergency_hover_callback(self, msg):
        """Receive emergency hover command"""
        self.emergency_mode = True
        
        # Override target with hover position
        self.target_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        self.get_logger().warning(f'{self.drone_id} - Emergency hover at {self.target_position}')
    
    def _ardupilot_pose_callback(self, msg):
        """Receive current position from ArduPilot"""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
    
    def _ardupilot_velocity_callback(self, msg):
        """Receive current velocity from ArduPilot"""
        self.current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
    
    def _ardupilot_status_callback(self, msg):
        """Receive ArduPilot status information"""
        self.connection_status = "CONNECTED"  # ArduPilot Status doesn't have connected field
        self.flight_mode = msg.mode
        self.armed = msg.armed
    
    def _ardupilot_battery_callback(self, msg):
        """Receive battery information from ArduPilot"""
        self.battery_level = msg.percentage * 100.0  # Convert to percentage
    
    def _publish_telemetry(self):
        """Publish telemetry data to ground station"""
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = f"{self.drone_id}_base_link"
        
        # Position
        odom_msg.pose.pose.position.x = self.current_position[0]
        odom_msg.pose.pose.position.y = self.current_position[1]
        odom_msg.pose.pose.position.z = self.current_position[2]
        odom_msg.pose.pose.orientation.w = 1.0  # Simplified orientation
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.linear.z = self.current_velocity[2]
        
        self.odom_pub.publish(odom_msg)
        
        # Publish battery status
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.percentage = self.battery_level / 100.0
        battery_msg.voltage = 12.0  # Simplified
        
        self.battery_pub.publish(battery_msg)
    
    def _execute_commands(self):
        """Execute movement commands to reach target position"""
        if self.emergency_mode:
            return  # Don't move during emergency
        
        # Calculate distance to target
        distance = self._calculate_distance(self.current_position, self.target_position)
        
        if distance > self.position_tolerance:
            # Calculate position command using GlobalPosition
            position_cmd = GlobalPosition()
            position_cmd.header.stamp = self.get_clock().now().to_msg()
            position_cmd.header.frame_id = "map"
            
            # Set coordinate frame for global positioning
            position_cmd.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT
            
            # TODO: Convert local coordinates to lat/lon/alt
            # For now, using placeholder values - this needs proper coordinate conversion
            position_cmd.latitude = 40.08370 + (self.target_position[0] * 0.00001)  # Rough conversion
            position_cmd.longitude = -105.21740 + (self.target_position[1] * 0.00001)  # Rough conversion  
            position_cmd.altitude = 1630.0 + self.target_position[2]  # Altitude offset
            
            # Set type mask to ignore velocity and acceleration
            position_cmd.type_mask = (
                GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | 
                GlobalPosition.IGNORE_VZ | GlobalPosition.IGNORE_AFX | 
                GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ |
                GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE
            )
            
            self.position_cmd_pub.publish(position_cmd)
    
    def _calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
    
    def _publish_status(self):
        """Publish drone status information"""
        status_msg = String()
        
        distance_to_target = self._calculate_distance(self.current_position, self.target_position)
        
        status_data = {
            'connection': self.connection_status,
            'mode': self.flight_mode,
            'armed': self.armed,
            'battery': f'{self.battery_level:.1f}%',
            'position': f'[{self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}]',
            'target_distance': f'{distance_to_target:.2f}m',
            'emergency': self.emergency_mode
        }
        
        status_msg.data = str(status_data)
        self.status_pub.publish(status_msg)
    
    def clear_emergency(self):
        """Clear emergency mode and resume normal operation"""
        self.emergency_mode = False
        self.get_logger().info(f'{self.drone_id} - Emergency mode cleared')
    
    def set_target_position(self, x, y, z):
        """Manually set target position (for testing)"""
        if not self.emergency_mode:
            self.target_position = [x, y, z]
            self.get_logger().info(f'{self.drone_id} - Manual target set: {self.target_position}')


def main(args=None):
    rclpy.init(args=args)
    
    # Get drone ID from command line argument or use default
    import sys
    drone_id = "drone_0"
    if len(sys.argv) > 1:
        drone_id = sys.argv[1]
    
    drone_interface = DroneInterface(drone_id)
    
    try:
        rclpy.spin(drone_interface)
    except KeyboardInterrupt:
        pass
    finally:
        drone_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()