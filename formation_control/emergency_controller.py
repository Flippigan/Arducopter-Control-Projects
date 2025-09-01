#!/usr/bin/env python3
"""
Emergency Controller Node

Handles emergency stop commands and safety interventions for multi-drone formations.
Provides immediate halt capability with <100ms response time.

Author: Finn Picotoli
Date: 2025-08-22
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool
from ardupilot_msgs.srv import ArmMotors, ModeSwitch


class EmergencyController(Node):
    def __init__(self):
        super().__init__('emergency_controller')
        
        # Emergency state
        self.num_drones = 5  # Updated to match 5-drone simulation
        self.emergency_active = False
        self.stopped_drones = set()
        self.drone_emergency_states = {}  # drone_id: emergency_active
        
        # Actual drone names from simulation
        self.drone_names = ["iris_9002", "iris_9012", "iris_9022", "iris_9032", "iris_9042"]
        
        # Publishers for emergency commands
        self.drone_stop_pubs = {}
        self.drone_hover_pubs = {}
        
        # Service clients for ArduPilot emergency control
        self.drone_arm_clients = {}
        self.drone_mode_clients = {}
        
        # Subscribers
        self.emergency_stop_sub = self.create_subscription(
            String, '/safety/emergency_stop', self._emergency_stop_callback, 10)
        
        self.collision_warning_sub = self.create_subscription(
            String, '/safety/collision_warning', self._collision_warning_callback, 10)
        
        # Publishers
        self.emergency_status_pub = self.create_publisher(
            String, '/emergency/status', 10)
        
        # Parameters
        self.declare_parameter('num_drones', 5)
        self.declare_parameter('emergency_timeout', 30.0)  # seconds
        self.declare_parameter('hover_altitude', 10.0)  # meters
        
        self.num_drones = self.get_parameter('num_drones').value
        self.emergency_timeout = self.get_parameter('emergency_timeout').value
        self.hover_altitude = self.get_parameter('hover_altitude').value
        
        # Setup drone communication
        self._setup_drone_communications()
        
        # Timers
        self.status_timer = self.create_timer(0.1, self._publish_emergency_status)
        self.timeout_timer = self.create_timer(1.0, self._check_emergency_timeout)
        
        self.get_logger().info(f'Emergency Controller initialized for {self.num_drones} drones')
    
    def _get_mode_number(self, mode_name):
        """Convert flight mode name to mode number for ArduPilot"""
        # ArduPilot Copter mode numbers
        mode_map = {
            "STABILIZE": 0,
            "ACRO": 1, 
            "ALT_HOLD": 2,
            "AUTO": 3,
            "GUIDED": 4,
            "LOITER": 5,
            "RTL": 6,
            "CIRCLE": 7,
            "LAND": 9,
            "BRAKE": 17,
            "THROW": 18,
            "AVOID_ADSB": 19,
            "GUIDED_NOGPS": 20,
            "SMART_RTL": 21,
            "FLOWHOLD": 22,
            "FOLLOW": 23,
            "ZIGZAG": 24,
            "AUTOROTATE": 25
        }
        return mode_map.get(mode_name, 4)  # Default to GUIDED mode

    def _setup_drone_communications(self):
        """Setup emergency communication channels for all drones"""
        for drone_id in self.drone_names:
            
            # Emergency stop publishers (velocity override)
            self.drone_stop_pubs[drone_id] = self.create_publisher(
                TwistStamped, f'/{drone_id}/emergency/stop', 10)
            
            # Hover command publishers
            self.drone_hover_pubs[drone_id] = self.create_publisher(
                PoseStamped, f'/{drone_id}/emergency/hover', 10)
            
            # ArduPilot service clients for emergency control
            self.drone_arm_clients[drone_id] = self.create_client(
                ArmMotors, f'/{drone_id}/ap/arm_motors')
            
            self.drone_mode_clients[drone_id] = self.create_client(
                ModeSwitch, f'/{drone_id}/ap/mode_switch')
            
            # Initialize emergency state
            self.drone_emergency_states[drone_id] = False
    
    def _emergency_stop_callback(self, msg):
        """Handle emergency stop commands from safety monitor"""
        if "EMERGENCY_STOP:" in msg.data:
            # Parse drone IDs from message
            drone_list_str = msg.data.split("EMERGENCY_STOP:")[1].strip()
            drone_ids = [d.strip() for d in drone_list_str.split(',') if d.strip()]
            
            self.get_logger().error(f'Executing emergency stop for drones: {drone_ids}')
            
            # Execute emergency stop for specified drones
            for drone_id in drone_ids:
                if drone_id in self.drone_names:
                    self._execute_emergency_stop(drone_id)
            
            self.emergency_active = True
    
    def _collision_warning_callback(self, msg):
        """Handle collision warnings from safety monitor"""
        if "COLLISION_WARNING:" in msg.data:
            # Parse drone IDs from message
            drone_list_str = msg.data.split("COLLISION_WARNING:")[1].strip()
            drone_ids = [d.strip() for d in drone_list_str.split(',') if d.strip()]
            
            self.get_logger().warning(f'Collision warning for drones: {drone_ids}')
            
            # Execute precautionary hover for warned drones
            for drone_id in drone_ids:
                if drone_id in self.drone_names:
                    self._execute_precautionary_hover(drone_id)
    
    def _execute_emergency_stop(self, drone_id):
        """Execute immediate emergency stop for a specific drone"""
        if drone_id not in self.drone_stop_pubs:
            self.get_logger().error(f'Unknown drone ID: {drone_id}')
            return
        
        # Mark drone as stopped
        self.stopped_drones.add(drone_id)
        self.drone_emergency_states[drone_id] = True
        
        # Send immediate stop command (zero velocity)
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.header.frame_id = "base_link"
        
        # All velocities to zero
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.linear.y = 0.0
        stop_cmd.twist.linear.z = 0.0
        stop_cmd.twist.angular.x = 0.0
        stop_cmd.twist.angular.y = 0.0
        stop_cmd.twist.angular.z = 0.0
        
        self.drone_stop_pubs[drone_id].publish(stop_cmd)
        
        # Switch to LOITER mode for safety (if ArduPilot)
        self._set_drone_mode(drone_id, "LOITER")
        
        self.get_logger().error(f'Emergency stop executed for {drone_id}')
    
    def _execute_precautionary_hover(self, drone_id):
        """Execute precautionary hover for collision warning"""
        if drone_id not in self.drone_hover_pubs:
            self.get_logger().error(f'Unknown drone ID: {drone_id}')
            return
        
        # Send hover command at current position but safe altitude
        hover_cmd = PoseStamped()
        hover_cmd.header.stamp = self.get_clock().now().to_msg()
        hover_cmd.header.frame_id = "map"
        
        # Hover at safe altitude (simplified - should use current position)
        hover_cmd.pose.position.x = 0.0  # Should get current position
        hover_cmd.pose.position.y = 0.0
        hover_cmd.pose.position.z = self.hover_altitude
        hover_cmd.pose.orientation.w = 1.0
        
        self.drone_hover_pubs[drone_id].publish(hover_cmd)
        
        # Switch to GUIDED mode for hover
        self._set_drone_mode(drone_id, "GUIDED")
        
        self.get_logger().warning(f'Precautionary hover executed for {drone_id}')
    
    def _set_drone_mode(self, drone_id, mode):
        """Set ArduPilot flight mode for emergency control"""
        if drone_id not in self.drone_mode_clients:
            return
        
        if self.drone_mode_clients[drone_id].wait_for_service(timeout_sec=1.0):
            request = ModeSwitch.Request()
            request.mode = self._get_mode_number(mode)
            
            future = self.drone_mode_clients[drone_id].call_async(request)
            future.add_done_callback(
                lambda fut, drone=drone_id, m=mode: self._mode_change_callback(fut, drone, m))
        else:
            self.get_logger().warning(f'Mode service not available for {drone_id}')
    
    def _mode_change_callback(self, future, drone_id, mode):
        """Callback for mode change service response"""
        try:
            response = future.result()
            if response.status:
                self.get_logger().info(f'Mode changed to {mode} (#{response.curr_mode}) for {drone_id}')
            else:
                self.get_logger().error(f'Failed to change mode to {mode} for {drone_id}')
        except Exception as e:
            self.get_logger().error(f'Mode change service failed for {drone_id}: {e}')
    
    def _check_emergency_timeout(self):
        """Check for emergency timeout and automatic recovery"""
        # Implementation for automatic recovery after timeout
        # For now, manual recovery only
        pass
    
    def _publish_emergency_status(self):
        """Publish emergency controller status"""
        status_msg = String()
        
        if self.emergency_active:
            num_stopped = len(self.stopped_drones)
            status_msg.data = f"EMERGENCY_ACTIVE: {num_stopped} drones stopped"
        else:
            status_msg.data = "READY"
        
        self.emergency_status_pub.publish(status_msg)
    
    def clear_emergency(self, drone_id=None):
        """Clear emergency state for specific drone or all drones"""
        if drone_id:
            if drone_id in self.stopped_drones:
                self.stopped_drones.remove(drone_id)
                self.drone_emergency_states[drone_id] = False
                self.get_logger().info(f'Emergency cleared for {drone_id}')
        else:
            # Clear all emergency states
            self.stopped_drones.clear()
            for drone in self.drone_emergency_states:
                self.drone_emergency_states[drone] = False
            self.emergency_active = False
            self.get_logger().info('All emergency states cleared')
    
    def emergency_land_all(self):
        """Command all drones to emergency land"""
        self.get_logger().error('Emergency landing all drones')
        
        for drone_id in self.drone_names:
            self._set_drone_mode(drone_id, "LAND")
        
        self.emergency_active = True
    
    def return_to_launch_all(self):
        """Command all drones to return to launch"""
        self.get_logger().warning('Return to launch for all drones')
        
        for drone_id in self.drone_names:
            self._set_drone_mode(drone_id, "RTL")
        
        self.emergency_active = True


def main(args=None):
    rclpy.init(args=args)
    
    emergency_controller = EmergencyController()
    
    try:
        rclpy.spin(emergency_controller)
    except KeyboardInterrupt:
        pass
    finally:
        emergency_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()