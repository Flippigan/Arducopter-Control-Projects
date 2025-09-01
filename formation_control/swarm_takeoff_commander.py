#!/usr/bin/env python3
"""
Swarm Takeoff Commander

Simple script to command all 5 drones in the swarm to takeoff using ArduPilot DDS interface.
Uses the ArduPilot DDS takeoff service and proper mode switching.

MIGRATED TO ARDUPILOT DDS:
- Uses correct DDS service paths: /{drone_ns}/ap/arm_motors, /ap/mode_switch, /ap/takeoff
- Uses correct DDS topic paths: /{drone_ns}/ap/status, /ap/pose/filtered
- No /rs/ or /rt/ prefixes - direct /ap/ paths only
- Essential for Phase 2 formation control testing workflow

Author: Finn Picotoli
Date: 2025-08-26
Updated: 2025-08-27 (ArduPilot DDS path corrections)
"""

import rclpy
from rclpy.node import Node
import asyncio
import time
from std_srvs.srv import Trigger
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from ardupilot_msgs.msg import Status
from geometry_msgs.msg import PoseStamped


class SwarmTakeoffCommander(Node):
    def __init__(self):
        super().__init__('swarm_takeoff_commander')
        
        # Drone namespaces matching your launch file
        self.drone_namespaces = [
            'iris_9002',
            'iris_9012', 
            'iris_9022',
            'iris_9032',
            'iris_9042'
        ]
        
        # Takeoff parameters
        self.takeoff_altitude = 10.0  # meters
        
        # Service clients for each drone
        self.arm_clients = {}
        self.mode_clients = {}
        self.takeoff_clients = {}
        self.prearm_clients = {}
        
        # Status subscribers
        self.status_subs = {}
        self.drone_status = {}
        
        # Position subscribers
        self.position_subs = {}
        self.drone_positions = {}
        
        # Initialize clients and subscribers for each drone
        for drone_ns in self.drone_namespaces:
            # Service clients using ArduPilot DDS interface (CORRECTED PATHS)
            self.arm_clients[drone_ns] = self.create_client(
                ArmMotors, f'/{drone_ns}/ap/arm_motors')
            
            self.mode_clients[drone_ns] = self.create_client(
                ModeSwitch, f'/{drone_ns}/ap/mode_switch')
            
            self.takeoff_clients[drone_ns] = self.create_client(
                Takeoff, f'/{drone_ns}/ap/takeoff')
            
            self.prearm_clients[drone_ns] = self.create_client(
                Trigger, f'/{drone_ns}/ap/prearm_check')
            
            # Status subscriber (CORRECTED PATH)
            status_topic = f'/{drone_ns}/ap/status'
            self.status_subs[drone_ns] = self.create_subscription(
                Status, status_topic,
                lambda msg, ns=drone_ns: self.status_callback(msg, ns), 10)
            
            # Position subscriber (CORRECTED PATH)  
            position_topic = f'/{drone_ns}/ap/pose/filtered'
            self.position_subs[drone_ns] = self.create_subscription(
                PoseStamped, position_topic,
                lambda msg, ns=drone_ns: self.position_callback(msg, ns), 10)
            
            self.get_logger().info(f'Created DDS clients/subscribers for {drone_ns}')
            self.get_logger().debug(f'  Status: {status_topic}')
            self.get_logger().debug(f'  Position: {position_topic}')
            
            # Initialize status storage
            self.drone_status[drone_ns] = None
            self.drone_positions[drone_ns] = None
        
        self.get_logger().info(f'Swarm Takeoff Commander initialized for {len(self.drone_namespaces)} drones: {self.drone_namespaces}')
        self.get_logger().info(f'Target takeoff altitude: {self.takeoff_altitude}m')
        self.get_logger().info('Using ArduPilot DDS with corrected paths - /ap/* (no /rs/ or /rt/ prefixes)')

    def status_callback(self, msg, drone_ns):
        """Handle status updates from drones"""
        self.drone_status[drone_ns] = msg
        
    def position_callback(self, msg, drone_ns):
        """Handle position updates from drones"""
        self.drone_positions[drone_ns] = msg

    async def wait_for_services(self):
        """Wait for all DDS services to be available"""
        self.get_logger().info('Waiting for ArduPilot DDS services...')
        
        for drone_ns in self.drone_namespaces:
            # Wait for each service with timeout (CORRECTED SERVICE NAMES)
            services = [
                (self.arm_clients[drone_ns], 'arm_motors'),
                (self.mode_clients[drone_ns], 'mode_switch'),
                (self.takeoff_clients[drone_ns], 'takeoff'),
                (self.prearm_clients[drone_ns], 'prearm_check')
            ]
            
            for client, service_name in services:
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error(f'{drone_ns}/{service_name} not available')
                    return False
                    
        self.get_logger().info('All DDS services available')
        return True

    async def prearm_check_all(self):
        """Run pre-arm checks on all drones"""
        self.get_logger().info('Running pre-arm checks...')
        
        tasks = []
        for drone_ns in self.drone_namespaces:
            task = asyncio.create_task(self.prearm_check_drone(drone_ns))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        success_count = sum(1 for r in results if r is True)
        self.get_logger().info(f'Pre-arm checks: {success_count}/{len(self.drone_namespaces)} passed')
        
        return success_count == len(self.drone_namespaces)

    async def prearm_check_drone(self, drone_ns):
        """Run pre-arm check on individual drone"""
        try:
            request = Trigger.Request()
            future = self.prearm_clients[drone_ns].call_async(request)
            response = await future
            
            if response.success:
                self.get_logger().info(f'{drone_ns}: Pre-arm check PASSED')
            else:
                self.get_logger().warning(f'{drone_ns}: Pre-arm check FAILED - {response.message}')
                
            return response.success
            
        except Exception as e:
            self.get_logger().error(f'{drone_ns}: Pre-arm check ERROR - {e}')
            return False

    async def set_guided_mode_all(self):
        """Set all drones to GUIDED mode"""
        self.get_logger().info('Setting all drones to GUIDED mode...')
        
        tasks = []
        for drone_ns in self.drone_namespaces:
            task = asyncio.create_task(self.set_guided_mode_drone(drone_ns))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        success_count = sum(1 for r in results if r is True)
        self.get_logger().info(f'Mode switch: {success_count}/{len(self.drone_namespaces)} successful')
        
        return success_count == len(self.drone_namespaces)

    async def set_guided_mode_drone(self, drone_ns):
        """Set individual drone to GUIDED mode"""
        try:
            request = ModeSwitch.Request()
            request.mode = 4  # GUIDED mode for ArduCopter
            
            future = self.mode_clients[drone_ns].call_async(request)
            response = await future
            
            if response.status:
                self.get_logger().info(f'{drone_ns}: Mode set to GUIDED')
            else:
                self.get_logger().error(f'{drone_ns}: Failed to set GUIDED mode')
                
            return response.status
            
        except Exception as e:
            self.get_logger().error(f'{drone_ns}: Mode switch ERROR - {e}')
            return False

    async def arm_all_drones(self):
        """Arm all drone motors"""
        self.get_logger().info('Arming all drones...')
        
        tasks = []
        for drone_ns in self.drone_namespaces:
            task = asyncio.create_task(self.arm_drone(drone_ns))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        success_count = sum(1 for r in results if r is True)
        self.get_logger().info(f'Arming: {success_count}/{len(self.drone_namespaces)} successful')
        
        return success_count == len(self.drone_namespaces)

    async def arm_drone(self, drone_ns):
        """Arm individual drone"""
        try:
            request = ArmMotors.Request()
            request.arm = True
            
            future = self.arm_clients[drone_ns].call_async(request)
            response = await future
            
            if response.result:
                self.get_logger().info(f'{drone_ns}: Armed successfully')
            else:
                self.get_logger().error(f'{drone_ns}: Failed to arm')
                
            return response.result
            
        except Exception as e:
            self.get_logger().error(f'{drone_ns}: Arming ERROR - {e}')
            return False

    async def takeoff_all_drones(self):
        """Command all drones to takeoff using DDS takeoff service"""
        self.get_logger().info(f'Commanding all drones to takeoff to {self.takeoff_altitude}m...')
        
        tasks = []
        for drone_ns in self.drone_namespaces:
            task = asyncio.create_task(self.takeoff_drone(drone_ns))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        success_count = sum(1 for r in results if r is True)
        self.get_logger().info(f'Takeoff commands: {success_count}/{len(self.drone_namespaces)} successful')
        
        return success_count == len(self.drone_namespaces)

    async def takeoff_drone(self, drone_ns):
        """Command individual drone to takeoff"""
        try:
            request = Takeoff.Request()
            request.alt = self.takeoff_altitude
            
            future = self.takeoff_clients[drone_ns].call_async(request)
            response = await future
            
            if response.status:
                self.get_logger().info(f'{drone_ns}: Takeoff command accepted')
            else:
                self.get_logger().error(f'{drone_ns}: Takeoff command rejected')
                
            return response.status
            
        except Exception as e:
            self.get_logger().error(f'{drone_ns}: Takeoff ERROR - {e}')
            return False

    def wait_for_takeoff_completion(self, timeout=60.0):
        """Wait for all drones to reach target altitude"""
        self.get_logger().info(f'Waiting for takeoff completion (timeout: {timeout}s)...')
        
        start_time = time.time()
        target_reached = {drone_ns: False for drone_ns in self.drone_namespaces}
        
        while (time.time() - start_time) < timeout:
            # Check each drone's altitude
            all_at_altitude = True
            
            for drone_ns in self.drone_namespaces:
                if target_reached[drone_ns]:
                    continue
                    
                pos = self.drone_positions.get(drone_ns)
                if pos and pos.pose.position.z >= (self.takeoff_altitude - 1.0):
                    target_reached[drone_ns] = True
                    self.get_logger().info(f'{drone_ns}: Reached target altitude ({pos.pose.position.z:.1f}m)')
                else:
                    all_at_altitude = False
            
            if all_at_altitude:
                self.get_logger().info('All drones reached target altitude!')
                return True
                
            time.sleep(0.5)  # Check every 500ms
            
        # Timeout reached
        completed = sum(target_reached.values())
        self.get_logger().warning(f'Takeoff timeout: {completed}/{len(self.drone_namespaces)} drones at altitude')
        return False

    def print_swarm_status(self):
        """Print current status of all drones"""
        self.get_logger().info('=== SWARM STATUS ===')
        
        for drone_ns in self.drone_namespaces:
            status = self.drone_status.get(drone_ns)
            pos = self.drone_positions.get(drone_ns)
            
            if status and pos:
                armed = "ARMED" if status.armed else "DISARMED"
                flying = "FLYING" if status.flying else "GROUND"
                alt = pos.pose.position.z
                
                self.get_logger().info(f'{drone_ns}: {armed}, {flying}, ALT: {alt:.1f}m')
            else:
                self.get_logger().info(f'{drone_ns}: NO DATA')

    async def execute_swarm_takeoff(self):
        """Execute complete swarm takeoff sequence"""
        self.get_logger().info('=== STARTING SWARM TAKEOFF SEQUENCE ===')
        
        try:
            # Step 0: Wait for services
            if not await self.wait_for_services():
                self.get_logger().error('DDS services not available - aborting takeoff')
                return False
                
            # Wait for status data
            self.get_logger().info('Waiting for drone status data...')
            await asyncio.sleep(3.0)
            
            # Step 1: Pre-arm checks
            if not await self.prearm_check_all():
                self.get_logger().error('Pre-arm checks failed - aborting takeoff')
                return False
            
            # Step 2: Set GUIDED mode
            if not await self.set_guided_mode_all():
                self.get_logger().error('Mode switch failed - aborting takeoff')
                return False
                
            await asyncio.sleep(2.0)  # Allow mode switch to complete
            
            # Step 3: Arm all drones
            if not await self.arm_all_drones():
                self.get_logger().error('Arming failed - aborting takeoff')
                return False
                
            await asyncio.sleep(2.0)  # Allow arming to complete
            
            # Step 4: Takeoff command
            if not await self.takeoff_all_drones():
                self.get_logger().error('Takeoff commands failed')
                return False
            
            # Step 5: Wait for takeoff completion
            success = self.wait_for_takeoff_completion(timeout=30.0)
            
            if success:
                self.get_logger().info('=== SWARM TAKEOFF COMPLETE ===')
                self.print_swarm_status()
            else:
                self.get_logger().warning('=== SWARM TAKEOFF INCOMPLETE ===')
                self.print_swarm_status()
            
            return success
                
        except KeyboardInterrupt:
            self.get_logger().info('Takeoff sequence interrupted by user')
            return False
        except Exception as e:
            self.get_logger().error(f'Takeoff sequence failed: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        takeoff_commander = SwarmTakeoffCommander()
        
        # Wait a moment for connections
        time.sleep(2.0)
        
        # Execute takeoff sequence
        loop = asyncio.get_event_loop()
        success = loop.run_until_complete(takeoff_commander.execute_swarm_takeoff())
        
        if success:
            takeoff_commander.get_logger().info('Takeoff successful! Press Ctrl+C to exit.')
            try:
                rclpy.spin(takeoff_commander)
            except KeyboardInterrupt:
                takeoff_commander.get_logger().info('Shutting down...')
        else:
            takeoff_commander.get_logger().error('Takeoff failed!')
            
    except KeyboardInterrupt:
        print('\nTakeoff interrupted by user')
    except Exception as e:
        print(f'Error during takeoff: {e}')
    finally:
        if 'takeoff_commander' in locals():
            takeoff_commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()