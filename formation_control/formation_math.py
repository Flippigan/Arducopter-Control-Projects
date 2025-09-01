#!/usr/bin/env python3
"""
Formation Math Module

Mathematical models and utilities for 3D formation control.
Includes sphere formation generation, collision detection, and trajectory planning.

Author: Finn Picotoli
Date: 2025-08-22
"""

import numpy as np
import math
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass


@dataclass
class DroneState:
    """Represents the state of a single drone"""
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    timestamp: float


@dataclass
class FormationParameters:
    """Parameters defining a formation"""
    center: np.ndarray  # [x, y, z]
    radius: float
    pattern_type: str
    scale: float = 1.0
    rotation: np.ndarray = None  # [roll, pitch, yaw]


class SphereFormation:
    """Sphere formation generator using various distribution algorithms"""
    
    def __init__(self):
        self.golden_ratio = (1 + math.sqrt(5)) / 2
    
    def fibonacci_spiral_distribution(self, num_points: int, radius: float, 
                                    center: np.ndarray = np.zeros(3)) -> List[np.ndarray]:
        """
        Generate uniform sphere distribution using Fibonacci spiral method.
        Optimal for uniform coverage with any number of points.
        """
        points = []
        
        for i in range(num_points):
            # Fibonacci spiral parameters
            theta = 2 * math.pi * i / self.golden_ratio
            phi = math.acos(1 - 2 * i / num_points)
            
            # Convert spherical to Cartesian coordinates
            x = center[0] + radius * math.sin(phi) * math.cos(theta)
            y = center[1] + radius * math.sin(phi) * math.sin(theta)
            z = center[2] + radius * math.cos(phi)
            
            points.append(np.array([x, y, z]))
        
        return points
    
    def geodesic_distribution(self, num_points: int, radius: float,
                            center: np.ndarray = np.zeros(3)) -> List[np.ndarray]:
        """
        Generate geodesic sphere distribution based on icosahedron subdivision.
        Good for structured, symmetric formations.
        """
        # Simplified geodesic - for full implementation, use icosahedron subdivision
        # For now, fall back to Fibonacci spiral
        return self.fibonacci_spiral_distribution(num_points, radius, center)
    
    def regular_distribution(self, num_points: int, radius: float,
                           center: np.ndarray = np.zeros(3)) -> List[np.ndarray]:
        """
        Generate regular angular distribution (rings at different latitudes).
        Good for predictable, symmetric patterns.
        """
        points = []
        
        if num_points <= 2:
            # Special cases
            if num_points == 1:
                points.append(center + np.array([0, 0, radius]))
            elif num_points == 2:
                points.append(center + np.array([0, 0, radius]))
                points.append(center + np.array([0, 0, -radius]))
            return points
        
        # Distribute points in rings
        num_rings = max(1, int(math.sqrt(num_points)))
        points_per_ring = num_points // num_rings
        
        for ring in range(num_rings):
            ring_z = radius * math.cos(math.pi * ring / (num_rings - 1))
            ring_radius = radius * math.sin(math.pi * ring / (num_rings - 1))
            
            points_in_this_ring = points_per_ring
            if ring == num_rings - 1:
                points_in_this_ring = num_points - ring * points_per_ring
            
            for point in range(points_in_this_ring):
                angle = 2 * math.pi * point / points_in_this_ring
                x = center[0] + ring_radius * math.cos(angle)
                y = center[1] + ring_radius * math.sin(angle)
                z = center[2] + ring_z
                
                points.append(np.array([x, y, z]))
        
        return points
    
    def optimized_distribution(self, num_points: int, radius: float,
                             center: np.ndarray = np.zeros(3),
                             min_separation: float = 2.0) -> List[np.ndarray]:
        """
        Generate optimized distribution with minimum separation constraints.
        Uses repulsion algorithm to ensure minimum distances.
        """
        # Start with Fibonacci distribution
        points = self.fibonacci_spiral_distribution(num_points, radius, center)
        
        # Apply repulsion algorithm to enforce minimum separation
        for iteration in range(100):  # Maximum iterations
            forces = [np.zeros(3) for _ in range(num_points)]
            max_force = 0
            
            # Calculate repulsion forces
            for i in range(num_points):
                for j in range(i + 1, num_points):
                    diff = points[i] - points[j]
                    distance = np.linalg.norm(diff)
                    
                    if distance < min_separation and distance > 0:
                        # Repulsion force inversely proportional to distance²
                        force_magnitude = (min_separation - distance) / (distance ** 2)
                        force_direction = diff / distance
                        
                        forces[i] += force_magnitude * force_direction
                        forces[j] -= force_magnitude * force_direction
                        
                        max_force = max(max_force, force_magnitude)
            
            # Apply forces and project back to sphere
            for i in range(num_points):
                if np.linalg.norm(forces[i]) > 0:
                    # Move point
                    points[i] += forces[i] * 0.1  # Small step size
                    
                    # Project back to sphere surface
                    direction = points[i] - center
                    if np.linalg.norm(direction) > 0:
                        points[i] = center + radius * direction / np.linalg.norm(direction)
            
            # Check convergence
            if max_force < 0.01:
                break
        
        return points


class CollisionDetection:
    """Collision detection and prediction algorithms"""
    
    def __init__(self, safety_radius: float = 3.0, prediction_time: float = 2.0):
        self.safety_radius = safety_radius
        self.prediction_time = prediction_time
    
    def check_immediate_collision(self, drone_states: Dict[str, DroneState]) -> List[Tuple[str, str, float]]:
        """Check for immediate collision risks between all drone pairs"""
        collisions = []
        drone_ids = list(drone_states.keys())
        
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                id1, id2 = drone_ids[i], drone_ids[j]
                state1, state2 = drone_states[id1], drone_states[id2]
                
                distance = np.linalg.norm(state2.position - state1.position)
                if distance < self.safety_radius:
                    risk_level = 1.0 - (distance / self.safety_radius)
                    collisions.append((id1, id2, risk_level))
        
        return collisions
    
    def predict_collision(self, state1: DroneState, state2: DroneState) -> Optional[Tuple[float, float]]:
        """
        Predict collision between two drones.
        Returns (time_to_collision, minimum_distance) or None if no collision.
        """
        # Relative position and velocity
        rel_pos = state2.position - state1.position
        rel_vel = state2.velocity - state1.velocity
        
        # If relative velocity is very small, no collision prediction
        if np.linalg.norm(rel_vel) < 0.1:
            current_distance = np.linalg.norm(rel_pos)
            return None if current_distance >= self.safety_radius else (0.0, current_distance)
        
        # Time when drones are closest (derivative of distance = 0)
        t_closest = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
        
        # Only consider future times within prediction horizon
        if t_closest <= 0 or t_closest > self.prediction_time:
            return None
        
        # Calculate minimum distance
        future_rel_pos = rel_pos + rel_vel * t_closest
        min_distance = np.linalg.norm(future_rel_pos)
        
        if min_distance < self.safety_radius:
            return (t_closest, min_distance)
        
        return None
    
    def predict_all_collisions(self, drone_states: Dict[str, DroneState]) -> List[Tuple[str, str, float, float]]:
        """Predict all potential collisions in the fleet"""
        predictions = []
        drone_ids = list(drone_states.keys())
        
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                id1, id2 = drone_ids[i], drone_ids[j]
                state1, state2 = drone_states[id1], drone_states[id2]
                
                prediction = self.predict_collision(state1, state2)
                if prediction:
                    time_to_collision, min_distance = prediction
                    risk_level = 1.0 - (min_distance / self.safety_radius)
                    predictions.append((id1, id2, time_to_collision, risk_level))
        
        return predictions


class TrajectoryPlanning:
    """Trajectory planning and optimization for formation control"""
    
    def __init__(self, max_velocity: float = 3.0, max_acceleration: float = 2.0):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
    
    def plan_formation_assembly(self, current_positions: Dict[str, np.ndarray],
                              target_positions: Dict[str, np.ndarray],
                              assembly_time: float = 30.0) -> Dict[str, List[np.ndarray]]:
        """
        Plan trajectories for formation assembly.
        Returns waypoints for each drone.
        """
        trajectories = {}
        
        for drone_id in current_positions:
            if drone_id in target_positions:
                current = current_positions[drone_id]
                target = target_positions[drone_id]
                
                # Simple linear trajectory with velocity constraints
                trajectory = self._plan_constrained_trajectory(current, target, assembly_time)
                trajectories[drone_id] = trajectory
        
        return trajectories
    
    def _plan_constrained_trajectory(self, start: np.ndarray, end: np.ndarray,
                                   duration: float, num_waypoints: int = 10) -> List[np.ndarray]:
        """Plan trajectory with velocity and acceleration constraints"""
        waypoints = []
        
        # Calculate required velocity
        distance = np.linalg.norm(end - start)
        required_velocity = distance / duration
        
        if required_velocity > self.max_velocity:
            # Need to extend duration or use maximum velocity
            duration = distance / self.max_velocity
        
        # Generate waypoints along trajectory
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            # Use smooth trajectory (ease in/out)
            smooth_t = self._smooth_step(t)
            waypoint = start + smooth_t * (end - start)
            waypoints.append(waypoint)
        
        return waypoints
    
    def _smooth_step(self, t: float) -> float:
        """Smooth step function for trajectory smoothing"""
        return t * t * (3.0 - 2.0 * t)
    
    def optimize_formation_movement(self, formation_params: FormationParameters,
                                  new_params: FormationParameters,
                                  current_positions: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """Optimize formation movement to minimize drone travel distances"""
        # Generate target positions for new formation
        sphere_gen = SphereFormation()
        num_drones = len(current_positions)
        
        new_positions = sphere_gen.fibonacci_spiral_distribution(
            num_drones, new_params.radius, new_params.center)
        
        # Optimize assignment to minimize total travel distance
        drone_ids = list(current_positions.keys())
        optimized_targets = {}
        
        # Simple greedy assignment (for better optimization, use Hungarian algorithm)
        available_positions = new_positions.copy()
        
        for drone_id in drone_ids:
            current_pos = current_positions[drone_id]
            
            # Find closest available position
            min_distance = float('inf')
            best_position = None
            best_index = -1
            
            for i, pos in enumerate(available_positions):
                distance = np.linalg.norm(pos - current_pos)
                if distance < min_distance:
                    min_distance = distance
                    best_position = pos
                    best_index = i
            
            if best_position is not None:
                optimized_targets[drone_id] = best_position
                available_positions.pop(best_index)
        
        return optimized_targets


class FormationQuality:
    """Formation quality assessment and metrics"""
    
    @staticmethod
    def calculate_formation_error(current_positions: Dict[str, np.ndarray],
                                target_positions: Dict[str, np.ndarray]) -> Dict[str, float]:
        """Calculate position errors for each drone in the formation"""
        errors = {}
        
        for drone_id in current_positions:
            if drone_id in target_positions:
                error = np.linalg.norm(current_positions[drone_id] - target_positions[drone_id])
                errors[drone_id] = error
        
        return errors
    
    @staticmethod
    def calculate_formation_quality(current_positions: Dict[str, np.ndarray],
                                  target_positions: Dict[str, np.ndarray],
                                  tolerance: float = 2.0) -> float:
        """
        Calculate overall formation quality (0.0 to 1.0).
        1.0 = perfect formation, 0.0 = completely dispersed
        """
        if not current_positions or not target_positions:
            return 0.0
        
        errors = FormationQuality.calculate_formation_error(current_positions, target_positions)
        
        if not errors:
            return 0.0
        
        # Calculate quality based on error distribution
        total_error = sum(errors.values())
        average_error = total_error / len(errors)
        max_error = max(errors.values())
        
        # Quality decreases exponentially with error
        quality = math.exp(-average_error / tolerance)
        
        # Penalize large maximum errors
        if max_error > tolerance * 2:
            quality *= 0.5
        
        return min(1.0, max(0.0, quality))


# Utility functions for formation control
def rotate_points_3d(points: List[np.ndarray], rotation_matrix: np.ndarray,
                    center: np.ndarray = np.zeros(3)) -> List[np.ndarray]:
    """Rotate a list of 3D points around a center point"""
    rotated_points = []
    
    for point in points:
        # Translate to origin, rotate, translate back
        centered_point = point - center
        rotated_point = rotation_matrix @ centered_point
        final_point = rotated_point + center
        rotated_points.append(final_point)
    
    return rotated_points


def create_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create 3D rotation matrix from Euler angles"""
    # Roll (rotation around x-axis)
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    # Pitch (rotation around y-axis)
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    # Yaw (rotation around z-axis)
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    # Combined rotation matrix
    return R_z @ R_y @ R_x


if __name__ == "__main__":
    # Test the formation math functions
    print("Testing Formation Math Module")
    
    # Test sphere formation generation
    sphere_gen = SphereFormation()
    positions = sphere_gen.fibonacci_spiral_distribution(8, 10.0, np.array([0, 0, 20]))
    
    print(f"Generated {len(positions)} sphere positions:")
    for i, pos in enumerate(positions):
        print(f"  Drone {i}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
    
    # Test collision detection
    collision_detector = CollisionDetection(safety_radius=3.0)
    
    # Create test drone states
    drone_states = {
        'drone_0': DroneState(np.array([0, 0, 0]), np.array([1, 0, 0]), 0.0),
        'drone_1': DroneState(np.array([2, 0, 0]), np.array([-1, 0, 0]), 0.0)
    }
    
    collisions = collision_detector.predict_all_collisions(drone_states)
    print(f"\nPredicted collisions: {len(collisions)}")
    for collision in collisions:
        drone1, drone2, time, risk = collision
        print(f"  {drone1} <-> {drone2}: t={time:.2f}s, risk={risk:.2f}")