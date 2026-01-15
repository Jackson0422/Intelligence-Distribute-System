#!/usr/bin/env python3
"""
RRT (Rapidly-exploring Random Tree) path planning algorithm
Suitable for TurtleBot3 World hexagonal map

Features:
- Fixed random seed ensures identical output for identical input
- Considers all obstacles (cylinders, hexagons, walls)
- Can be called by track_baseline.py
"""

import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Point:
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def __iter__(self):
        yield self.x
        yield self.y


@dataclass
class CircleObstacle:
    """Circular obstacle"""
    x: float
    y: float
    radius: float
    
    def contains_point(self, p: Point, margin: float = 0.0) -> bool:
        dist = math.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
        return dist < (self.radius + margin)
    
    def intersects_line(self, p1: Point, p2: Point, margin: float = 0.0) -> bool:
        """Check if line segment intersects with circular obstacle"""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        fx = p1.x - self.x
        fy = p1.y - self.y
        
        a = dx*dx + dy*dy
        b = 2*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - (self.radius + margin)**2
        
        if a == 0:
            return self.contains_point(p1, margin)
        
        discriminant = b*b - 4*a*c
        
        if discriminant < 0:
            return False
        
        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)


class RRTNode:
    """RRT tree node"""
    def __init__(self, point: Point, parent: Optional['RRTNode'] = None):
        self.point = point
        self.parent = parent


class TurtleBot3WorldMap:
    """
    TurtleBot3 World map definition
    
    Map information:
    - Map range: X[-2.5, 3.0]m, Y[-2.3, 2.3]m
    - Center 9 cylinders: 3x3 grid at (±1.1, ±1.1), radius 0.15m
    - Hexagon obstacles:
      - Head: (3.5, 0), radius ~0.8m
      - Left Hand: (1.8, 2.7), radius ~0.55m
      - Right Hand: (1.8, -2.7), radius ~0.55m
      - Left Foot: (-1.8, 2.7), radius ~0.55m
      - Right Foot: (-1.8, -2.7), radius ~0.55m
    """
    
    # Robot radius (TurtleBot3 Burger, slightly larger than actual 0.09m for safety margin)
    ROBOT_RADIUS = 0.1 # Changed to be the same as nav2_params

    def __init__(self):
        # Map boundaries
        self.x_min = -2.5
        self.x_max = 3.0
        self.y_min = -2.3
        self.y_max = 2.3
        
        # Define all obstacles
        self.obstacles: List[CircleObstacle] = []
        
        # 9 cylinders (3x3 grid), radius 0.15m
        cylinder_positions = [
            (-1.1, -1.1), (-1.1, 0), (-1.1, 1.1),
            (0, -1.1), (0, 0), (0, 1.1),
            (1.1, -1.1), (1.1, 0), (1.1, 1.1)
        ]
        for x, y in cylinder_positions:
            self.obstacles.append(CircleObstacle(x, y, 0.18))
        
        # 5 hexagon obstacles (approximated as circles)
        hexagon_obstacles = [
            (3.5, 0, 0.8),      # Head
            (1.8, 2.7, 0.55),   # Left Hand
            (1.8, -2.7, 0.55),  # Right Hand
            (-1.8, 2.7, 0.55),  # Left Foot
            (-1.8, -2.7, 0.55)  # Right Foot
        ]
        for x, y, r in hexagon_obstacles:
            self.obstacles.append(CircleObstacle(x, y, r))
    
    def is_point_valid(self, p: Point) -> bool:
        """Check if point is in valid area (no collision)"""
        if not (self.x_min <= p.x <= self.x_max and 
                self.y_min <= p.y <= self.y_max):
            return False
        
        for obs in self.obstacles:
            if obs.contains_point(p, self.ROBOT_RADIUS):
                return False
        
        return True
    
    def is_path_valid(self, p1: Point, p2: Point) -> bool:
        """Check if path between two points is valid"""
        if not self.is_point_valid(p1) or not self.is_point_valid(p2):
            return False
        
        for obs in self.obstacles:
            if obs.intersects_line(p1, p2, self.ROBOT_RADIUS):
                return False
        
        return True


class RRTPlanner:
    """RRT path planner"""
    
    def __init__(self, 
                 world_map: TurtleBot3WorldMap,
                 step_size: float = 0.3,
                 max_iterations: int = 5000,
                 goal_sample_rate: float = 0.1,
                 goal_tolerance: float = 0.2,
                 seed: int = 42):
        """
        Initialize RRT planner
        
        Args:
            world_map: Map object
            step_size: Maximum distance to extend per step
            max_iterations: Maximum number of iterations
            goal_sample_rate: Probability of sampling goal point
            goal_tolerance: Tolerance for reaching goal
            seed: Random seed (fixed for reproducibility)
        """
        self.map = world_map
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate
        self.goal_tolerance = goal_tolerance
        self.seed = seed
    
    def plan(self, start: Tuple[float, float], 
             goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal
        
        Args:
            start: Start coordinates (x, y)
            goal: Goal coordinates (x, y)
            
        Returns:
            List of path points, or None if no path found
        """
        # Fix random seed
        random.seed(self.seed)
        
        start_point = Point(start[0], start[1])
        goal_point = Point(goal[0], goal[1])
        
        # Validate start and goal points
        if not self.map.is_point_valid(start_point):
            print(f"Warning: start point {start} is in obstacle or out of bounds")
            return None
        if not self.map.is_point_valid(goal_point):
            print(f"Warning: goal point {goal} is in obstacle or out of bounds")
            return None
        
        # Try direct connection first
        if self.map.is_path_valid(start_point, goal_point):
            return [start, goal]
        
        # Initialize tree
        start_node = RRTNode(start_point)
        nodes = [start_node]
        
        for _ in range(self.max_iterations):
            # Sample random point
            if random.random() < self.goal_sample_rate:
                sample = goal_point
            else:
                sample = self._random_sample()
            
            # Find nearest node
            nearest_node = self._find_nearest(nodes, sample)
            
            # Extend towards sample point
            new_point = self._steer(nearest_node.point, sample)
            
            # Check if path is valid
            if self.map.is_path_valid(nearest_node.point, new_point):
                new_node = RRTNode(new_point, nearest_node)
                nodes.append(new_node)
                
                # Check if goal is reached
                if new_point.distance_to(goal_point) < self.goal_tolerance:
                    if self.map.is_path_valid(new_point, goal_point):
                        goal_node = RRTNode(goal_point, new_node)
                        nodes.append(goal_node)
                        path = self._extract_path(goal_node)
                        return self._smooth_path(path)
        
        print(f"Warning: no path found after {self.max_iterations} iterations")
        return None
    
    def _random_sample(self) -> Point:
        """Sample randomly within map bounds"""
        x = random.uniform(self.map.x_min, self.map.x_max)
        y = random.uniform(self.map.y_min, self.map.y_max)
        return Point(x, y)
    
    def _find_nearest(self, nodes: List[RRTNode], point: Point) -> RRTNode:
        """Find nearest node to sampled point"""
        min_dist = float('inf')
        nearest = nodes[0]
        for node in nodes:
            dist = node.point.distance_to(point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def _steer(self, from_point: Point, to_point: Point) -> Point:
        """Move from one point towards another, max distance step_size"""
        dist = from_point.distance_to(to_point)
        if dist <= self.step_size:
            return to_point
        
        ratio = self.step_size / dist
        new_x = from_point.x + ratio * (to_point.x - from_point.x)
        new_y = from_point.y + ratio * (to_point.y - from_point.y)
        return Point(new_x, new_y)
    
    def _extract_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """Backtrack from goal node to extract path"""
        path = []
        node = goal_node
        while node is not None:
            path.append((node.point.x, node.point.y))
            node = node.parent
        path.reverse()
        return path
    
    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path: try to skip intermediate points by direct connections"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                p1 = Point(path[i][0], path[i][1])
                p2 = Point(path[j][0], path[j][1])
                if self.map.is_path_valid(p1, p2):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        
        return smoothed


def plan_path(start: Tuple[float, float], 
              goal: Tuple[float, float],
              seed: int = 42) -> Optional[List[Tuple[float, float]]]:
    """
    Convenience function: plan a single path
    
    Args:
        start: Start point (x, y)
        goal: Goal point (x, y)
        seed: Random seed
        
    Returns:
        List of path points
    """
    world_map = TurtleBot3WorldMap()
    planner = RRTPlanner(world_map, seed=seed)
    return planner.plan(start, goal)


def plan_multi_waypoints(waypoints: List[Tuple[float, float]], 
                         seed: int = 42) -> Optional[List[Tuple[float, float]]]:
    """
    Plan complete path through multiple waypoints
    
    Args:
        waypoints: List of waypoints [(x1,y1), (x2,y2), ...]
        seed: Random seed
        
    Returns:
        Complete path point list
    """
    if len(waypoints) < 2:
        return list(waypoints) if waypoints else []
    
    world_map = TurtleBot3WorldMap()
    planner = RRTPlanner(world_map, seed=seed)
    
    full_path = []
    
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]
        
        # Use different but reproducible seed for each segment
        segment_seed = seed + i * 1000
        planner.seed = segment_seed
        
        segment_path = planner.plan(start, goal)
        
        if segment_path is None:
            print(f"Warning: cannot plan path from {start} to {goal}")
            return None
        
        # Avoid duplicating connection points
        if i == 0:
            full_path.extend(segment_path)
        else:
            full_path.extend(segment_path[1:])
    
    return full_path


# ==================== Test Code ====================
if __name__ == '__main__':
    print("=" * 60)
    print("TurtleBot3 World RRT Path Planning Test")
    print("=" * 60)
    
    # Test single path
    print("\n[Test 1] Single path planning")
    start = (-2.0, -0.5)
    goal = (2.0, 1.5)
    
    path = plan_path(start, goal, seed=42)
    
    if path:
        print(f"✓ Path found! Total {len(path)} points:")
        for i, (x, y) in enumerate(path):
            print(f"  [{i}] ({x:.2f}, {y:.2f})")
    else:
        print("✗ No path found")
    
    # Test multi-waypoint path
    print("\n" + "=" * 60)
    print("[Test 2] Multi-waypoint path planning")
    
    waypoints = [
        (-2.0, -0.5),  # Start
        (2.5, 0.0),    # Right side
        (0.0, 2.0),    # Top
        (-2.0, -0.5),  # Return to start
    ]
    
    print(f"Key waypoints: {waypoints}")
    
    full_path = plan_multi_waypoints(waypoints, seed=42)
    
    if full_path:
        print(f"\n✓ Complete path found! Total {len(full_path)} points:")
        for i, (x, y) in enumerate(full_path):
            print(f"  [{i}] ({x:.2f}, {y:.2f})")
    else:
        print("✗ No path found")
    
    print("\n" + "=" * 60)
    print("Test completed")
