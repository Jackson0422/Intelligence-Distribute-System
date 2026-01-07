#!/usr/bin/env python3
"""
RRT (Rapidly-exploring Random Tree) 路径规划算法
适用于 TurtleBot3 World 六边形地图

特点：
- 固定随机种子，确保相同输入产生相同输出
- 考虑所有障碍物（圆柱、六边形、墙壁）
- 可被 track_baseline.py 调用
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
    """圆形障碍物"""
    x: float
    y: float
    radius: float
    
    def contains_point(self, p: Point, margin: float = 0.0) -> bool:
        dist = math.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)
        return dist < (self.radius + margin)
    
    def intersects_line(self, p1: Point, p2: Point, margin: float = 0.0) -> bool:
        """检查线段是否与圆形障碍物相交"""
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
    """RRT树节点"""
    def __init__(self, point: Point, parent: Optional['RRTNode'] = None):
        self.point = point
        self.parent = parent


class TurtleBot3WorldMap:
    """
    TurtleBot3 World 地图定义
    
    地图信息：
    - 地图范围: X[-2.5, 3.0]m, Y[-2.3, 2.3]m
    - 中心9个圆柱: 3x3网格在 (±1.1, ±1.1)，半径0.15m
    - 六边形障碍物:
      - Head: (3.5, 0), 半径约0.8m
      - Left Hand: (1.8, 2.7), 半径约0.55m
      - Right Hand: (1.8, -2.7), 半径约0.55m
      - Left Foot: (-1.8, 2.7), 半径约0.55m
      - Right Foot: (-1.8, -2.7), 半径约0.55m
    """
    
    # 机器人半径（TurtleBot3 Burger，略大于实际0.09m以增加安全裕度）
    ROBOT_RADIUS = 0.22

    def __init__(self):
        # 地图边界
        self.x_min = -2.5
        self.x_max = 3.0
        self.y_min = -2.3
        self.y_max = 2.3
        
        # 定义所有障碍物
        self.obstacles: List[CircleObstacle] = []
        
        # 9个圆柱 (3x3网格)，半径0.15m
        cylinder_positions = [
            (-1.1, -1.1), (-1.1, 0), (-1.1, 1.1),
            (0, -1.1), (0, 0), (0, 1.1),
            (1.1, -1.1), (1.1, 0), (1.1, 1.1)
        ]
        for x, y in cylinder_positions:
            self.obstacles.append(CircleObstacle(x, y, 0.18))
        
        # 5个六边形障碍物（用圆形近似）
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
        """检查点是否在有效区域内（不碰撞）"""
        if not (self.x_min <= p.x <= self.x_max and 
                self.y_min <= p.y <= self.y_max):
            return False
        
        for obs in self.obstacles:
            if obs.contains_point(p, self.ROBOT_RADIUS):
                return False
        
        return True
    
    def is_path_valid(self, p1: Point, p2: Point) -> bool:
        """检查两点之间的路径是否有效"""
        if not self.is_point_valid(p1) or not self.is_point_valid(p2):
            return False
        
        for obs in self.obstacles:
            if obs.intersects_line(p1, p2, self.ROBOT_RADIUS):
                return False
        
        return True


class RRTPlanner:
    """RRT路径规划器"""
    
    def __init__(self, 
                 world_map: TurtleBot3WorldMap,
                 step_size: float = 0.3,
                 max_iterations: int = 5000,
                 goal_sample_rate: float = 0.1,
                 goal_tolerance: float = 0.2,
                 seed: int = 42):
        """
        初始化RRT规划器
        
        Args:
            world_map: 地图对象
            step_size: 每步扩展的最大距离
            max_iterations: 最大迭代次数
            goal_sample_rate: 采样目标点的概率
            goal_tolerance: 到达目标的容差
            seed: 随机种子（固定以保证可重复性）
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
        规划从起点到终点的路径
        
        Args:
            start: 起点坐标 (x, y)
            goal: 终点坐标 (x, y)
            
        Returns:
            路径点列表，如果找不到路径则返回None
        """
        # 固定随机种子
        random.seed(self.seed)
        
        start_point = Point(start[0], start[1])
        goal_point = Point(goal[0], goal[1])
        
        # 验证起点和终点
        if not self.map.is_point_valid(start_point):
            print(f"警告：起点 {start} 在障碍物内或超出边界")
            return None
        if not self.map.is_point_valid(goal_point):
            print(f"警告：终点 {goal} 在障碍物内或超出边界")
            return None
        
        # 首先尝试直接连接
        if self.map.is_path_valid(start_point, goal_point):
            return [start, goal]
        
        # 初始化树
        start_node = RRTNode(start_point)
        nodes = [start_node]
        
        for _ in range(self.max_iterations):
            # 采样随机点
            if random.random() < self.goal_sample_rate:
                sample = goal_point
            else:
                sample = self._random_sample()
            
            # 找最近节点
            nearest_node = self._find_nearest(nodes, sample)
            
            # 向采样点方向扩展
            new_point = self._steer(nearest_node.point, sample)
            
            # 检查路径是否有效
            if self.map.is_path_valid(nearest_node.point, new_point):
                new_node = RRTNode(new_point, nearest_node)
                nodes.append(new_node)
                
                # 检查是否到达目标
                if new_point.distance_to(goal_point) < self.goal_tolerance:
                    if self.map.is_path_valid(new_point, goal_point):
                        goal_node = RRTNode(goal_point, new_node)
                        nodes.append(goal_node)
                        path = self._extract_path(goal_node)
                        return self._smooth_path(path)
        
        print(f"警告：在 {self.max_iterations} 次迭代后未找到路径")
        return None
    
    def _random_sample(self) -> Point:
        """在地图范围内随机采样"""
        x = random.uniform(self.map.x_min, self.map.x_max)
        y = random.uniform(self.map.y_min, self.map.y_max)
        return Point(x, y)
    
    def _find_nearest(self, nodes: List[RRTNode], point: Point) -> RRTNode:
        """找到距离采样点最近的节点"""
        min_dist = float('inf')
        nearest = nodes[0]
        for node in nodes:
            dist = node.point.distance_to(point)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest
    
    def _steer(self, from_point: Point, to_point: Point) -> Point:
        """从一点向另一点移动，最大距离为step_size"""
        dist = from_point.distance_to(to_point)
        if dist <= self.step_size:
            return to_point
        
        ratio = self.step_size / dist
        new_x = from_point.x + ratio * (to_point.x - from_point.x)
        new_y = from_point.y + ratio * (to_point.y - from_point.y)
        return Point(new_x, new_y)
    
    def _extract_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """从目标节点回溯提取路径"""
        path = []
        node = goal_node
        while node is not None:
            path.append((node.point.x, node.point.y))
            node = node.parent
        path.reverse()
        return path
    
    def _smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """路径平滑：尝试跳过中间点直接连接"""
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
    便捷函数：规划单条路径
    
    Args:
        start: 起点 (x, y)
        goal: 终点 (x, y)
        seed: 随机种子
        
    Returns:
        路径点列表
    """
    world_map = TurtleBot3WorldMap()
    planner = RRTPlanner(world_map, seed=seed)
    return planner.plan(start, goal)


def plan_multi_waypoints(waypoints: List[Tuple[float, float]], 
                         seed: int = 42) -> Optional[List[Tuple[float, float]]]:
    """
    规划经过多个航点的完整路径
    
    Args:
        waypoints: 航点列表 [(x1,y1), (x2,y2), ...]
        seed: 随机种子
        
    Returns:
        完整路径点列表
    """
    if len(waypoints) < 2:
        return list(waypoints) if waypoints else []
    
    world_map = TurtleBot3WorldMap()
    planner = RRTPlanner(world_map, seed=seed)
    
    full_path = []
    
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]
        
        # 每段使用不同但可重复的种子
        segment_seed = seed + i * 1000
        planner.seed = segment_seed
        
        segment_path = planner.plan(start, goal)
        
        if segment_path is None:
            print(f"警告：无法规划从 {start} 到 {goal} 的路径")
            return None
        
        # 避免重复添加连接点
        if i == 0:
            full_path.extend(segment_path)
        else:
            full_path.extend(segment_path[1:])
    
    return full_path


# ==================== 测试代码 ====================
if __name__ == '__main__':
    print("=" * 60)
    print("TurtleBot3 World RRT 路径规划测试")
    print("=" * 60)
    
    # 测试单条路径
    print("\n[测试1] 单条路径规划")
    start = (-2.0, -0.5)
    goal = (2.0, 1.5)
    
    path = plan_path(start, goal, seed=42)
    
    if path:
        print(f"✓ 找到路径！共 {len(path)} 个点：")
        for i, (x, y) in enumerate(path):
            print(f"  [{i}] ({x:.2f}, {y:.2f})")
    else:
        print("✗ 未找到路径")
    
    # 测试多航点路径
    print("\n" + "=" * 60)
    print("[测试2] 多航点路径规划")
    
    waypoints = [
        (-2.0, -0.5),  # 起点
        (2.5, 0.0),    # 右侧
        (0.0, 2.0),    # 顶部
        (-2.0, -0.5),  # 返回起点
    ]
    
    print(f"关键航点: {waypoints}")
    
    full_path = plan_multi_waypoints(waypoints, seed=42)
    
    if full_path:
        print(f"\n✓ 找到完整路径！共 {len(full_path)} 个点：")
        for i, (x, y) in enumerate(full_path):
            print(f"  [{i}] ({x:.2f}, {y:.2f})")
    else:
        print("✗ 未找到路径")
    
    print("\n" + "=" * 60)
    print("测试完成")

