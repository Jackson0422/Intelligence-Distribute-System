import os
import csv
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


@dataclass
class RobotState:
    gt: Optional[PoseStamped] = None
    amcl: Optional[PoseWithCovarianceStamped] = None
    coloc: Optional[PoseStamped] = None


class GroundTruthLogger(Node):
    """Publishes ground truth poses from Gazebo and logs GT/AMCL/Coloc to CSV."""

    def __init__(self):
        super().__init__('ground_truth_logger')
        self.declare_parameter(
            'robot_names',
            [f'tb3_{i}' for i in range(1, 21)]
        )
        # Prefer the source-tree logs folder next to this file; fall back to installed share logs.
        source_logs = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'logs'))
        if os.path.isdir(source_logs) or os.access(os.path.dirname(source_logs), os.W_OK):
            default_logs = source_logs
        else:
            try:
                pkg_share = get_package_share_directory('localization_evaluation')
                default_logs = os.path.join(pkg_share, 'logs')
            except Exception:
                default_logs = os.path.join(os.getcwd(), 'logs')
        self.declare_parameter('log_dir', default_logs)
        self.declare_parameter('log_enabled', True)
        self.declare_parameter('auto_detect_robots', True)

        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.log_enabled = self.get_parameter('log_enabled').get_parameter_value().bool_value
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.auto_detect = self.get_parameter('auto_detect_robots').get_parameter_value().bool_value

        self.state: Dict[str, RobotState] = {name: RobotState() for name in self.robot_names}

        self.sub_model_states = self.create_subscription(
            ModelStates, '/gazebo/model_states', self._on_model_states, 10
        )
        for name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self._on_amcl(msg, n),
                10,
            )
            self.create_subscription(
                PoseStamped,
                f'/{name}/coloc_pose',
                lambda msg, n=name: self._on_coloc(msg, n),
                10,
            )

        # Publishers to visualize ground truth
        self.gt_pubs = {
            name: self.create_publisher(PoseStamped, f'/{name}/ground_truth', 10)
            for name in self.robot_names
        }
        self.marker_pub = self.create_publisher(MarkerArray, '/localization_markers', 10)

        if self.log_enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            self.log_path = os.path.join(self.log_dir, f'localization_log_{ts}.csv')
            self.log_file = open(self.log_path, mode='w', newline='', encoding='utf-8')
            self.logger = csv.writer(self.log_file)
            self.logger.writerow([
                'timestamp',
                'robot',
                'gt_x', 'gt_y', 'gt_yaw',
                'amcl_x', 'amcl_y', 'amcl_yaw',
                'coloc_x', 'coloc_y', 'coloc_yaw',
            ])
            self.get_logger().info(f'Logging to {self.log_path}')
        else:
            self.log_file = None
            self.logger = None

        self.timer = self.create_timer(0.2, self._on_timer)

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        super().destroy_node()

    def _on_model_states(self, msg: ModelStates):
        # Dynamically add robots if auto_detect is enabled
        if self.auto_detect:
            for name in msg.name:
                if name.startswith('tb3_') and name not in self.state:
                    self.get_logger().info(f'Auto-detected robot {name}')
                    self.state[name] = RobotState()
                    self.robot_names.append(name)
                    self.create_subscription(
                        PoseWithCovarianceStamped,
                        f'/{name}/amcl_pose',
                        lambda msg, n=name: self._on_amcl(msg, n),
                        10,
                    )
                    self.create_subscription(
                        PoseStamped,
                        f'/{name}/coloc_pose',
                        lambda msg, n=name: self._on_coloc(msg, n),
                        10,
                    )
                    self.gt_pubs[name] = self.create_publisher(PoseStamped, f'/{name}/ground_truth', 10)

        for name in self.robot_names:
            if name in msg.name:
                idx = msg.name.index(name)
                pose = msg.pose[idx]
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'map'
                ps.pose = pose
                self.state[name].gt = ps
                self.gt_pubs[name].publish(ps)

    def _on_amcl(self, msg: PoseWithCovarianceStamped, robot_name: str):
        self.state[robot_name].amcl = msg

    def _on_coloc(self, msg: PoseStamped, robot_name: str):
        self.state[robot_name].coloc = msg

    def _on_timer(self):
        now = self.get_clock().now()
        stamp_msg = now.to_msg()
        timestamp = float(stamp_msg.sec) + stamp_msg.nanosec * 1e-9
        marker_array = MarkerArray()

        def yaw_from_quat(q):
            import math
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        def pose_fields(pose):
            return (
                pose.position.x,
                pose.position.y,
                yaw_from_quat(pose.orientation),
            )

        for idx, (name, st) in enumerate(self.state.items()):
            gt = st.gt
            amcl = st.amcl
            coloc = st.coloc

            if not (gt or amcl or coloc):
                continue

            gt_vals = pose_fields(gt.pose) if gt else ('', '', '')
            amcl_vals = pose_fields(amcl.pose.pose) if amcl else ('', '', '')
            coloc_vals = pose_fields(coloc.pose) if coloc else ('', '', '')

            if self.logger:
                self.logger.writerow([
                    timestamp,
                    name,
                    *gt_vals,
                    *amcl_vals,
                    *coloc_vals,
                ])
                self.log_file.flush()

            # Markers for RViz
            def make_marker(marker_id, pose_msg, color):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp_msg
                m.ns = name
                m.id = marker_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = pose_msg.pose if isinstance(pose_msg, PoseWithCovarianceStamped) else pose_msg.pose
                m.scale.x = m.scale.y = m.scale.z = 0.2
                m.color.r, m.color.g, m.color.b, m.color.a = color
                m.lifetime.sec = 0
                m.lifetime.nanosec = 0
                return m

            mid_base = idx * 10
            if gt:
                marker_array.markers.append(make_marker(mid_base + 1, gt, (0.2, 0.9, 0.2, 0.9)))
            if amcl:
                marker_array.markers.append(make_marker(mid_base + 2, amcl, (0.2, 0.4, 0.9, 0.9)))
            if coloc:
                marker_array.markers.append(make_marker(mid_base + 3, coloc, (0.9, 0.2, 0.8, 0.9)))

        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
