import os
import csv
import time
import copy
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy import parameter as rclpy_parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


@dataclass
class RobotState:
    gt: Optional[PoseStamped] = None
    amcl: Optional[PoseWithCovarianceStamped] = None
    coloc: Optional[PoseStamped] = None
    odom: Optional[Odometry] = None


class GroundTruthLogger(Node):
    """Publishes ground truth poses from Gazebo and logs GT/AMCL/Coloc to CSV."""

    def __init__(self):
        super().__init__('ground_truth_logger')
        self.declare_parameter(
            'robot_names',
            [f'tb3_{i}' for i in range(1, 21)]
        )
        # use_sim_time is automatically handled by ROS2, no need to declare/set it explicitly
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
        # Some setups publish /model_states instead; subscribe to both.
        self.sub_model_states_alt = self.create_subscription(
            ModelStates, '/model_states', self._on_model_states, 10
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
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._on_odom(msg, n),
                10,
            )

        # Publishers to visualize ground truth
        self.gt_pubs = {
            name: self.create_publisher(PoseStamped, f'/{name}/ground_truth', 10)
            for name in self.robot_names
        }
        # Combined markers (legacy) plus per-source topics for easier toggling in RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/localization_markers', 10)
        self.marker_pub_gt = self.create_publisher(MarkerArray, '/localization_markers_gt', 10)
        self.marker_pub_amcl = self.create_publisher(MarkerArray, '/localization_markers_amcl', 10)
        self.marker_pub_coloc = self.create_publisher(MarkerArray, '/localization_markers_coloc', 10)

        # Path publishers per source (one Path per robot per source)
        self.path_pub_gt: Dict[str, Path] = {}
        self.path_pub_amcl: Dict[str, Path] = {}
        self.path_pub_coloc: Dict[str, Path] = {}
        for name in self.robot_names:
            self.path_pub_gt[name] = self.create_publisher(Path, f'/{name}/path_gt', 10)
            self.path_pub_amcl[name] = self.create_publisher(Path, f'/{name}/path_amcl', 10)
            self.path_pub_coloc[name] = self.create_publisher(Path, f'/{name}/path_coloc', 10)
        self.path_history: Dict[str, Dict[str, list]] = {
            name: {'gt': [], 'amcl': [], 'coloc': []} for name in self.robot_names
        }
        self.path_history_limit = 500  # keep last N poses per path

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
                    self.create_subscription(
                        Odometry,
                        f'/{name}/odom',
                        lambda msg, n=name: self._on_odom(msg, n),
                        10,
                    )
                    self.gt_pubs[name] = self.create_publisher(PoseStamped, f'/{name}/ground_truth', 10)
                    self.path_pub_gt[name] = self.create_publisher(Path, f'/{name}/path_gt', 10)
                    self.path_pub_amcl[name] = self.create_publisher(Path, f'/{name}/path_amcl', 10)
                    self.path_pub_coloc[name] = self.create_publisher(Path, f'/{name}/path_coloc', 10)
                    self.path_history[name] = {'gt': [], 'amcl': [], 'coloc': []}

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

    def _on_odom(self, msg: Odometry, robot_name: str):
        self.state[robot_name].odom = msg

    def _on_timer(self):
        now = self.get_clock().now()
        stamp_msg = now.to_msg()
        timestamp = float(stamp_msg.sec) + stamp_msg.nanosec * 1e-9
        marker_all = MarkerArray()
        marker_gt = MarkerArray()
        marker_amcl = MarkerArray()
        marker_coloc = MarkerArray()

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
            if gt is None and st.odom is not None:
                # Fallback: use odometry if Gazebo GT is unavailable
                ps = PoseStamped()
                ps.header = st.odom.header
                ps.pose = st.odom.pose.pose
                gt = ps

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

            # Markers for RViz - use different shapes and sizes to distinguish overlapping estimates
            def make_marker(marker_id, pose_msg, color, ns, shape=Marker.SPHERE, z_offset=0.0, scale=0.1):
                import copy
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp_msg
                m.ns = ns
                m.id = marker_id
                m.type = shape
                m.action = Marker.ADD
                # PoseWithCovarianceStamped stores the pose under pose.pose; PoseStamped stores pose
                # Make a deep copy to avoid modifying the original message
                if isinstance(pose_msg, PoseWithCovarianceStamped):
                    m.pose = copy.deepcopy(pose_msg.pose.pose)
                else:
                    m.pose = copy.deepcopy(pose_msg.pose)
                # Set fixed vertical offset to prevent complete overlap when estimates are close
                m.pose.position.z = z_offset
                m.scale.x = m.scale.y = m.scale.z = scale
                m.color.r, m.color.g, m.color.b, m.color.a = color
                m.lifetime.sec = 0
                m.lifetime.nanosec = 0
                return m

            mid_base = idx * 10
            if gt:
                # Ground truth: Green sphere at base level (larger)
                m = make_marker(mid_base + 1, gt, (0.2, 0.9, 0.2, 0.9), f'{name}/gt', 
                               Marker.SPHERE, z_offset=0.0, scale=0.1)
                marker_all.markers.append(m)
                marker_gt.markers.append(m)
                
                # Add axes to ground truth marker for orientation
                axes = Marker()
                axes.header.frame_id = 'map'
                axes.header.stamp = stamp_msg
                axes.ns = f'{name}/gt_axes'
                axes.id = mid_base + 1
                axes.type = Marker.ARROW
                axes.action = Marker.ADD
                axes.pose = copy.deepcopy(gt.pose)
                axes.pose.position.z = 0.0  # Same level as sphere
                axes.scale.x = 0.15  # Arrow length
                axes.scale.y = 0.02  # Arrow width
                axes.scale.z = 0.02  # Arrow height
                axes.color.r, axes.color.g, axes.color.b, axes.color.a = (0.0, 1.0, 0.0, 1.0)  # Bright green
                axes.lifetime.sec = 0
                axes.lifetime.nanosec = 0
                marker_all.markers.append(axes)
                marker_gt.markers.append(axes)
                
                hist = self.path_history[name]['gt']
                hist.append(gt)
                if len(hist) > self.path_history_limit:
                    del hist[0]
            if amcl:
                # AMCL: Blue cube slightly above ground truth
                m = make_marker(mid_base + 2, amcl, (0.2, 0.4, 0.9, 0.9), f'{name}/amcl',
                               Marker.CUBE, z_offset=0.15, scale=0.1)
                marker_all.markers.append(m)
                marker_amcl.markers.append(m)
                hist = self.path_history[name]['amcl']
                ps = PoseStamped()
                ps.header = amcl.header
                ps.pose = amcl.pose.pose
                hist.append(ps)
                if len(hist) > self.path_history_limit:
                    del hist[0]
            if coloc:
                # Coloc: Magenta cylinder at top level
                m = make_marker(mid_base + 3, coloc, (0.9, 0.2, 0.8, 0.9), f'{name}/coloc',
                               Marker.CYLINDER, z_offset=0.30, scale=0.15)
                marker_all.markers.append(m)
                marker_coloc.markers.append(m)
                hist = self.path_history[name]['coloc']
                hist.append(coloc)
                if len(hist) > self.path_history_limit:
                    del hist[0]

        # Always publish the marker topics so RViz sees the namespaces/topics even if empty
        self.marker_pub.publish(marker_all)
        self.marker_pub_gt.publish(marker_gt)
        self.marker_pub_amcl.publish(marker_amcl)
        self.marker_pub_coloc.publish(marker_coloc)

        # Publish paths per robot/source
        for name, histories in self.path_history.items():
            if histories['gt'] and name in self.path_pub_gt:
                p = Path()
                p.header.frame_id = 'map'
                p.header.stamp = stamp_msg
                p.poses = histories['gt']
                self.path_pub_gt[name].publish(p)
            if histories['amcl'] and name in self.path_pub_amcl:
                p = Path()
                p.header.frame_id = 'map'
                p.header.stamp = stamp_msg
                p.poses = histories['amcl']
                self.path_pub_amcl[name].publish(p)
            if histories['coloc'] and name in self.path_pub_coloc:
                p = Path()
                p.header.frame_id = 'map'
                p.header.stamp = stamp_msg
                p.poses = histories['coloc']
                self.path_pub_coloc[name].publish(p)


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
