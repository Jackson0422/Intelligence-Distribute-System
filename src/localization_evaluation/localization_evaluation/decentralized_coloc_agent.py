#!/usr/bin/env python3
"""
Decentralized Collaborative Localization Agent

Implements a decentralized collaborative localization layer based on the Gossip protocol.
Each robot runs independently, shares pose information via P2P communication, and fuses
multi-source information with a weighted consensus approach.

Author: Distributed Intelligent Systems Course
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import json
import math
import numpy as np
from collections import defaultdict
import tf2_ros
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
import rclpy.time
import rclpy.duration


class DecentralizedColocAgent(Node):
    """Decentralized collaborative localization agent."""

    # Robot physical parameters (meters)
    ROBOT_RADIUS = 0.10  # Consistent with nav2_params

    def __init__(self):
        super().__init__('decentralized_coloc_agent')

        # Declare parameters
        self.declare_parameter('robot_id', 'tb3_1')
        self.declare_parameter('peer_ids', ['tb3_2'])
        self.declare_parameter('gossip_rate', 1.0)  # Hz
        self.declare_parameter('peer_timeout', 3.0)  # seconds
        self.declare_parameter('correction_threshold', 0.001)  # meters
        self.declare_parameter('self_weight', 0.7)  # weight on AMCL vs consensus (0-1)
        self.declare_parameter('max_correction', 0.8)  # meters, <=0 disables clamp
        self.declare_parameter('min_covariance', 1e-4)  # floor for covariance diagonals
        self.declare_parameter('max_peer_updates', 2)  # limit EKF updates per cycle (0 = no limit)
        self.declare_parameter('ambiguity_distance', 0.3)  # meters, <=0 disables ambiguity check
        self.declare_parameter('max_innovation_dist', 0.5)  # meters, <=0 disables gate
        self.declare_parameter('debug_gt', False)  # subscribe to ground truth for debug
        self.declare_parameter('robot_radius', self.ROBOT_RADIUS)  # meters
        self.declare_parameter('peer_search_radius', 0.6)  # meters
        self.declare_parameter('cluster_link_distance', 0.12)  # meters, <=0 uses robot_radius
        self.declare_parameter('min_cluster_points', 4)
        self.declare_parameter('cluster_extent_min', 0.05)  # meters, <=0 uses robot_radius
        self.declare_parameter('cluster_extent_max', 0.45)  # meters, <=0 uses robot_radius
        self.declare_parameter('cluster_span_max', 0.1)  # meters, <=0 disables span gate
        self.declare_parameter('peer_detection_log_period', 1.0)  # seconds, <=0 logs every detection
        # EKF collaboration parameters
        self.declare_parameter('relative_obs_std_xy', 0.10)  # Relative observation xy noise (m)
        self.declare_parameter('relative_obs_std_yaw', 0.087)  # Relative observation yaw noise (rad, ~5 deg)
        self.declare_parameter('max_comm_range', 3.0)  # Communication range threshold (m)
        self.declare_parameter('mahalanobis_threshold', 9.0)  # Mahalanobis gate (chi^2_3, p=0.05)

        # Read parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.peer_ids = self.get_parameter('peer_ids').value
        self.gossip_rate = self.get_parameter('gossip_rate').value
        self.peer_timeout = self.get_parameter('peer_timeout').value
        self.correction_threshold = self.get_parameter('correction_threshold').value
        self.self_weight = float(self.get_parameter('self_weight').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.min_covariance = float(self.get_parameter('min_covariance').value)
        self.max_peer_updates = int(self.get_parameter('max_peer_updates').value)
        self.ambiguity_distance = float(self.get_parameter('ambiguity_distance').value)
        self.max_innovation_dist = float(self.get_parameter('max_innovation_dist').value)
        self.debug_gt = bool(self.get_parameter('debug_gt').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.peer_search_radius = float(self.get_parameter('peer_search_radius').value)
        self.cluster_link_distance = float(self.get_parameter('cluster_link_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.cluster_extent_min = float(self.get_parameter('cluster_extent_min').value)
        self.cluster_extent_max = float(self.get_parameter('cluster_extent_max').value)
        self.cluster_span_max = float(self.get_parameter('cluster_span_max').value)
        self.peer_detection_log_period = float(self.get_parameter('peer_detection_log_period').value)

        # Validate parameters
        if self.gossip_rate <= 0.0:
            raise ValueError(f"gossip_rate must be > 0, got {self.gossip_rate}")
        if self.peer_timeout <= 0.0:
            raise ValueError(f"peer_timeout must be > 0, got {self.peer_timeout}")
        if self.self_weight < 0.0 or self.self_weight > 1.0:
            self.get_logger().warning(
                f'self_weight out of range [0,1], clamping from {self.self_weight}'
            )
            self.self_weight = min(1.0, max(0.0, self.self_weight))
        if self.max_peer_updates < 0:
            self.get_logger().warning(
                f'max_peer_updates < 0, clamping from {self.max_peer_updates} to 0'
            )
            self.max_peer_updates = 0
        if self.min_cluster_points < 1:
            self.get_logger().warning(
                f'min_cluster_points < 1, clamping from {self.min_cluster_points} to 1'
            )
            self.min_cluster_points = 1
        if self.peer_detection_log_period < 0.0:
            self.get_logger().warning(
                f'peer_detection_log_period < 0, clamping from {self.peer_detection_log_period} to 0'
            )
            self.peer_detection_log_period = 0.0

        # State variables
        self.amcl_pose = None  # current AMCL estimate
        self.current_belief = None  # current belief
        self.peer_beliefs = defaultdict(dict)  # {peer_id: {'pose': ..., 'timestamp': ...}}
        self.last_scan = None  # Latest LIDAR scan
        self.peer_subs = []  # keep subscription handles to avoid GC
        self.last_stats_time = 0.0  # last stats print timestamp
        self.last_consensus_stats = {}
        self.last_correction_dist = None
        self.last_correction_clamped = False
        self.last_weight_used = None
        self.gt_pose = None
        self.last_peer_detection_log_time = {}

        # TF listener (for relative poses)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Initialized collaborative localization agent: {self.robot_id}')
        self.get_logger().info(f'  Peers: {self.peer_ids}')
        self.get_logger().info(f'  Gossip rate: {self.gossip_rate} Hz')

        # Setup communication
        self._setup_communication()

        if self.debug_gt:
            gt_topic = f'/{self.robot_id}/ground_truth'
            self.gt_sub = self.create_subscription(
                PoseStamped,
                gt_topic,
                self.gt_callback,
                10
            )
            self.get_logger().info(f'  Subscribed to ground truth: {gt_topic}')

        # Start Gossip timer
        gossip_period = 1.0 / self.gossip_rate
        self.gossip_timer = self.create_timer(gossip_period, self.gossip_callback)

        # Stats counters
        self.amcl_count = 0
        self.peer_msg_count = defaultdict(int)
        self.correction_count = 0

    def _setup_communication(self):
        """Configure ROS communication."""

        # QoS settings
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to this robot's AMCL pose
        amcl_topic = self._get_topic_name('amcl_pose')
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self.amcl_callback,
            qos_reliable
        )

        # Subscribe to LIDAR for relative detection
        scan_topic = self._get_topic_name('scan')
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_best_effort
        )

        # Publish visualization markers on separate topics for easier toggling in Rviz
        # Always use namespaced viz topics (even for tb3_1)
        viz_ns = f'/{self.robot_id}/viz'
        self.pub_viz_estimated = self.create_publisher(Marker, f'{viz_ns}/estimated_pose', 10)
        self.pub_viz_peer = self.create_publisher(Marker, f'{viz_ns}/peer_measurement', 10)
        self.pub_viz_inferred = self.create_publisher(Marker, f'{viz_ns}/inferred_pose', 10)
        self.pub_viz_inferred_arrow = self.create_publisher(Marker, f'{viz_ns}/inferred_vector', 10)
        self.pub_viz_corrected = self.create_publisher(Marker, f'{viz_ns}/corrected_pose', 10)
        self.pub_viz_corrected_arrow = self.create_publisher(Marker, f'{viz_ns}/corrected_vector', 10)

        # Publish collaborative localization results
        coloc_topic = self._get_topic_name('coloc_pose')
        self.coloc_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            coloc_topic,
            qos_reliable
        )

        # Publish belief (String + JSON)
        belief_topic = self._get_topic_name('coloc_belief')
        self.belief_pub = self.create_publisher(
            String,
            belief_topic,
            qos_best_effort
        )

        # Subscribe to neighbors' belief
        for peer_id in self.peer_ids:
            peer_belief_topic = self._get_topic_name('coloc_belief', for_peer_id=peer_id)
            # Keep subscription handles to avoid GC
            sub = self.create_subscription(
                String,
                peer_belief_topic,
                lambda msg, pid=peer_id: self.peer_belief_callback(msg, pid),
                qos_best_effort
            )
            self.peer_subs.append(sub)

            self.get_logger().info(f'  Subscribed to peer belief: {peer_belief_topic}')

        self.get_logger().info(f'  Subscribed to AMCL: {amcl_topic}')
        self.get_logger().info(f'  Publishing collaborative pose: {coloc_topic}')
        self.get_logger().info(f'  Publishing belief: {belief_topic}')

    def scan_callback(self, msg):
        """Receive LIDAR scan."""
        self.last_scan = msg

    def amcl_callback(self, msg):
        """Receive AMCL pose estimate."""
        self.amcl_pose = msg
        self.amcl_count += 1

        # Initialize belief
        if self.current_belief is None:
            self.current_belief = msg
            self.get_logger().info(f'Initialized belief: ({msg.pose.pose.position.x:.3f}, '
                                 f'{msg.pose.pose.position.y:.3f})')

    def peer_belief_callback(self, msg, peer_id):
        """Receive a neighbor's belief message."""
        try:
            data = json.loads(msg.data)

            # Use local receive time to avoid clock sync issues
            current_time = self.get_clock().now().nanoseconds / 1e9

            # Parse pose
            pose_data = {
                'position': {
                    'x': data['pose']['position']['x'],
                    'y': data['pose']['position']['y'],
                    'z': data['pose']['position']['z']
                },
                'orientation': {
                    'x': data['pose']['orientation']['x'],
                    'y': data['pose']['orientation']['y'],
                    'z': data['pose']['orientation']['z'],
                    'w': data['pose']['orientation']['w']
                },
                'covariance': data['pose']['covariance'],
                'recv_time': current_time,  # local receive time (for timeout)
                'src_time': data['timestamp']  # remote timestamp (optional, for logs)
            }

            self.peer_beliefs[peer_id] = pose_data
            self.peer_msg_count[peer_id] += 1

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warning(f'Failed to parse belief from {peer_id}: {e}')

    def gossip_callback(self):
        """Periodic Gossip callback."""

        if self.amcl_pose is None or self.current_belief is None:
            return

        # --- STEP 1: Maintenance ---
        # Remove neighbors that haven't communicated recently (timeout)
        current_time = self.get_clock().now().nanoseconds / 1e9
        peers_to_remove = []
        for peer_id, belief_data in self.peer_beliefs.items():
            # Use local receive time to avoid clock sync issues
            if current_time - belief_data['recv_time'] > self.peer_timeout:
                peers_to_remove.append(peer_id)

        for peer_id in peers_to_remove:
            del self.peer_beliefs[peer_id]
            self.get_logger().warning(f'Neighbor {peer_id} timed out and was removed')

        # --- STEP 2: Fusion ---
        # If we have active neighbors, use their beliefs to correct our own position
        # using the Weighted Consensus / EKF approach.
        update_count = 0
        if len(self.peer_beliefs) > 0:
            consensus_pose, update_count = self.compute_consensus()
        else:
            consensus_pose = self.amcl_pose

        if consensus_pose is None:
            return

        # Limit extreme corrections and blend with AMCL based on self_weight
        limited_pose = self.limit_correction(self.amcl_pose, consensus_pose)
        fused_pose = self.blend_with_amcl(self.amcl_pose, limited_pose, update_count)

        # Compute correction distance vs AMCL
        dx = fused_pose.pose.pose.position.x - self.amcl_pose.pose.pose.position.x
        dy = fused_pose.pose.pose.position.y - self.amcl_pose.pose.pose.position.y
        correction_dist = math.hypot(dx, dy)
        self.last_correction_dist = correction_dist

        # Always publish the fused pose (even if below threshold)
        self.publish_correction(fused_pose)

        # Update belief so broadcasts stay current
        self.current_belief = fused_pose
        if correction_dist > self.correction_threshold:
            self.correction_count += 1

        # --- STEP 3: Communication ---
        # Broadcast our current belief (position + covariance) to all peers
        self.broadcast_belief()

        # --- STEP 4: Visualization ---
        self.publish_consensus_markers(self.amcl_pose, fused_pose, update_count)

        # --- STEP 5: Statistics ---
        # Print stats periodically (every 10 seconds)
        # Use time delta to avoid division-by-zero issues
        if current_time - self.last_stats_time >= 10.0:
            self.print_statistics()
            self.last_stats_time = current_time

    def broadcast_belief(self):
        """Broadcast current belief."""
        if self.current_belief is None:
            return

        # Build JSON message
        belief_data = {
            'robot_id': self.robot_id,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'pose': {
                'position': {
                    'x': self.current_belief.pose.pose.position.x,
                    'y': self.current_belief.pose.pose.position.y,
                    'z': self.current_belief.pose.pose.position.z
                },
                'orientation': {
                    'x': self.current_belief.pose.pose.orientation.x,
                    'y': self.current_belief.pose.pose.orientation.y,
                    'z': self.current_belief.pose.pose.orientation.z,
                    'w': self.current_belief.pose.pose.orientation.w
                },
                'covariance': list(self.current_belief.pose.covariance)
            }
        }

        msg = String()
        msg.data = json.dumps(belief_data)
        self.belief_pub.publish(msg)

    def compute_consensus(self):
        """
        EKF-constrained update (replaces the original weighted average).
        State estimates only itself: x_i = [x, y, theta].
        Neighbor poses are treated as constraints via relative observations.
        """
        # --- Step 1: Initialization (Prior) ---
        # Initialize the state estimate with our current belief (fused or AMCL)
        prior_pose = self.current_belief if self.current_belief is not None else self.amcl_pose
        if prior_pose is None:
            return self.amcl_pose, 0
        x_prior, y_prior, yaw_prior = self.extract_pose(prior_pose)
        state_prior = np.array([x_prior, y_prior, yaw_prior])

        cov_prior = np.array(prior_pose.pose.covariance).reshape(6, 6)
        # Extract 3x3 block (x, y, yaw)
        P_prior = np.array([
            [cov_prior[0, 0], cov_prior[0, 1], cov_prior[0, 5]],
            [cov_prior[1, 0], cov_prior[1, 1], cov_prior[1, 5]],
            [cov_prior[5, 0], cov_prior[5, 1], cov_prior[5, 5]]
        ])

        # If no neighbors, return AMCL
        if len(self.peer_beliefs) == 0:
            return self.amcl_pose, 0

        # --- Step 2: Iterative EKF Update ---
        # Iterate through each neighbor to perform an EKF update
        state_est = state_prior.copy()
        P_est = self.apply_covariance_floor(P_prior.copy(), self.min_covariance)

        max_range = self.get_parameter('max_comm_range').value
        maha_thresh = self.get_parameter('mahalanobis_threshold').value

        update_count = 0  # successful updates
        detections = 0
        range_rejects = 0
        innovation_rejects = 0
        maha_rejects = 0
        candidates = []

        for peer_id, belief_data in self.peer_beliefs.items():
            # Step 2a: Get the neighbor's belief (their claimed position)
            x_j = belief_data['position']['x']
            y_j = belief_data['position']['y']
            peer_quat = belief_data['orientation']
            theta_j = self.quaternion_to_yaw(peer_quat)

            peer_cov = np.array(belief_data['covariance']).reshape(6, 6)
            cov_j = np.array([
                [peer_cov[0, 0], peer_cov[0, 1], peer_cov[0, 5]],
                [peer_cov[1, 0], peer_cov[1, 1], peer_cov[1, 5]],
                [peer_cov[5, 0], peer_cov[5, 1], peer_cov[5, 5]]
            ])
            cov_j = self.apply_covariance_floor(cov_j, self.min_covariance)

            # Step 2b: Measure the neighbor's relative position using our sensors (LIDAR)
            z_rel = self.generate_relative_observation(peer_id)
            if z_rel is None:
                continue
            detections += 1

            # Gate 1: communication range
            dist = np.linalg.norm(z_rel[:2])
            if dist > max_range:
                self.get_logger().debug(
                    f'Gate reject {peer_id}: distance {dist:.2f}m > {max_range}m'
                )
                range_rejects += 1
                continue

            candidates.append((dist, peer_id, belief_data, z_rel, cov_j, theta_j, x_j, y_j))

        # Prefer closer neighbors to reduce confusion with many robots
        candidates.sort(key=lambda item: item[0])
        if self.max_peer_updates > 0:
            candidates = candidates[:self.max_peer_updates]
        selected_count = len(candidates)

        for _, peer_id, belief_data, z_rel, cov_j, theta_j, x_j, y_j in candidates:

            # Step 2c: Predict where the neighbor *should* be relative to us
            # Predicted measurement: h(x_i, x_j) = neighbor in self frame
            x_i, y_i, theta_i = state_est
            c, s = math.cos(theta_i), math.sin(theta_i)

            dx = x_j - x_i
            dy = y_j - y_i

            z_pred = np.array([
                c * dx + s * dy,
                -s * dx + c * dy,
                self.wrap_angle(theta_j - theta_i)
            ])

            # Jacobian H_i = ∂h/∂x_i (3x3)
            H = np.array([
                [-c,        -s,         -s * dx + c * dy],
                [s,         -c,         -c * dx - s * dy],
                [0,          0,         -1]
            ])

            # Effective measurement noise: R_eff = R + neighbor uncertainty
            std_xy = self.get_parameter('relative_obs_std_xy').value
            std_yaw = self.get_parameter('relative_obs_std_yaw').value
            R = np.diag([std_xy**2, std_xy**2, std_yaw**2])

            # Conservative: add neighbor covariance diagonal to R
            R_eff = R + np.diag([cov_j[0, 0], cov_j[1, 1], cov_j[2, 2]])

            # Step 2d: Calculate the Innovation (Measurement - Prediction)
            innovation = z_rel - z_pred
            innovation[2] = self.wrap_angle(innovation[2])  # wrap angle residual
            if self.max_innovation_dist > 0.0:
                innov_norm = np.linalg.norm(innovation[:2])
                if innov_norm > self.max_innovation_dist:
                    innovation_rejects += 1
                    continue

            # Innovation covariance
            S = H @ P_est @ H.T + R_eff

            # Gate 2: Mahalanobis distance
            try:
                S_inv = np.linalg.inv(S)
                maha_dist = innovation.T @ S_inv @ innovation
                if maha_dist > maha_thresh:
                    self.get_logger().debug(
                        f'Gate reject {peer_id}: Mahalanobis {maha_dist:.1f} > {maha_thresh}'
                    )
                    maha_rejects += 1
                    continue
            except np.linalg.LinAlgError:
                self.get_logger().warning(f'Singular covariance matrix, skipping {peer_id}')
                continue

            # Step 2e: Update our estimate using the Kalman Gain
            K = P_est @ H.T @ S_inv
            state_est = state_est + K @ innovation
            state_est[2] = self.wrap_angle(state_est[2])  # wrap yaw
            P_est = (np.eye(3) - K @ H) @ P_est

            update_count += 1

        # Log update info
        if update_count > 0:
            self.get_logger().debug(f'EKF updates: {update_count}/{len(self.peer_beliefs)} neighbors')
        self.last_consensus_stats = {
            'peers': len(self.peer_beliefs),
            'detections': detections,
            'range_rejects': range_rejects,
            'innovation_rejects': innovation_rejects,
            'maha_rejects': maha_rejects,
            'selected': selected_count,
            'updates': update_count,
        }

        # Build output message
        consensus_pose = PoseWithCovarianceStamped()
        consensus_pose.header.frame_id = prior_pose.header.frame_id
        consensus_pose.header.stamp = self.get_clock().now().to_msg()

        consensus_pose.pose.pose.position.x = state_est[0]
        consensus_pose.pose.pose.position.y = state_est[1]
        consensus_pose.pose.pose.position.z = 0.0

        quat = self.yaw_to_quaternion(state_est[2])
        consensus_pose.pose.pose.orientation.x = quat[0]
        consensus_pose.pose.pose.orientation.y = quat[1]
        consensus_pose.pose.pose.orientation.z = quat[2]
        consensus_pose.pose.pose.orientation.w = quat[3]

        # Fill covariance (6x6)
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[0:2, 0:2] = P_est[0:2, 0:2]  # x,y block
        cov_6x6[0:2, 5] = P_est[0:2, 2]      # x,y to yaw cross
        cov_6x6[5, 0:2] = P_est[2, 0:2]
        cov_6x6[5, 5] = P_est[2, 2]          # yaw
        consensus_pose.pose.covariance = list(cov_6x6.flatten())

        return consensus_pose, update_count

    def publish_correction(self, pose):
        """Publish corrected pose."""
        self.coloc_pub.publish(pose)

    def extract_pose(self, pose_msg):
        """Extract x, y, yaw from PoseWithCovarianceStamped."""
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        quat = pose_msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw({
            'x': quat.x, 'y': quat.y, 'z': quat.z, 'w': quat.w
        })
        return x, y, yaw

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (quat['w'] * quat['z'] + quat['x'] * quat['y'])
        cosy_cosp = 1 - 2 * (quat['y']**2 + quat['z']**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (roll=0, pitch=0)."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def wrap_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def apply_covariance_floor(self, cov, min_cov):
        """Clamp covariance diagonal to avoid singular matrices."""
        if min_cov <= 0.0:
            return cov
        cov_out = cov.copy()
        for i in range(min(cov_out.shape[0], cov_out.shape[1])):
            if cov_out[i, i] < min_cov:
                cov_out[i, i] = min_cov
        return cov_out

    def limit_correction(self, amcl_pose, consensus_pose):
        """Limit correction magnitude relative to AMCL."""
        if amcl_pose is None or consensus_pose is None:
            self.last_correction_clamped = False
            return consensus_pose
        if self.max_correction <= 0.0:
            self.last_correction_clamped = False
            return consensus_pose

        ax, ay, ayaw = self.extract_pose(amcl_pose)
        cx, cy, cyaw = self.extract_pose(consensus_pose)
        dx = cx - ax
        dy = cy - ay
        dist = math.hypot(dx, dy)
        if dist <= self.max_correction:
            self.last_correction_clamped = False
            return consensus_pose

        scale = self.max_correction / dist
        new_x = ax + dx * scale
        new_y = ay + dy * scale
        dyaw = self.wrap_angle(cyaw - ayaw)
        new_yaw = self.wrap_angle(ayaw + dyaw * scale)

        limited = PoseWithCovarianceStamped()
        limited.header.frame_id = consensus_pose.header.frame_id
        limited.header.stamp = self.get_clock().now().to_msg()
        limited.pose.pose.position.x = new_x
        limited.pose.pose.position.y = new_y
        limited.pose.pose.position.z = consensus_pose.pose.pose.position.z
        q = self.yaw_to_quaternion(new_yaw)
        limited.pose.pose.orientation.x = q[0]
        limited.pose.pose.orientation.y = q[1]
        limited.pose.pose.orientation.z = q[2]
        limited.pose.pose.orientation.w = q[3]
        limited.pose.covariance = list(consensus_pose.pose.covariance)
        self.last_correction_clamped = True
        return limited

    def blend_with_amcl(self, amcl_pose, consensus_pose, update_count):
        """Blend consensus with AMCL using self_weight."""
        if amcl_pose is None or consensus_pose is None:
            return consensus_pose
        if self.self_weight <= 0.0:
            self.last_weight_used = 0.0
            return consensus_pose
        if self.self_weight >= 1.0:
            self.last_weight_used = 1.0
            return amcl_pose

        ax, ay, ayaw = self.extract_pose(amcl_pose)
        cx, cy, cyaw = self.extract_pose(consensus_pose)
        weight = self.self_weight
        if update_count > 1:
            weight = max(0.2, self.self_weight / float(update_count))
        self.last_weight_used = weight

        out = PoseWithCovarianceStamped()
        out.header.frame_id = consensus_pose.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.pose.position.x = weight * ax + (1.0 - weight) * cx
        out.pose.pose.position.y = weight * ay + (1.0 - weight) * cy
        out.pose.pose.position.z = consensus_pose.pose.pose.position.z

        dyaw = self.wrap_angle(cyaw - ayaw)
        blended_yaw = self.wrap_angle(ayaw + (1.0 - weight) * dyaw)
        q = self.yaw_to_quaternion(blended_yaw)
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]

        amcl_cov = np.array(amcl_pose.pose.covariance)
        cons_cov = np.array(consensus_pose.pose.covariance)
        out.pose.covariance = list((weight * amcl_cov + (1.0 - weight) * cons_cov).flatten())
        return out

    def publish_consensus_markers(self, amcl_pose, fused_pose, update_count):
        """Publish markers for the final cooperative correction."""
        if amcl_pose is None or fused_pose is None:
            return

        timestamp = self.get_clock().now().to_msg()
        frame_id = fused_pose.header.frame_id or 'map'

        # Consensus pose marker (cyan)
        m_pose = Marker()
        m_pose.header.frame_id = frame_id
        m_pose.header.stamp = timestamp
        m_pose.ns = 'corrected_pose'
        m_pose.id = 0
        m_pose.type = Marker.SPHERE
        m_pose.action = Marker.ADD
        m_pose.pose.position.x = fused_pose.pose.pose.position.x
        m_pose.pose.position.y = fused_pose.pose.pose.position.y
        m_pose.pose.position.z = 0.35
        m_pose.pose.orientation.w = 1.0
        m_pose.scale.x = 0.08
        m_pose.scale.y = 0.08
        m_pose.scale.z = 0.08
        m_pose.color.a = 0.9
        m_pose.color.r = 0.0
        m_pose.color.g = 0.7
        m_pose.color.b = 1.0

        # Correction vector marker (green if updated, gray otherwise)
        start = Point()
        start.x = amcl_pose.pose.pose.position.x
        start.y = amcl_pose.pose.pose.position.y
        start.z = 0.1
        end = Point()
        end.x = fused_pose.pose.pose.position.x
        end.y = fused_pose.pose.pose.position.y
        end.z = 0.1

        m_vec = Marker()
        m_vec.header.frame_id = frame_id
        m_vec.header.stamp = timestamp
        m_vec.ns = 'corrected_vector'
        m_vec.id = 1
        m_vec.type = Marker.ARROW
        m_vec.action = Marker.ADD
        m_vec.points = [start, end]
        m_vec.scale.x = 0.015
        m_vec.scale.y = 0.03
        m_vec.scale.z = 0.03
        if update_count > 0:
            m_vec.color.r = 0.1
            m_vec.color.g = 0.9
            m_vec.color.b = 0.1
            m_vec.color.a = 0.9
        else:
            m_vec.color.r = 0.6
            m_vec.color.g = 0.6
            m_vec.color.b = 0.6
            m_vec.color.a = 0.6

        self.pub_viz_corrected.publish(m_pose)
        self.pub_viz_corrected_arrow.publish(m_vec)

    def _get_topic_name(self, topic_base: str, for_peer_id: str = None) -> str:
        """Get a namespaced topic name, handling the special case for tb3_1."""
        robot_id = for_peer_id if for_peer_id is not None else self.robot_id
        # Always use namespaced topics for all robots
        return f'/{robot_id}/{topic_base}'

    def _get_base_frame(self, robot_id):
        """
        Get the robot's base_footprint frame name.
        Handles the special tb3_1 naming (no prefix, backward compatible).

        Args:
            robot_id: 'tb3_1', 'tb3_2', 'tb3_3', 'tb3_4'
        """
        return f'{robot_id}/base_footprint'

    def generate_relative_observation(self, peer_id):
        """
        Generate relative observation using LIDAR and Beliefs (No Ground Truth).
        Detects the peer robot in the LIDAR scan by looking near its expected position.
        """
        if self.last_scan is None or self.amcl_pose is None:
            return None
            
        if peer_id not in self.peer_beliefs:
            return None

        try:
            # --- Step 1: Get Self Pose (Estimated) ---
            self_pose = self.current_belief if self.current_belief is not None else self.amcl_pose
            if self_pose is None:
                return None
            x_i, y_i, yaw_i = self.extract_pose(self_pose)
            
            # --- Step 2: Get Peer Pose (Belief) ---
            belief = self.peer_beliefs[peer_id]
            x_j = belief['position']['x']
            y_j = belief['position']['y']
            
            # --- Step 3: Ambiguity Check ---
            # If another robot is very close to where we expect this peer,
            # we might confuse them. Skip detection to be safe.
            if self.ambiguity_distance > 0.0:
                ambiguity_thresh_sq = self.ambiguity_distance ** 2
                for other_id, other_belief in self.peer_beliefs.items():
                    if other_id == peer_id:
                        continue
                    ox = other_belief['position']['x']
                    oy = other_belief['position']['y']
                    if (x_j - ox)**2 + (y_j - oy)**2 < ambiguity_thresh_sq:
                        return None
            # -----------------------------------

            # Peer orientation for dtheta (since lidar can't measure it)
            q_j = belief['orientation']
            yaw_j = self.quaternion_to_yaw(q_j)

            # --- Step 4: Calculate Expected Relative Position ---
            # Calculate where we expect the peer to be in our local frame (base_footprint)
            dx_global = x_j - x_i
            dy_global = y_j - y_i
            
            # Rotate to local frame
            cos_i = math.cos(yaw_i)
            sin_i = math.sin(yaw_i)
            
            expected_dx = cos_i * dx_global + sin_i * dy_global
            expected_dy = -sin_i * dx_global + cos_i * dy_global
            
            # --- Step 5: Process LIDAR to find the robot ---
            # Get TF: base_footprint -> base_scan (to transform scan points)
            self_frame = self._get_base_frame(self.robot_id)
            scan_frame = self.last_scan.header.frame_id
            
            if not self.tf_buffer.can_transform(self_frame, scan_frame, rclpy.time.Time()):
                return None
                
            tf_scan_to_base = self.tf_buffer.lookup_transform(
                self_frame, scan_frame, rclpy.time.Time())
            
            # Extract transform
            tx = tf_scan_to_base.transform.translation.x
            ty = tf_scan_to_base.transform.translation.y
            tq = tf_scan_to_base.transform.rotation
            tyaw = self.quaternion_to_yaw({'x':tq.x, 'y':tq.y, 'z':tq.z, 'w':tq.w})
            
            # Convert scan to Cartesian in base_footprint
            ranges = np.array(self.last_scan.ranges)
            angles = self.last_scan.angle_min + np.arange(len(ranges)) * self.last_scan.angle_increment
            
            # Filter invalid ranges
            valid_mask = (ranges > self.last_scan.range_min) & (ranges < self.last_scan.range_max)
            r_valid = ranges[valid_mask]
            a_valid = angles[valid_mask]
            
            # Polar to Cartesian (in scan frame)
            x_scan = r_valid * np.cos(a_valid)
            y_scan = r_valid * np.sin(a_valid)
            
            # Transform to base_footprint
            c_t = math.cos(tyaw)
            s_t = math.sin(tyaw)
            x_base = x_scan * c_t - y_scan * s_t + tx
            y_base = x_scan * s_t + y_scan * c_t + ty
            
            # --- Step 6: Association (Find points near expected position) ---
            dist_sq = (x_base - expected_dx)**2 + (y_base - expected_dy)**2
            search_radius = self.peer_search_radius
            
            matches = dist_sq < (search_radius**2)
            
            if np.sum(matches) < self.min_cluster_points: # Need at least a few points to confirm detection
                return None
                
            # --- Step 7: Cluster Selection (Size vs Obstacles) ---
            matched_x = x_base[matches]
            matched_y = y_base[matches]
            matched_x_scan = x_scan[matches]
            matched_y_scan = y_scan[matches]

            link_dist = self.cluster_link_distance
            if link_dist <= 0.0:
                link_dist = max(self.robot_radius * 1.5, 0.08)

            deltas = np.hypot(np.diff(matched_x), np.diff(matched_y))
            split_indices = np.where(deltas > link_dist)[0] + 1
            clusters = np.split(np.arange(len(matched_x)), split_indices)

            min_extent = self.cluster_extent_min
            max_extent = self.cluster_extent_max
            if min_extent <= 0.0:
                min_extent = max(0.02, self.robot_radius * 0.5)
            if max_extent <= 0.0:
                max_extent = self.robot_radius * 4.0

            best_cluster = None
            best_key = None
            for cluster_idx in clusters:
                if len(cluster_idx) < self.min_cluster_points:
                    continue
                cluster_x = matched_x[cluster_idx]
                cluster_y = matched_y[cluster_idx]
                if self.cluster_span_max > 0.0:
                    span = math.hypot(
                        float(cluster_x[-1] - cluster_x[0]),
                        float(cluster_y[-1] - cluster_y[0])
                    )
                    if span > self.cluster_span_max:
                        continue
                extent = math.hypot(
                    float(cluster_x.max() - cluster_x.min()),
                    float(cluster_y.max() - cluster_y.min())
                )
                if extent < min_extent or extent > max_extent:
                    continue
                centroid_x = float(np.mean(cluster_x))
                centroid_y = float(np.mean(cluster_y))
                dist_to_expected = math.hypot(centroid_x - expected_dx, centroid_y - expected_dy)
                key = (dist_to_expected, extent)
                if best_key is None or key < best_key:
                    best_key = key
                    best_cluster = cluster_idx

            if best_cluster is None:
                return None
            # ---------------------------------------

            # --- Step 8: Measurement (Calculate Centroid) ---
            cluster_x = matched_x[best_cluster]
            cluster_y = matched_y[best_cluster]
            cluster_x_scan = matched_x_scan[best_cluster]
            cluster_y_scan = matched_y_scan[best_cluster]

            measured_dx = np.mean(cluster_x)
            measured_dy = np.mean(cluster_y)
            measured_dx_scan = np.mean(cluster_x_scan)
            measured_dy_scan = np.mean(cluster_y_scan)
            
            # Calculate dtheta (from beliefs, as LIDAR can't see orientation)
            measured_dtheta = self.wrap_angle(yaw_j - yaw_i)
            
            # Add measurement noise (simulating sensor noise)
            # Note: The centroid already has noise from LIDAR, but we can add extra if needed.
            # The EKF expects a noisy measurement.
            
            # VISUALIZATION: Publish markers to see what's happening in Rviz
            self.log_active_neighbor(peer_id, measured_dx, measured_dy)
            self.publish_debug_markers(
                peer_id,
                x_i,
                y_i,
                yaw_i,
                x_j,
                y_j,
                measured_dx,
                measured_dy,
                scan_frame,
                measured_dx_scan,
                measured_dy_scan,
                cluster_x_scan,
                cluster_y_scan,
            )
            
            return np.array([measured_dx, measured_dy, measured_dtheta])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Catch TF-specific errors
            self.get_logger().warn(f'LIDAR detection failed for {peer_id}: {e}')
            return None

    def log_active_neighbor(self, peer_id, meas_dx, meas_dy):
        """Log detections to the terminal with throttling."""
        now = self.get_clock().now().nanoseconds / 1e9
        last_time = self.last_peer_detection_log_time.get(peer_id, 0.0)
        if self.peer_detection_log_period > 0.0 and now - last_time < self.peer_detection_log_period:
            return
        dist = math.hypot(meas_dx, meas_dy)
        self.get_logger().info(
            f'Active neighbor found in scan: {peer_id} at (dx={meas_dx:.2f}, dy={meas_dy:.2f}, r={dist:.2f}m)'
        )
        self.last_peer_detection_log_time[peer_id] = now
            
    def publish_debug_markers(
        self,
        peer_id,
        self_x,
        self_y,
        self_yaw,
        peer_x,
        peer_y,
        meas_dx,
        meas_dy,
        scan_frame,
        meas_dx_scan,
        meas_dy_scan,
        matched_x_scan,
        matched_y_scan,
    ):
        """
        Publish visualization markers for debug on separate topics.
        
        Red: Estimated pose from AMCL - Map Frame
        Green: Measured Peer Position (LIDAR) - Base Frame (Relative)
        Blue: Inferred Self Pose (Calculated from Peer + LIDAR) - Map Frame
        Yellow: Inferred correction vector (AMCL -> inferred)
        """
        timestamp = self.get_clock().now().to_msg()
        frame_id = "map"
        base_frame = self._get_base_frame(self.robot_id)
        
        sphere_scale = 0.08
        # 1. Red Sphere: Self AMCL Pose (Current Estimate)
        m1 = Marker()
        m1.header.frame_id = frame_id
        m1.header.stamp = timestamp
        m1.ns = f"estimated_amcl_{peer_id}"
        m1.id = 0
        m1.type = Marker.SPHERE
        m1.action = Marker.ADD
        m1.pose.position.x = self_x
        m1.pose.position.y = self_y
        m1.pose.position.z = 0.3
        m1.scale.x = sphere_scale; m1.scale.y = sphere_scale; m1.scale.z = sphere_scale
        m1.color.a = 1.0; m1.color.r = 1.0; m1.color.g = 0.0; m1.color.b = 0.0 # Red
        
        # 2. Green Sphere: Measured Peer Position (LIDAR)
        m2 = Marker()
        m2.header.frame_id = base_frame
        m2.header.stamp = timestamp
        m2.ns = f"measured_peer_{peer_id}"
        m2.id = 1
        m2.type = Marker.SPHERE
        m2.action = Marker.ADD
        m2.pose.position.x = meas_dx
        m2.pose.position.y = meas_dy
        m2.pose.position.z = 0.3
        m2.scale.x = sphere_scale; m2.scale.y = sphere_scale; m2.scale.z = sphere_scale
        m2.color.a = 1.0; m2.color.r = 0.0; m2.color.g = 1.0; m2.color.b = 0.0 # Green

        # 2b. Green Points: Matched LIDAR points (scan frame) for visual alignment with LaserScan
        m2_points = Marker()
        m2_points.header.frame_id = scan_frame
        m2_points.header.stamp = timestamp
        m2_points.ns = f"measured_peer_points_{peer_id}"
        m2_points.id = 5
        m2_points.type = Marker.POINTS
        m2_points.action = Marker.ADD
        m2_points.scale.x = 0.02
        m2_points.scale.y = 0.02
        m2_points.color.a = 0.8
        m2_points.color.r = 0.0
        m2_points.color.g = 1.0
        m2_points.color.b = 0.0
        m2_points.points = [
            Point(x=float(px), y=float(py), z=0.0)
            for px, py in zip(matched_x_scan, matched_y_scan)
        ]

        # 2c. Green Sphere: Measured centroid in scan frame (laser point cloud)
        m2_scan = Marker()
        m2_scan.header.frame_id = scan_frame
        m2_scan.header.stamp = timestamp
        m2_scan.ns = f"measured_peer_scan_{peer_id}"
        m2_scan.id = 6
        m2_scan.type = Marker.SPHERE
        m2_scan.action = Marker.ADD
        m2_scan.pose.position.x = float(meas_dx_scan)
        m2_scan.pose.position.y = float(meas_dy_scan)
        m2_scan.pose.position.z = 0.0
        m2_scan.scale.x = 0.05; m2_scan.scale.y = 0.05; m2_scan.scale.z = 0.05
        m2_scan.color.a = 0.9; m2_scan.color.r = 0.0; m2_scan.color.g = 1.0; m2_scan.color.b = 0.0
        
        # 3. Blue Sphere: Inferred Self Pose
        # Calculation: Self = Peer_Global - Rotate(Meas_Local)
        # Rotate measured vector (dx, dy) by self_yaw to get global offset
        cos_a = math.cos(self_yaw)
        sin_a = math.sin(self_yaw)
        
        global_dx = meas_dx * cos_a - meas_dy * sin_a
        global_dy = meas_dx * sin_a + meas_dy * cos_a
        
        inferred_x = peer_x - global_dx
        inferred_y = peer_y - global_dy
        
        m3 = Marker()
        m3.header.frame_id = frame_id
        m3.header.stamp = timestamp
        m3.ns = f"inferred_self_{peer_id}"
        m3.id = 2
        m3.type = Marker.SPHERE
        m3.action = Marker.ADD
        m3.pose.position.x = inferred_x
        m3.pose.position.y = inferred_y
        m3.pose.position.z = 0.3
        m3.scale.x = sphere_scale; m3.scale.y = sphere_scale; m3.scale.z = sphere_scale
        m3.color.a = 1.0; m3.color.r = 0.0; m3.color.g = 0.0; m3.color.b = 1.0 # Blue
        
        # 4. Yellow Arrow: Correction Vector (Red -> Blue)
        m4 = Marker()
        m4.header.frame_id = frame_id
        m4.header.stamp = timestamp
        m4.ns = f"inferred_vector_{peer_id}"
        m4.id = 3
        m4.type = Marker.ARROW
        m4.action = Marker.ADD
        m4.points = [m1.pose.position, m3.pose.position] # From Red to Blue
        m4.scale.x = 0.015; m4.scale.y = 0.03; m4.scale.z = 0.03
        m4.color.a = 0.8; m4.color.r = 1.0; m4.color.g = 1.0; m4.color.b = 0.0 # Yellow
        
        # 5. Magenta Sphere: Ground Truth Position of Peer Robot (from TF)
        m5 = Marker()
        m5.header.frame_id = frame_id
        m5.header.stamp = timestamp
        m5.ns = f"peer_ground_truth_{peer_id}"
        m5.id = 4
        m5.type = Marker.SPHERE
        m5.action = Marker.ADD
        
        try:
            # Query the peer's actual transform in the map frame
            peer_frame = self._get_base_frame(peer_id)
            if self.tf_buffer.can_transform(frame_id, peer_frame, rclpy.time.Time()):
                tf_peer = self.tf_buffer.lookup_transform(frame_id, peer_frame, rclpy.time.Time())
                m5.pose.position.x = tf_peer.transform.translation.x
                m5.pose.position.y = tf_peer.transform.translation.y
                m5.pose.position.z = 0.3
                m5.pose.orientation.x = tf_peer.transform.rotation.x
                m5.pose.orientation.y = tf_peer.transform.rotation.y
                m5.pose.orientation.z = tf_peer.transform.rotation.z
                m5.pose.orientation.w = tf_peer.transform.rotation.w
                m5.scale.x = sphere_scale; m5.scale.y = sphere_scale; m5.scale.z = sphere_scale
                m5.color.a = 1.0; m5.color.r = 1.0; m5.color.g = 0.0; m5.color.b = 1.0 # Magenta
            else:
                return  # Can't get ground truth, skip visualization
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return  # TF lookup failed, skip visualization
        
        self.pub_viz_estimated.publish(m1)
        self.pub_viz_peer.publish(m2)
        self.pub_viz_peer.publish(m2_points)
        self.pub_viz_peer.publish(m2_scan)
        self.pub_viz_inferred.publish(m3)
        self.pub_viz_inferred_arrow.publish(m4)
        self.pub_viz_peer.publish(m5)  # Reusing peer publisher for ground truth

    def print_statistics(self):
        """Print statistics."""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Collaborative localization stats - {self.robot_id}')
        self.get_logger().info(f'  AMCL messages: {self.amcl_count}')
        self.get_logger().info(f'  Corrections: {self.correction_count}')
        self.get_logger().info(f'  Active neighbors: {len(self.peer_beliefs)}')
        for peer_id, count in self.peer_msg_count.items():
            active = '✓' if peer_id in self.peer_beliefs else '✗'
            self.get_logger().info(f'    {peer_id}: {count} messages [{active}]')

        if self.current_belief is not None:
            x, y, yaw = self.extract_pose(self.current_belief)
            self.get_logger().info(f'  Current belief: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')
        if self.last_consensus_stats:
            stats = self.last_consensus_stats
            self.get_logger().info(
                '  Peer obs: peers={peers} detections={detections} selected={selected} '
                'updates={updates} range_rejects={range_rejects} innovation_rejects={innovation_rejects} '
                'maha_rejects={maha_rejects}'.format(**stats)
            )
        if self.last_correction_dist is not None:
            weight_str = f'{self.last_weight_used:.2f}' if self.last_weight_used is not None else 'n/a'
            self.get_logger().info(
                f'  Last correction: dist={self.last_correction_dist:.3f}m '
                f'clamped={self.last_correction_clamped} weight={weight_str}'
            )
        if self.gt_pose is not None and self.amcl_pose is not None and self.current_belief is not None:
            gt_x, gt_y, _ = self.extract_pose_any(self.gt_pose)
            amcl_x, amcl_y, _ = self.extract_pose(self.amcl_pose)
            corr_x, corr_y, _ = self.extract_pose(self.current_belief)
            amcl_err = math.hypot(amcl_x - gt_x, amcl_y - gt_y)
            corr_err = math.hypot(corr_x - gt_x, corr_y - gt_y)
            self.get_logger().info(f'  GT error: amcl={amcl_err:.3f}m corrected={corr_err:.3f}m')
        self.get_logger().info('=' * 60)

    def gt_callback(self, msg):
        """Receive ground truth pose for debug."""
        self.gt_pose = msg

    def extract_pose_any(self, pose_msg):
        """Extract x, y, yaw from PoseStamped or PoseWithCovarianceStamped."""
        if hasattr(pose_msg, 'pose') and hasattr(pose_msg.pose, 'pose'):
            pose = pose_msg.pose.pose
        else:
            pose = pose_msg.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw({
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w,
        })
        return x, y, yaw


def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedColocAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
