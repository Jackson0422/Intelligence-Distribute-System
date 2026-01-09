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
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import math
import numpy as np
from collections import defaultdict
from tf2_ros import TransformListener, Buffer
import rclpy.time
import rclpy.duration


class DecentralizedColocAgent(Node):
    """Decentralized collaborative localization agent."""

    def __init__(self):
        super().__init__('decentralized_coloc_agent')

        # Declare parameters
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('peer_ids', ['tb3_1'])
        self.declare_parameter('gossip_rate', 1.0)  # Hz
        self.declare_parameter('self_weight', 0.7)  # Self weight (kept for compatibility, no longer used in averaging)
        self.declare_parameter('peer_timeout', 3.0)  # seconds
        self.declare_parameter('correction_threshold', 0.001)  # meters
        # EKF collaboration parameters
        self.declare_parameter('relative_obs_std_xy', 0.10)  # Relative observation xy noise (m)
        self.declare_parameter('relative_obs_std_yaw', 0.087)  # Relative observation yaw noise (rad, ~5 deg)
        self.declare_parameter('max_comm_range', 3.0)  # Communication range threshold (m)
        self.declare_parameter('mahalanobis_threshold', 9.0)  # Mahalanobis gate (chi^2_3, p=0.05)

        # Read parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.peer_ids = self.get_parameter('peer_ids').value
        self.gossip_rate = self.get_parameter('gossip_rate').value
        self.self_weight = self.get_parameter('self_weight').value
        self.peer_timeout = self.get_parameter('peer_timeout').value
        self.correction_threshold = self.get_parameter('correction_threshold').value

        # Validate parameters
        if self.gossip_rate <= 0.0:
            raise ValueError(f"gossip_rate must be > 0, got {self.gossip_rate}")
        if not (0.0 <= self.self_weight <= 1.0):
            raise ValueError(f"self_weight must be in [0, 1], got {self.self_weight}")
        if self.peer_timeout <= 0.0:
            raise ValueError(f"peer_timeout must be > 0, got {self.peer_timeout}")

        # State variables
        self.amcl_pose = None  # current AMCL estimate
        self.current_belief = None  # current belief
        self.peer_beliefs = defaultdict(dict)  # {peer_id: {'pose': ..., 'timestamp': ...}}
        self.peer_subs = []  # keep subscription handles to avoid GC
        self.last_stats_time = 0.0  # last stats print timestamp

        # TF listener (for relative poses)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Initialized collaborative localization agent: {self.robot_id}')
        self.get_logger().info(f'  Peers: {self.peer_ids}')
        self.get_logger().info(f'  Self weight: {self.self_weight}')
        self.get_logger().info(f'  Gossip rate: {self.gossip_rate} Hz')

        # Setup communication
        self._setup_communication()

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
        if self.robot_id == 'tb3_0':
            amcl_topic = '/amcl_pose'
        else:
            amcl_topic = f'/{self.robot_id}/amcl_pose'

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self.amcl_callback,
            qos_reliable
        )

        # Publish collaborative localization results
        if self.robot_id == 'tb3_0':
            coloc_topic = '/coloc_pose'
            belief_topic = '/coloc_belief'
        else:
            coloc_topic = f'/{self.robot_id}/coloc_pose'
            belief_topic = f'/{self.robot_id}/coloc_belief'

        self.coloc_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            coloc_topic,
            qos_reliable
        )

        # Publish belief (String + JSON)
        self.belief_pub = self.create_publisher(
            String,
            belief_topic,
            qos_best_effort
        )

        # Subscribe to neighbors' belief
        for peer_id in self.peer_ids:
            if peer_id == 'tb3_0':
                peer_belief_topic = '/coloc_belief'
            else:
                peer_belief_topic = f'/{peer_id}/coloc_belief'

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

        # 1. Remove expired neighbors
        current_time = self.get_clock().now().nanoseconds / 1e9
        peers_to_remove = []
        for peer_id, belief_data in self.peer_beliefs.items():
            # Use local receive time to avoid clock sync issues
            if current_time - belief_data['recv_time'] > self.peer_timeout:
                peers_to_remove.append(peer_id)

        for peer_id in peers_to_remove:
            del self.peer_beliefs[peer_id]
            self.get_logger().warning(f'Neighbor {peer_id} timed out and was removed')

        # 2. Broadcast current belief
        self.broadcast_belief()

        # 3. If neighbors exist, run consensus fusion
        if len(self.peer_beliefs) > 0:
            consensus_pose = self.compute_consensus()

            if consensus_pose is not None:
                # Compute correction distance
                dx = consensus_pose.pose.pose.position.x - self.current_belief.pose.pose.position.x
                dy = consensus_pose.pose.pose.position.y - self.current_belief.pose.pose.position.y
                correction_dist = math.sqrt(dx**2 + dy**2)

                # Always publish the fused pose (even if below threshold)
                self.publish_correction(consensus_pose)

                # Option B: always update belief if fusion succeeded, so broadcasts stay current
                # correction_threshold only counts significant corrections, it no longer blocks updates
                self.current_belief = consensus_pose
                if correction_dist > self.correction_threshold:
                    self.correction_count += 1
        else:
            # Even with no neighbors, publish current belief (from AMCL)
            self.publish_correction(self.current_belief)

        # 4. Print stats periodically (every 10 seconds)
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
        # Prior from AMCL
        x_prior, y_prior, yaw_prior = self.extract_pose(self.amcl_pose)
        state_prior = np.array([x_prior, y_prior, yaw_prior])

        cov_amcl = np.array(self.amcl_pose.pose.covariance).reshape(6, 6)
        # Extract 3x3 block (x, y, yaw)
        P_prior = np.array([
            [cov_amcl[0, 0], cov_amcl[0, 1], cov_amcl[0, 5]],
            [cov_amcl[1, 0], cov_amcl[1, 1], cov_amcl[1, 5]],
            [cov_amcl[5, 0], cov_amcl[5, 1], cov_amcl[5, 5]]
        ])

        # If no neighbors, return AMCL
        if len(self.peer_beliefs) == 0:
            return self.amcl_pose

        # EKF update for each neighbor
        state_est = state_prior.copy()
        P_est = P_prior.copy()

        max_range = self.get_parameter('max_comm_range').value
        maha_thresh = self.get_parameter('mahalanobis_threshold').value

        update_count = 0  # successful updates

        for peer_id, belief_data in self.peer_beliefs.items():
            # Neighbor pose
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

            # Generate relative observation
            z_rel = self.generate_relative_observation(peer_id)
            if z_rel is None:
                continue

            # Gate 1: communication range
            if np.linalg.norm(z_rel[:2]) > max_range:
                self.get_logger().debug(
                    f'Gate reject {peer_id}: distance {np.linalg.norm(z_rel[:2]):.2f}m > {max_range}m'
                )
                continue

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

            # Innovation
            innovation = z_rel - z_pred
            innovation[2] = self.wrap_angle(innovation[2])  # wrap angle residual

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
                    continue
            except np.linalg.LinAlgError:
                self.get_logger().warning(f'Singular covariance matrix, skipping {peer_id}')
                continue

            # EKF update
            K = P_est @ H.T @ S_inv
            state_est = state_est + K @ innovation
            state_est[2] = self.wrap_angle(state_est[2])  # wrap yaw
            P_est = (np.eye(3) - K @ H) @ P_est

            update_count += 1

        # Log update info
        if update_count > 0:
            self.get_logger().debug(f'EKF updates: {update_count}/{len(self.peer_beliefs)} neighbors')

        # Build output message
        consensus_pose = PoseWithCovarianceStamped()
        consensus_pose.header.frame_id = self.amcl_pose.header.frame_id
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

        return consensus_pose

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

    def _get_base_frame(self, robot_id):
        """
        Get the robot's base_footprint frame name.
        Handles the special tb3_0 naming (no prefix, backward compatible).

        Args:
            robot_id: 'tb3_0', 'tb3_1', 'tb3_2', 'tb3_3'

        Returns:
            TF frame name (tb3_0 has no prefix, others are namespaced)
        """
        if robot_id == 'tb3_0':
            return 'base_footprint'
        else:
            return f'{robot_id}/base_footprint'

    def generate_relative_observation(self, peer_id):
        """
        Get relative pose from TF and add noise to simulate UWB/vision relative sensing.
        Returns: (dx, dy, dtheta) in self frame, or None if TF is unavailable.
        """
        try:
            # Get ground-truth relative transform: T_self^{-1} * T_peer
            # Use helper to handle tb3_0 naming
            self_frame = self._get_base_frame(self.robot_id)
            peer_frame = self._get_base_frame(peer_id)

            # Avoid blocking lookup_transform(timeout=0.1).
            # If TF is briefly unavailable, blocking would throttle gossip to ~10 Hz,
            # breaking gossip_rate=30 Hz. Use a quick check then immediate lookup.
            if not self.tf_buffer.can_transform(
                self_frame, peer_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.0)
            ):
                return None

            transform = self.tf_buffer.lookup_transform(
                self_frame, peer_frame,
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.0)
            )

            # Extract relative pose
            dx = transform.transform.translation.x
            dy = transform.transform.translation.y
            quat = transform.transform.rotation
            dtheta = self.quaternion_to_yaw({'x': quat.x, 'y': quat.y, 'z': quat.z, 'w': quat.w})

            # Add Gaussian noise
            std_xy = self.get_parameter('relative_obs_std_xy').value
            std_yaw = self.get_parameter('relative_obs_std_yaw').value

            dx_noisy = dx + np.random.normal(0, std_xy)
            dy_noisy = dy + np.random.normal(0, std_xy)
            dtheta_noisy = self.wrap_angle(dtheta + np.random.normal(0, std_yaw))

            return np.array([dx_noisy, dy_noisy, dtheta_noisy])

        except Exception as e:
            self.get_logger().debug(f'Failed to get relative observation for {peer_id}: {e}')
            return None

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
        self.get_logger().info('=' * 60)


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
