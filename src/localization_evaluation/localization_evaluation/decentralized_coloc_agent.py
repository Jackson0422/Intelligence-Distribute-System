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

        # Validate parameters
        if self.gossip_rate <= 0.0:
            raise ValueError(f"gossip_rate must be > 0, got {self.gossip_rate}")
        if self.peer_timeout <= 0.0:
            raise ValueError(f"peer_timeout must be > 0, got {self.peer_timeout}")

        # State variables
        self.amcl_pose = None  # current AMCL estimate
        self.current_belief = None  # current belief
        self.peer_beliefs = defaultdict(dict)  # {peer_id: {'pose': ..., 'timestamp': ...}}
        self.last_scan = None  # Latest LIDAR scan
        self.peer_subs = []  # keep subscription handles to avoid GC
        self.last_stats_time = 0.0  # last stats print timestamp

        # TF listener (for relative poses)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Initialized collaborative localization agent: {self.robot_id}')
        self.get_logger().info(f'  Peers: {self.peer_ids}')
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
        self.pub_viz_self = self.create_publisher(Marker, f'{viz_ns}/self_pose', 10)
        self.pub_viz_peer = self.create_publisher(Marker, f'{viz_ns}/peer_measurement', 10)
        self.pub_viz_inferred = self.create_publisher(Marker, f'{viz_ns}/inferred_pose', 10)
        self.pub_viz_arrow = self.create_publisher(Marker, f'{viz_ns}/correction_vector', 10)

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

        # --- STEP 2: Communication ---
        # Broadcast our current belief (position + covariance) to all peers
        self.broadcast_belief()

        # --- STEP 3: Fusion ---
        # If we have active neighbors, use their beliefs to correct our own position
        # using the Weighted Consensus / EKF approach.
        if len(self.peer_beliefs) > 0:
            consensus_pose = self.compute_consensus()

            if consensus_pose is not None:
                # Compute correction distance
                dx = consensus_pose.pose.pose.position.x - self.current_belief.pose.pose.position.x
                dy = consensus_pose.pose.pose.position.y - self.current_belief.pose.pose.position.y
                correction_dist = math.sqrt(dx**2 + dy**2)

                # Always publish the fused pose (even if below threshold)
                self.publish_correction(consensus_pose)

                # Update belief if fusion succeeded, so broadcasts stay current
                # correction_threshold only counts significant corrections for stats
                self.current_belief = consensus_pose
                if correction_dist > self.correction_threshold:
                    self.correction_count += 1
        else:
            # Even with no neighbors, publish current belief (from AMCL)
            self.publish_correction(self.current_belief)

        # --- STEP 4: Statistics ---
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
        # Initialize the state estimate with our current AMCL pose
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

        # --- Step 2: Iterative EKF Update ---
        # Iterate through each neighbor to perform an EKF update
        state_est = state_prior.copy()
        P_est = P_prior.copy()

        max_range = self.get_parameter('max_comm_range').value
        maha_thresh = self.get_parameter('mahalanobis_threshold').value

        update_count = 0  # successful updates

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

            # Step 2b: Measure the neighbor's relative position using our sensors (LIDAR)
            z_rel = self.generate_relative_observation(peer_id)
            if z_rel is None:
                continue

            # Gate 1: communication range
            if np.linalg.norm(z_rel[:2]) > max_range:
                self.get_logger().debug(
                    f'Gate reject {peer_id}: distance {np.linalg.norm(z_rel[:2]):.2f}m > {max_range}m'
                )
                continue

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

            # Step 2e: Update our estimate using the Kalman Gain
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
            x_i, y_i, yaw_i = self.extract_pose(self.amcl_pose)
            
            # --- Step 2: Get Peer Pose (Belief) ---
            belief = self.peer_beliefs[peer_id]
            x_j = belief['position']['x']
            y_j = belief['position']['y']
            
            # --- Step 3: Ambiguity Check ---
            # If another robot is very close to where we expect this peer, 
            # we might confuse them. Skip detection to be safe.
            for other_id, other_belief in self.peer_beliefs.items():
                if other_id == peer_id:
                    continue
                ox = other_belief['position']['x']
                oy = other_belief['position']['y']
                
                # Threshold: 3 * Robot Size (Diameter)
                ambiguity_thresh = (self.ROBOT_RADIUS * 2) * 3.0
                if (x_j - ox)**2 + (y_j - oy)**2 < ambiguity_thresh**2: 
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
            search_radius = 0.6 # meters (generous radius to catch the robot)
            
            matches = dist_sq < (search_radius**2)
            
            if np.sum(matches) < 3: # Need at least a few points to confirm detection
                return None
                
            # --- Step 7: Cluster Shape Check (Robustness) ---
            # Filter out walls/obstacles. A robot is small/compact.
            # If the points are too spread out, it's likely a wall.
            matched_x = x_base[matches]
            matched_y = y_base[matches]
            std_x = np.std(matched_x)
            std_y = np.std(matched_y)
            
            # TurtleBot3 is ~0.2m. If std > 0.2, spread is likely > 0.6m -> Wall
            if std_x > 0.2 or std_y > 0.2:
                return None
            # ---------------------------------------

            # --- Step 8: Measurement (Calculate Centroid) ---
            measured_dx = np.mean(matched_x)
            measured_dy = np.mean(matched_y)
            
            # Calculate dtheta (from beliefs, as LIDAR can't see orientation)
            measured_dtheta = self.wrap_angle(yaw_j - yaw_i)
            
            # Add measurement noise (simulating sensor noise)
            # Note: The centroid already has noise from LIDAR, but we can add extra if needed.
            # The EKF expects a noisy measurement.
            
            # VISUALIZATION: Publish markers to see what's happening in Rviz
            self.publish_debug_markers(peer_id, x_i, y_i, yaw_i, x_j, y_j, measured_dx, measured_dy)
            
            return np.array([measured_dx, measured_dy, measured_dtheta])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Catch TF-specific errors
            self.get_logger().warn(f'LIDAR detection failed for {peer_id}: {e}')
            return None
            
    def publish_debug_markers(self, peer_id, self_x, self_y, self_yaw, peer_x, peer_y, meas_dx, meas_dy):
        """
        Publish visualization markers for debug on separate topics.
        
        Red: Self AMCL Pose (Measured by Odom/IMU) - Map Frame
        Green: Measured Peer Position (LIDAR) - Base Frame (Relative)
        Blue: Inferred Self Pose (Calculated from Peer + LIDAR) - Map Frame
        Yellow: Correction Vector
        """
        timestamp = self.get_clock().now().to_msg()
        frame_id = "map"
        base_frame = self._get_base_frame(self.robot_id)
        
        # 1. Red Sphere: Self AMCL Pose (Current Estimate)
        m1 = Marker()
        m1.header.frame_id = frame_id
        m1.header.stamp = timestamp
        m1.ns = f"self_amcl_{peer_id}"
        m1.id = 0
        m1.type = Marker.SPHERE
        m1.action = Marker.ADD
        m1.pose.position.x = self_x
        m1.pose.position.y = self_y
        m1.pose.position.z = 0.3
        m1.scale.x = 0.2; m1.scale.y = 0.2; m1.scale.z = 0.2
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
        m2.scale.x = 0.2; m2.scale.y = 0.2; m2.scale.z = 0.2
        m2.color.a = 1.0; m2.color.r = 0.0; m2.color.g = 1.0; m2.color.b = 0.0 # Green
        
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
        m3.ns = f"self_inferred_{peer_id}"
        m3.id = 2
        m3.type = Marker.SPHERE
        m3.action = Marker.ADD
        m3.pose.position.x = inferred_x
        m3.pose.position.y = inferred_y
        m3.pose.position.z = 0.3
        m3.scale.x = 0.2; m3.scale.y = 0.2; m3.scale.z = 0.2
        m3.color.a = 1.0; m3.color.r = 0.0; m3.color.g = 0.0; m3.color.b = 1.0 # Blue
        
        # 4. Yellow Arrow: Correction Vector (Red -> Blue)
        m4 = Marker()
        m4.header.frame_id = frame_id
        m4.header.stamp = timestamp
        m4.ns = f"correction_line_{peer_id}"
        m4.id = 3
        m4.type = Marker.ARROW
        m4.action = Marker.ADD
        m4.points = [m1.pose.position, m3.pose.position] # From Red to Blue
        m4.scale.x = 0.05; m4.scale.y = 0.1; m4.scale.z = 0.1
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
                m5.scale.x = 0.2; m5.scale.y = 0.2; m5.scale.z = 0.2
                m5.color.a = 1.0; m5.color.r = 1.0; m5.color.g = 0.0; m5.color.b = 1.0 # Magenta
            else:
                return  # Can't get ground truth, skip visualization
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return  # TF lookup failed, skip visualization
        
        self.pub_viz_self.publish(m1)
        self.pub_viz_peer.publish(m2)
        self.pub_viz_inferred.publish(m3)
        self.pub_viz_arrow.publish(m4)
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
