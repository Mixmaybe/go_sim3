#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
import tf2_ros
import tf_transformations

from ForwardKinematics import robot_FK
from quadropted_msgs.msg import RobotFootContact, RobotVelocity


# Order required by ForwardKinematics.forward_kinematics_all_legs().
JOINT_ORDER = [
    'rf_hip_joint',
    'rf_upper_leg_joint',
    'rf_lower_leg_joint',
    'lf_hip_joint',
    'lf_upper_leg_joint',
    'lf_lower_leg_joint',
    'rh_hip_joint',
    'rh_upper_leg_joint',
    'rh_lower_leg_joint',
    'lh_hip_joint',
    'lh_upper_leg_joint',
    'lh_lower_leg_joint',
]

LEG_NAMES = ['FR', 'FL', 'RR', 'RL']


class DogOdometry(Node):
    def __init__(self):
        super().__init__('dog_odometry')

        self.declare_parameter('publish_rate', 50)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_legacy_odom', True)
        self.declare_parameter('enable_odom_tf', True)
        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('has_imu_heading', True)
        self.declare_parameter('use_amcl_soft_correction', False)
        self.declare_parameter('amcl_correction_alpha_xy', 0.02)
        self.declare_parameter('amcl_correction_alpha_yaw', 0.02)
        self.declare_parameter('amcl_max_jump', 0.5)
        self.declare_parameter('min_stable_contacts', 1)
        self.declare_parameter('contact_anchor_residual_threshold', 0.08)
        self.declare_parameter('slip_velocity_threshold', 0.08)
        self.declare_parameter('no_contact_mode', 'freeze_xy')
        self.declare_parameter('twist_lowpass_alpha', 0.25)
        self.declare_parameter('pose_lowpass_alpha', 0.15)
        self.declare_parameter('verbose', False)
        self.declare_parameter('is_gazebo', True)
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('measured_contacts_topic', 'foot_contacts_measured')
        self.declare_parameter('expected_contacts_topic', 'foot_contacts_expected')
        self.declare_parameter('imu_topic', 'imu_plugin/out')
        self.declare_parameter('robot_velocity_topic', 'robot_velocity')
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('publish_filtered_odom', False)
        self.declare_parameter('path_max_length', 3000)

        publish_rate = self.get_parameter('publish_rate').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_legacy_odom = self.get_parameter('publish_legacy_odom').value
        self.enable_odom_tf = self.get_parameter('enable_odom_tf').value
        self.use_imu_heading = self.get_parameter('use_imu_heading').value
        self.use_amcl_soft_correction = self.get_parameter('use_amcl_soft_correction').value
        self.amcl_correction_alpha_xy = self.get_parameter('amcl_correction_alpha_xy').value
        self.amcl_correction_alpha_yaw = self.get_parameter('amcl_correction_alpha_yaw').value
        self.amcl_max_jump = self.get_parameter('amcl_max_jump').value
        self.min_stable_contacts = self.get_parameter('min_stable_contacts').value
        self.contact_anchor_residual_threshold = self.get_parameter('contact_anchor_residual_threshold').value
        self.slip_velocity_threshold = self.get_parameter('slip_velocity_threshold').value
        self.no_contact_mode = self.get_parameter('no_contact_mode').value
        self.twist_lowpass_alpha = self.get_parameter('twist_lowpass_alpha').value
        self.pose_lowpass_alpha = self.get_parameter('pose_lowpass_alpha').value
        self.verbose = self.get_parameter('verbose').value
        self.is_gazebo = self.get_parameter('is_gazebo').value
        self.clock_topic = self.get_parameter('clock_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.measured_contacts_topic = self.get_parameter('measured_contacts_topic').value
        self.expected_contacts_topic = self.get_parameter('expected_contacts_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.robot_velocity_topic = self.get_parameter('robot_velocity_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.publish_filtered_odom = self.get_parameter('publish_filtered_odom').value
        self.path_max_length = self.get_parameter('path_max_length').value

        self.steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        body_dimensions = [0.3762, 0.0935]
        leg_dimensions = [0.0, 0.0955, 0.213, 0.213]
        self.fk_solver = robot_FK.ForwardKinematics(body_dimensions, leg_dimensions)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.twist_vx = 0.0
        self.twist_vy = 0.0
        self.twist_wz = 0.0
        self.path_length = 0.0
        self.slip_events_count = 0

        self.joint_positions = None
        self.foot_positions_base = None
        self.foot_anchors_odom = [None, None, None, None]
        self.prev_contacts = [False, False, False, False]
        self.measured_contacts = [False, False, False, False]
        self.expected_contacts = None
        self.prev_foot_odom_positions = [None, None, None, None]
        self.leg_slipping = [False, False, False, False]

        self.command_vx = 0.0
        self.command_vy = 0.0
        self.command_wz = 0.0
        self.imu_yaw_unwrapped = None
        self.last_imu_yaw = None
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.amcl_pose = None
        self.gazebo_clock = Time()
        self.last_update_time = None
        self.last_good_pose_time = None
        self.last_warn_times = {}

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame_id

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )

        self.leg_odom_pub = self.create_publisher(Odometry, 'leg_odom_raw', reliable_qos)
        self.legacy_odom_pub = None
        self.filtered_odom_pub = None
        if self.publish_legacy_odom:
            self.legacy_odom_pub = self.create_publisher(Odometry, 'odom', reliable_qos)
        if self.publish_filtered_odom:
            self.filtered_odom_pub = self.create_publisher(Odometry, 'odometry/filtered', reliable_qos)
        self.debug_pub = self.create_publisher(String, 'leg_odom_debug', reliable_qos)
        self.path_pub = self.create_publisher(Path, 'leg_odom_path', reliable_qos)

        self.create_subscription(JointState, self.joint_states_topic, self.joint_states_callback, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(
            RobotFootContact,
            self.measured_contacts_topic,
            self.measured_contacts_callback,
            reliable_qos
        )
        self.create_subscription(
            RobotFootContact,
            self.expected_contacts_topic,
            self.expected_contacts_callback,
            reliable_qos
        )
        self.create_subscription(RobotVelocity, self.robot_velocity_topic, self.velocity_callback, reliable_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, reliable_qos)
        self.create_subscription(Clock, self.clock_topic, self.clock_callback, reliable_qos)

        if self.enable_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / float(publish_rate),
            self.timer_callback,
            clock=self.steady_clock
        )
        self.get_logger().info('Dog Odometry Node has been started with contact-anchor leg odometry.')
        self.get_logger().info('Odometry uses foot_contacts_measured; foot_contacts_expected are gait schedule debug only.')
        if self.publish_filtered_odom:
            self.get_logger().info('Publishing raw leg odometry compatibility topic odometry/filtered because EKF is disabled.')

    def warn_throttle(self, key, message, period_sec=2.0):
        now = time.monotonic()
        last = self.last_warn_times.get(key, 0.0)
        if now - last >= period_sec:
            self.get_logger().warn(message)
            self.last_warn_times[key] = now

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_to_quaternion(yaw):
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3])
        )

    @staticmethod
    def rotation_2d(yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=float)

    def stamp_now(self):
        if self.is_gazebo and (self.gazebo_clock.sec != 0 or self.gazebo_clock.nanosec != 0):
            return self.gazebo_clock
        return self.get_clock().now().to_msg()

    def joint_states_callback(self, msg):
        joint_positions_by_name = dict(zip(msg.name, msg.position))
        missing_joints = [
            joint_name for joint_name in JOINT_ORDER
            if joint_name not in joint_positions_by_name
        ]

        if missing_joints:
            self.warn_throttle(
                'missing_joints',
                f'JointState message is missing joints: {missing_joints}. Keeping previous joint positions.'
            )
            return

        self.joint_positions = [
            joint_positions_by_name[joint_name]
            for joint_name in JOINT_ORDER
        ]
        if self.verbose:
            self.get_logger().info(f'Joint Positions: {self.joint_positions}')

    def measured_contacts_callback(self, msg):
        if len(msg.contacts) != 4:
            self.warn_throttle(
                'bad_contacts',
                f'Unexpected number of measured contacts: {len(msg.contacts)}. Expected 4.'
            )
            return
        self.measured_contacts = [bool(contact) for contact in msg.contacts]

    def expected_contacts_callback(self, msg):
        if len(msg.contacts) == 4:
            self.expected_contacts = [bool(contact) for contact in msg.contacts]

    def velocity_callback(self, msg):
        self.command_vx = float(msg.cmd_vel.linear.x)
        self.command_vy = float(msg.cmd_vel.linear.y)
        self.command_wz = float(msg.cmd_vel.angular.z)

    def imu_callback(self, msg):
        orientation = msg.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.imu_roll = float(roll)
        self.imu_pitch = float(pitch)

        if self.imu_yaw_unwrapped is None:
            self.imu_yaw_unwrapped = float(yaw)
        else:
            self.imu_yaw_unwrapped += self.normalize_angle(float(yaw) - self.last_imu_yaw)
        self.last_imu_yaw = float(yaw)

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        self.amcl_pose = (float(position.x), float(position.y), float(yaw))

    def clock_callback(self, msg):
        self.gazebo_clock = msg.clock

    def calculate_foot_positions(self):
        if self.joint_positions is None:
            self.warn_throttle('no_joints', 'No complete JointState has been received yet.')
            self.foot_positions_base = None
            return None

        try:
            self.foot_positions_base = self.fk_solver.forward_kinematics_all_legs(self.joint_positions)
            return self.foot_positions_base
        except Exception as exc:
            self.warn_throttle('fk_error', f'Error in forward kinematics: {exc}')
            self.foot_positions_base = None
            return None

    def update_heading(self):
        if self.use_imu_heading and self.imu_yaw_unwrapped is not None:
            self.theta = float(self.imu_yaw_unwrapped)

    def apply_no_contact_fallback(self, dt):
        if self.no_contact_mode == 'integrate_cmd_vel' and dt > 0.0:
            rotation = self.rotation_2d(self.theta)
            delta_world = rotation @ np.array([self.command_vx * dt, self.command_vy * dt], dtype=float)
            self.x += float(delta_world[0])
            self.y += float(delta_world[1])

    def apply_amcl_soft_correction(self):
        # AMCL should normally publish map->odom. Fusing AMCL directly into odom can be invalid
        # if AMCL already uses odom, so this correction is disabled by default and only experimental.
        if not self.use_amcl_soft_correction or self.amcl_pose is None:
            return False

        amcl_x, amcl_y, amcl_yaw = self.amcl_pose
        jump = math.hypot(amcl_x - self.x, amcl_y - self.y)
        if jump > self.amcl_max_jump:
            self.warn_throttle('amcl_jump', f'Skipping AMCL soft correction jump {jump:.3f} m.')
            return False

        self.x += self.amcl_correction_alpha_xy * (amcl_x - self.x)
        self.y += self.amcl_correction_alpha_xy * (amcl_y - self.y)
        self.theta += self.amcl_correction_alpha_yaw * self.normalize_angle(amcl_yaw - self.theta)
        return True

    def update_contact_anchor_odometry(self, foot_positions_base, dt):
        contacts = list(self.measured_contacts)
        rotation = self.rotation_2d(self.theta)
        current_xy = np.array([self.x, self.y], dtype=float)

        candidates = []
        candidate_residuals = []
        foot_odom_positions = [None, None, None, None]
        slip_detected = False

        for leg_index, foot_base in enumerate(foot_positions_base):
            foot_base_xy = np.array([foot_base[0], foot_base[1]], dtype=float)
            foot_odom_xy = current_xy + rotation @ foot_base_xy
            foot_odom_positions[leg_index] = foot_odom_xy

            foot_vxy = 0.0
            if (
                dt > 1e-6
                and self.prev_foot_odom_positions[leg_index] is not None
            ):
                foot_delta = foot_odom_xy - self.prev_foot_odom_positions[leg_index]
                foot_vxy = float(np.linalg.norm(foot_delta) / dt)

            if contacts[leg_index] and not self.prev_contacts[leg_index]:
                self.foot_anchors_odom[leg_index] = foot_odom_xy.copy()
            elif not contacts[leg_index]:
                self.foot_anchors_odom[leg_index] = None

            if contacts[leg_index] and self.foot_anchors_odom[leg_index] is None:
                self.foot_anchors_odom[leg_index] = foot_odom_xy.copy()

            if contacts[leg_index] and self.foot_anchors_odom[leg_index] is not None:
                candidate_xy = self.foot_anchors_odom[leg_index] - rotation @ foot_base_xy
                residual = float(np.linalg.norm(candidate_xy - current_xy))
                foot_is_slipping = (
                    residual > self.contact_anchor_residual_threshold
                    or foot_vxy > self.slip_velocity_threshold
                )

                if foot_is_slipping:
                    slip_detected = True
                    if not self.leg_slipping[leg_index]:
                        self.slip_events_count += 1
                    self.leg_slipping[leg_index] = True
                else:
                    self.leg_slipping[leg_index] = False

                if residual <= self.contact_anchor_residual_threshold:
                    candidates.append(candidate_xy)
                    candidate_residuals.append(residual)
            else:
                self.leg_slipping[leg_index] = False

        stable_contact_count = len(candidates)
        if stable_contact_count >= self.min_stable_contacts:
            candidate_xy = np.mean(np.vstack(candidates), axis=0)
            alpha = float(np.clip(self.pose_lowpass_alpha, 0.0, 1.0))
            new_xy = alpha * current_xy + (1.0 - alpha) * candidate_xy
            self.x = float(new_xy[0])
            self.y = float(new_xy[1])
            self.last_good_pose_time = self.steady_clock.now()
        else:
            self.apply_no_contact_fallback(dt)

        self.prev_foot_odom_positions = foot_odom_positions
        return stable_contact_count, slip_detected, candidate_residuals

    def update_twist(self, prev_x, prev_y, prev_theta, dt):
        if dt <= 1e-6:
            return

        dx = self.x - prev_x
        dy = self.y - prev_y
        self.path_length += math.hypot(dx, dy)

        vx_world = dx / dt
        vy_world = dy / dt
        cos_yaw = math.cos(self.theta)
        sin_yaw = math.sin(self.theta)
        raw_vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        raw_vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
        raw_wz = self.normalize_angle(self.theta - prev_theta) / dt

        alpha = float(np.clip(self.twist_lowpass_alpha, 0.0, 1.0))
        self.twist_vx = alpha * raw_vx_body + (1.0 - alpha) * self.twist_vx
        self.twist_vy = alpha * raw_vy_body + (1.0 - alpha) * self.twist_vy
        self.twist_wz = alpha * raw_wz + (1.0 - alpha) * self.twist_wz

    def covariance_for_state(self, stable_contact_count, slip_detected):
        if stable_contact_count >= 2 and not slip_detected:
            xy_var = 0.02 ** 2
            yaw_var = 0.05 ** 2
            twist_var = 0.03 ** 2
        elif stable_contact_count >= 1 and not slip_detected:
            xy_var = 0.08 ** 2
            yaw_var = 0.08 ** 2
            twist_var = 0.08 ** 2
        else:
            xy_var = 0.50 ** 2
            yaw_var = 0.30 ** 2
            twist_var = 0.50 ** 2

        pose_covariance = [0.0] * 36
        twist_covariance = [0.0] * 36
        pose_covariance[0] = xy_var
        pose_covariance[7] = xy_var
        pose_covariance[14] = 999.0
        pose_covariance[21] = 999.0
        pose_covariance[28] = 999.0
        pose_covariance[35] = yaw_var
        twist_covariance[0] = twist_var
        twist_covariance[7] = twist_var
        twist_covariance[14] = 999.0
        twist_covariance[21] = 999.0
        twist_covariance[28] = 999.0
        twist_covariance[35] = max(twist_var, yaw_var)
        return pose_covariance, twist_covariance

    def build_odometry_msg(self, stamp, stable_contact_count, slip_detected):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = float(self.twist_vx)
        odom.twist.twist.linear.y = float(self.twist_vy)
        odom.twist.twist.angular.z = float(self.twist_wz)
        pose_covariance, twist_covariance = self.covariance_for_state(stable_contact_count, slip_detected)
        odom.pose.covariance = pose_covariance
        odom.twist.covariance = twist_covariance
        return odom

    def publish_tf(self, stamp):
        if not self.enable_odom_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(transform)

    def publish_path(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.odom_frame_id
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(self.theta)

        self.path_msg.header.stamp = stamp
        self.path_msg.header.frame_id = self.odom_frame_id
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.path_max_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_length:]
        self.path_pub.publish(self.path_msg)

    def publish_debug(self, used_amcl_correction, stable_contact_count, residuals):
        debug = {
            'x': float(self.x),
            'y': float(self.y),
            'theta': float(self.normalize_angle(self.theta)),
            'contacts_measured': list(self.measured_contacts),
            'contacts_expected': self.expected_contacts,
            'active_anchors': [anchor is not None for anchor in self.foot_anchors_odom],
            'stable_contact_count': int(stable_contact_count),
            'anchor_residuals': [float(residual) for residual in residuals],
            'slip_events_count': int(self.slip_events_count),
            'vx_body': float(self.twist_vx),
            'vy_body': float(self.twist_vy),
            'wz': float(self.twist_wz),
            'path_length': float(self.path_length),
            'command_vx': float(self.command_vx),
            'command_vy': float(self.command_vy),
            'command_wz': float(self.command_wz),
            'used_amcl_correction': bool(used_amcl_correction),
        }
        msg = String()
        msg.data = json.dumps(debug)
        self.debug_pub.publish(msg)

    def timer_callback(self):
        now = self.steady_clock.now()
        now_sec = now.nanoseconds * 1e-9
        if self.last_update_time is None:
            dt = 0.0
        else:
            dt = now_sec - self.last_update_time
            if dt < 0.0:
                dt = 0.0

        prev_x = self.x
        prev_y = self.y
        prev_theta = self.theta

        self.update_heading()
        foot_positions_base = self.calculate_foot_positions()

        stable_contact_count = 0
        slip_detected = False
        residuals = []
        if foot_positions_base is not None:
            stable_contact_count, slip_detected, residuals = self.update_contact_anchor_odometry(
                foot_positions_base,
                dt
            )
        else:
            self.apply_no_contact_fallback(dt)

        used_amcl_correction = self.apply_amcl_soft_correction()
        self.update_twist(prev_x, prev_y, prev_theta, dt)

        stamp = self.stamp_now()
        odom = self.build_odometry_msg(stamp, stable_contact_count, slip_detected)
        self.leg_odom_pub.publish(odom)
        if self.publish_legacy_odom and self.legacy_odom_pub is not None:
            self.legacy_odom_pub.publish(odom)
        if self.publish_filtered_odom and self.filtered_odom_pub is not None:
            self.filtered_odom_pub.publish(odom)
        self.publish_tf(stamp)
        self.publish_path(stamp)
        self.publish_debug(used_amcl_correction, stable_contact_count, residuals)

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta
        self.prev_contacts = list(self.measured_contacts)
        self.last_update_time = now_sec

        if self.verbose:
            self.get_logger().info(
                f'Leg odom: x={self.x:.4f}, y={self.y:.4f}, theta={self.theta:.4f}, '
                f'contacts={self.measured_contacts}, stable={stable_contact_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = DogOdometry()
    except Exception as exc:
        print(f'Exception during node initialization: {exc}')
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
