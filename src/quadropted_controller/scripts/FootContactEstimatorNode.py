#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf_transformations

from ForwardKinematics import robot_FK
from quadropted_msgs.msg import RobotFootContact


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


class FootContactEstimator(Node):
    def __init__(self):
        super().__init__('foot_contact_estimator')

        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('ground_truth_topic', '/model/robot1_my_bot/pose')
        self.declare_parameter('imu_topic', 'imu_plugin/out')
        self.declare_parameter('expected_contacts_topic', 'foot_contacts_expected')
        self.declare_parameter('preferred_child_frame_contains', 'robot1_my_bot')
        self.declare_parameter('contact_on_z_threshold', 0.035)
        self.declare_parameter('contact_off_z_threshold', 0.055)
        self.declare_parameter('contact_vxy_threshold', 0.08)
        self.declare_parameter('ground_z', 0.0)
        self.declare_parameter('publish_rate', 50)
        self.declare_parameter('verbose', False)

        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.expected_contacts_topic = self.get_parameter('expected_contacts_topic').value
        self.preferred_child_frame_contains = self.get_parameter('preferred_child_frame_contains').value
        self.contact_on_z_threshold = self.get_parameter('contact_on_z_threshold').value
        self.contact_off_z_threshold = self.get_parameter('contact_off_z_threshold').value
        self.contact_vxy_threshold = self.get_parameter('contact_vxy_threshold').value
        self.ground_z = self.get_parameter('ground_z').value
        publish_rate = self.get_parameter('publish_rate').value
        self.verbose = self.get_parameter('verbose').value

        body_dimensions = [0.3762, 0.0935]
        leg_dimensions = [0.0, 0.0955, 0.213, 0.213]
        self.fk_solver = robot_FK.ForwardKinematics(body_dimensions, leg_dimensions)
        self.steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)

        self.joint_positions = None
        self.base_world_position = None
        self.base_world_rotation = None
        self.expected_contacts = None
        self.current_contacts = [False, False, False, False]
        self.prev_foot_world = [None, None, None, None]
        self.prev_foot_time = None
        self.last_warn_times = {}
        self.last_imu = None

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

        self.create_subscription(JointState, self.joint_states_topic, self.joint_states_callback, sensor_qos)
        self.create_subscription(TFMessage, self.ground_truth_topic, self.ground_truth_callback, sensor_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)
        self.create_subscription(
            RobotFootContact,
            self.expected_contacts_topic,
            self.expected_contacts_callback,
            reliable_qos
        )

        self.contacts_pub = self.create_publisher(RobotFootContact, 'foot_contacts_measured', reliable_qos)
        self.debug_pub = self.create_publisher(String, 'foot_contacts_debug', reliable_qos)

        self.timer = self.create_timer(
            1.0 / float(publish_rate),
            self.timer_callback,
            clock=self.steady_clock
        )
        self.get_logger().info('Foot Contact Estimator Node has been started.')
        self.get_logger().info(
            'foot_contacts_measured are simulation contact estimates; '
            'in real robot this should be replaced by foot force/contact sensors or contact probability estimator.'
        )

    def warn_throttle(self, key, message, period_sec=2.0):
        now = time.monotonic()
        last = self.last_warn_times.get(key, 0.0)
        if now - last >= period_sec:
            self.get_logger().warn(message)
            self.last_warn_times[key] = now

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

    def ground_truth_callback(self, msg):
        if not msg.transforms:
            self.warn_throttle('empty_gt', 'Ground truth TFMessage has no transforms.')
            return

        selected = None
        for transform in msg.transforms:
            if self.preferred_child_frame_contains in transform.child_frame_id:
                selected = transform
                break

        if selected is None:
            selected = msg.transforms[0]
            self.warn_throttle(
                'gt_fallback',
                'Preferred ground truth child frame was not found; using first transform in TFMessage.'
            )

        translation = selected.transform.translation
        rotation = selected.transform.rotation
        self.base_world_position = np.array([translation.x, translation.y, translation.z], dtype=float)
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        self.base_world_rotation = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

    def imu_callback(self, msg):
        self.last_imu = msg

    def expected_contacts_callback(self, msg):
        if len(msg.contacts) == 4:
            self.expected_contacts = [bool(contact) for contact in msg.contacts]

    def publish_contacts(self, contacts, foot_world_z=None, foot_world_vxy=None, reason=None):
        contacts_msg = RobotFootContact()
        contacts_msg.contacts = [bool(contact) for contact in contacts]
        self.contacts_pub.publish(contacts_msg)

        debug_msg = String()
        debug_msg.data = json.dumps({
            'contacts': [bool(contact) for contact in contacts],
            'foot_world_z': foot_world_z if foot_world_z is not None else [None, None, None, None],
            'foot_world_vxy': foot_world_vxy if foot_world_vxy is not None else [None, None, None, None],
            'expected': self.expected_contacts,
            'reason': reason,
        })
        self.debug_pub.publish(debug_msg)

    def timer_callback(self):
        if self.joint_positions is None:
            self.warn_throttle('no_joints', 'No complete JointState has been received yet.')
            self.publish_contacts([False, False, False, False], reason='no_joint_states')
            return

        if self.base_world_position is None or self.base_world_rotation is None:
            self.warn_throttle('no_gt', 'No ground truth pose has been received yet.')
            self.publish_contacts([False, False, False, False], reason='no_ground_truth')
            return

        now = self.steady_clock.now()
        now_sec = now.nanoseconds * 1e-9
        dt = None
        if self.prev_foot_time is not None:
            dt = now_sec - self.prev_foot_time

        try:
            foot_positions_base = self.fk_solver.forward_kinematics_all_legs(self.joint_positions)
        except Exception as exc:
            self.warn_throttle('fk_error', f'Error in forward kinematics: {exc}')
            self.publish_contacts([False, False, False, False], reason='fk_error')
            return

        foot_world = []
        foot_world_z = []
        foot_world_vxy = []
        contacts = []

        for leg_index, foot_base in enumerate(foot_positions_base):
            foot_base_np = np.array(foot_base, dtype=float)
            foot_world_np = self.base_world_position + self.base_world_rotation @ foot_base_np
            foot_world.append(foot_world_np)
            foot_world_z.append(float(foot_world_np[2]))

            if dt is not None and dt > 1e-6 and self.prev_foot_world[leg_index] is not None:
                delta = foot_world_np - self.prev_foot_world[leg_index]
                vxy = math.hypot(delta[0], delta[1]) / dt
            else:
                vxy = 0.0
            foot_world_vxy.append(float(vxy))

            z_threshold = (
                self.contact_off_z_threshold
                if self.current_contacts[leg_index]
                else self.contact_on_z_threshold
            )
            in_contact = (
                foot_world_np[2] <= self.ground_z + z_threshold
                and vxy <= self.contact_vxy_threshold
            )
            contacts.append(bool(in_contact))

        self.current_contacts = contacts
        self.prev_foot_world = [foot.copy() for foot in foot_world]
        self.prev_foot_time = now_sec

        if self.verbose:
            self.get_logger().info(
                f'Measured contacts: {contacts}, foot_world_z={foot_world_z}, foot_world_vxy={foot_world_vxy}'
            )

        self.publish_contacts(contacts, foot_world_z=foot_world_z, foot_world_vxy=foot_world_vxy)


def main(args=None):
    rclpy.init(args=args)
    node = FootContactEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
