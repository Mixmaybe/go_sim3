#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf_transformations


class GroundTruthOdometry(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry')

        self.declare_parameter('ground_truth_pose_topic', '/model/robot1_my_bot/pose')
        self.declare_parameter('preferred_child_frame_contains', 'robot1_my_bot')
        self.declare_parameter('ground_truth_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'ground_truth_base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('path_max_length', 5000)
        self.declare_parameter('verbose', False)

        self.ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.preferred_child_frame_contains = self.get_parameter('preferred_child_frame_contains').value
        self.ground_truth_frame_id = self.get_parameter('ground_truth_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf_enabled = self.get_parameter('publish_tf').value
        self.path_max_length = self.get_parameter('path_max_length').value
        self.verbose = self.get_parameter('verbose').value

        self.prev_pose = None
        self.prev_time = None
        self.path_length = 0.0
        self.last_warn_times = {}

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

        self.odom_pub = self.create_publisher(Odometry, 'ground_truth/odom', reliable_qos)
        self.path_pub = self.create_publisher(Path, 'ground_truth/path', reliable_qos)
        self.debug_pub = self.create_publisher(String, 'ground_truth/debug', reliable_qos)
        self.create_subscription(TFMessage, self.ground_truth_pose_topic, self.ground_truth_callback, sensor_qos)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.ground_truth_frame_id

        if self.publish_tf_enabled:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Ground Truth Odometry Node has been started.')
        self.get_logger().info('Ground truth is only for evaluation/debug, not for production odometry.')

    def warn_throttle(self, key, message, period_sec=2.0):
        now = time.monotonic()
        last = self.last_warn_times.get(key, 0.0)
        if now - last >= period_sec:
            self.get_logger().warn(message)
            self.last_warn_times[key] = now

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(quaternion):
        _, _, yaw = tf_transformations.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ])
        return yaw

    def select_transform(self, msg):
        if not msg.transforms:
            self.warn_throttle('empty_gt', 'Ground truth TFMessage has no transforms.')
            return None

        for transform in msg.transforms:
            if self.preferred_child_frame_contains in transform.child_frame_id:
                return transform

        self.warn_throttle(
            'gt_fallback',
            'Preferred ground truth child frame was not found; using first transform in TFMessage.'
        )
        return msg.transforms[0]

    def ground_truth_callback(self, msg):
        transform = self.select_transform(msg)
        if transform is None:
            return

        stamp = transform.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        now_sec = self.stamp_to_sec(stamp)

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self.yaw_from_quaternion(rotation)

        vx = 0.0
        vy = 0.0
        vz = 0.0
        wz = 0.0
        if self.prev_pose is not None and self.prev_time is not None:
            dt = now_sec - self.prev_time
            if dt > 1e-6:
                dx = translation.x - self.prev_pose[0]
                dy = translation.y - self.prev_pose[1]
                dz = translation.z - self.prev_pose[2]
                vx = dx / dt
                vy = dy / dt
                vz = dz / dt
                wz = self.normalize_angle(yaw - self.prev_pose[3]) / dt
                self.path_length += math.hypot(dx, dy)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.ground_truth_frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(translation.x)
        odom.pose.pose.position.y = float(translation.y)
        odom.pose.pose.position.z = float(translation.z)
        odom.pose.pose.orientation = Quaternion(
            x=float(rotation.x),
            y=float(rotation.y),
            z=float(rotation.z),
            w=float(rotation.w)
        )
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)
        odom.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.path_max_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_length:]
        self.path_pub.publish(self.path_msg)

        debug = String()
        debug.data = json.dumps({
            'x': float(translation.x),
            'y': float(translation.y),
            'z': float(translation.z),
            'yaw': float(yaw),
            'vx': float(vx),
            'vy': float(vy),
            'wz': float(wz),
            'path_length': float(self.path_length),
            'source_child_frame_id': transform.child_frame_id,
        })
        self.debug_pub.publish(debug)

        if self.publish_tf_enabled:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.ground_truth_frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform = transform.transform
            self.tf_broadcaster.sendTransform(tf_msg)

        self.prev_pose = (float(translation.x), float(translation.y), float(translation.z), float(yaw))
        self.prev_time = now_sec

        if self.verbose:
            self.get_logger().info(
                f'GT odom: x={translation.x:.4f}, y={translation.y:.4f}, yaw={yaw:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
