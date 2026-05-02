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
        self.declare_parameter('model_frame_id', 'robot1_my_bot')
        self.declare_parameter('world_frame_id', 'city_second')
        self.declare_parameter('allow_fallback_first_transform', False)
        self.declare_parameter('ground_truth_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'ground_truth_base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('publish_nav_odom', False)
        self.declare_parameter('publish_nav_tf', False)
        self.declare_parameter('nav_odom_topic', 'odom')
        self.declare_parameter('nav_filtered_topic', 'odometry/filtered')
        self.declare_parameter('nav_odom_frame_id', 'odom')
        self.declare_parameter('nav_base_frame_id', 'base_link')
        self.declare_parameter('zero_start', True)
        self.declare_parameter('path_max_length', 5000)
        self.declare_parameter('verbose', False)

        self.ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.model_frame_id = self.get_parameter('model_frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.allow_fallback_first_transform = self.get_parameter('allow_fallback_first_transform').value
        self.ground_truth_frame_id = self.get_parameter('ground_truth_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf_enabled = self.get_parameter('publish_tf').value
        self.publish_nav_odom = self.get_parameter('publish_nav_odom').value
        self.publish_nav_tf = self.get_parameter('publish_nav_tf').value
        self.nav_odom_topic = self.get_parameter('nav_odom_topic').value
        self.nav_filtered_topic = self.get_parameter('nav_filtered_topic').value
        self.nav_odom_frame_id = self.get_parameter('nav_odom_frame_id').value
        self.nav_base_frame_id = self.get_parameter('nav_base_frame_id').value
        self.zero_start = self.get_parameter('zero_start').value
        self.path_max_length = self.get_parameter('path_max_length').value
        self.verbose = self.get_parameter('verbose').value

        self.prev_pose = None
        self.prev_time = None
        self.prev_nav_pose = None
        self.nav_origin = None
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
        self.nav_odom_pub = None
        self.nav_filtered_pub = None
        if self.publish_nav_odom:
            self.nav_odom_pub = self.create_publisher(Odometry, self.nav_odom_topic, reliable_qos)
            self.nav_filtered_pub = self.create_publisher(Odometry, self.nav_filtered_topic, reliable_qos)
        self.path_pub = self.create_publisher(Path, 'ground_truth/path', reliable_qos)
        self.debug_pub = self.create_publisher(String, 'ground_truth/debug', reliable_qos)
        self.create_subscription(TFMessage, self.ground_truth_pose_topic, self.ground_truth_callback, sensor_qos)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.ground_truth_frame_id

        if self.publish_tf_enabled or self.publish_nav_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Ground Truth Odometry Node has been started.')
        self.get_logger().info('Ground truth is only for evaluation/debug, not for production odometry.')
        if self.publish_nav_odom or self.publish_nav_tf:
            self.get_logger().info('Publishing Gazebo ground truth as simulation nav odometry.')

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

    @staticmethod
    def yaw_to_quaternion(yaw):
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )

    def select_transform(self, msg):
        if not msg.transforms:
            self.warn_throttle('empty_gt', 'Ground truth TFMessage has no transforms.')
            return None

        for transform in msg.transforms:
            if transform.child_frame_id != self.model_frame_id:
                continue

            if transform.header.frame_id != self.world_frame_id:
                self.warn_throttle(
                    'gt_world_mismatch',
                    'Ground truth model transform has frame_id '
                    f'"{transform.header.frame_id}", expected "{self.world_frame_id}".'
                )
            return transform

        if self.allow_fallback_first_transform:
            self.warn_throttle(
                'gt_fallback',
                f'Model transform child_frame_id == {self.model_frame_id} not found; '
                'using first transform in TFMessage because fallback is enabled.'
            )
            return msg.transforms[0]

        self.warn_throttle(
            'gt_missing_model',
            f'Model transform child_frame_id == {self.model_frame_id} not found.'
        )
        return None

    def nav_pose_from_ground_truth(self, x, y, z, yaw):
        if self.nav_origin is None:
            self.nav_origin = (float(x), float(y), float(z), float(yaw))

        if not self.zero_start:
            return float(x), float(y), float(z), float(yaw)

        origin_x, origin_y, origin_z, origin_yaw = self.nav_origin
        dx = float(x) - origin_x
        dy = float(y) - origin_y
        cos_origin = math.cos(origin_yaw)
        sin_origin = math.sin(origin_yaw)
        nav_x = cos_origin * dx + sin_origin * dy
        nav_y = -sin_origin * dx + cos_origin * dy
        nav_z = float(z) - origin_z
        nav_yaw = self.normalize_angle(float(yaw) - origin_yaw)
        return nav_x, nav_y, nav_z, nav_yaw

    def publish_nav_odometry(self, stamp, x, y, z, yaw, now_sec):
        nav_x, nav_y, nav_z, nav_yaw = self.nav_pose_from_ground_truth(x, y, z, yaw)

        vx = 0.0
        vy = 0.0
        vz = 0.0
        wz = 0.0
        if self.prev_nav_pose is not None and self.prev_time is not None:
            dt = now_sec - self.prev_time
            if dt > 1e-6:
                vx = (nav_x - self.prev_nav_pose[0]) / dt
                vy = (nav_y - self.prev_nav_pose[1]) / dt
                vz = (nav_z - self.prev_nav_pose[2]) / dt
                wz = self.normalize_angle(nav_yaw - self.prev_nav_pose[3]) / dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.nav_odom_frame_id
        odom.child_frame_id = self.nav_base_frame_id
        odom.pose.pose.position.x = float(nav_x)
        odom.pose.pose.position.y = float(nav_y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(nav_yaw)
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)
        odom.twist.twist.angular.z = float(wz)

        if self.publish_nav_odom and self.nav_odom_pub is not None and self.nav_filtered_pub is not None:
            self.nav_odom_pub.publish(odom)
            self.nav_filtered_pub.publish(odom)

        if self.publish_nav_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.nav_odom_frame_id
            tf_msg.child_frame_id = self.nav_base_frame_id
            tf_msg.transform.translation.x = float(nav_x)
            tf_msg.transform.translation.y = float(nav_y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = self.yaw_to_quaternion(nav_yaw)
            self.tf_broadcaster.sendTransform(tf_msg)

        self.prev_nav_pose = (float(nav_x), float(nav_y), float(nav_z), float(nav_yaw))

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
            'source_frame_id': transform.header.frame_id,
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

        self.publish_nav_odometry(
            stamp,
            translation.x,
            translation.y,
            translation.z,
            yaw,
            now_sec
        )

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
