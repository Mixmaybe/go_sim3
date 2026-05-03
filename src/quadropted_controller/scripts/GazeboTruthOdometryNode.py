#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf_transformations


class GazeboTruthOdometry(Node):
    def __init__(self):
        super().__init__('gazebo_truth_odometry')

        self.declare_parameter('ground_truth_pose_topic', '/model/robot1_my_bot/pose')
        self.declare_parameter('model_frame_id', 'robot1_my_bot')
        self.declare_parameter('world_frame_id', 'city_second')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_footprint_frame_id', 'base_footprint')
        self.declare_parameter('base_link_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('zero_start', False)
        self.declare_parameter('publish_odom_topics', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('map_to_world_x', 0.0)
        self.declare_parameter('map_to_world_y', 0.0)
        self.declare_parameter('map_to_world_yaw', 0.0)
        self.declare_parameter('smooth_pose', False)
        self.declare_parameter('pose_lowpass_alpha', 0.0)

        self.ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        self.model_frame_id = self.get_parameter('model_frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_footprint_frame_id = self.get_parameter('base_footprint_frame_id').value
        self.base_link_frame_id = self.get_parameter('base_link_frame_id').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.zero_start = self.get_parameter('zero_start').value
        self.publish_odom_topics = self.get_parameter('publish_odom_topics').value
        self.publish_tf_enabled = self.get_parameter('publish_tf').value
        self.publish_debug_enabled = self.get_parameter('publish_debug').value
        self.map_to_world_x = float(self.get_parameter('map_to_world_x').value)
        self.map_to_world_y = float(self.get_parameter('map_to_world_y').value)
        self.map_to_world_yaw = float(self.get_parameter('map_to_world_yaw').value)
        self.smooth_pose = self.get_parameter('smooth_pose').value
        self.pose_lowpass_alpha = float(self.get_parameter('pose_lowpass_alpha').value)

        self.latest_world_pose = None
        self.origin_pose = None
        self.smoothed_nav_pose = None
        self.prev_nav_pose = None
        self.prev_publish_time = None
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

        self.odom_pub = None
        self.filtered_pub = None
        if self.publish_odom_topics:
            self.odom_pub = self.create_publisher(Odometry, 'odom', reliable_qos)
            self.filtered_pub = self.create_publisher(Odometry, 'odometry/filtered', reliable_qos)

        self.debug_pub = None
        if self.publish_debug_enabled:
            self.debug_pub = self.create_publisher(String, 'gazebo_truth_odom_debug', reliable_qos)

        self.tf_broadcaster = None
        if self.publish_tf_enabled:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(TFMessage, self.ground_truth_pose_topic, self.pose_callback, sensor_qos)

        self.steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0),
            self.timer_callback,
            clock=self.steady_clock
        )

        self.get_logger().info('Gazebo truth odometry node started.')
        self.get_logger().info(
            'Gazebo truth owns /odom, /odometry/filtered and odom->base_footprint only in demo truth mode.'
        )

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
    def quaternion_to_rpy(quaternion):
        return tf_transformations.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ])

    @staticmethod
    def rpy_to_quaternion(roll, pitch, yaw):
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        )

    def select_model_transform(self, msg):
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.world_frame_id
                and transform.child_frame_id == self.model_frame_id
            ):
                return transform

        self.warn_throttle(
            'missing_model_transform',
            f'Exact Gazebo model transform {self.world_frame_id}->{self.model_frame_id} not found.'
        )
        return None

    def pose_callback(self, msg):
        transform = self.select_model_transform(msg)
        if transform is None:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = self.quaternion_to_rpy(rotation)
        self.latest_world_pose = {
            'source_frame': transform.header.frame_id,
            'source_child': transform.child_frame_id,
            'x': float(translation.x),
            'y': float(translation.y),
            'z': float(translation.z),
            'roll': float(roll),
            'pitch': float(pitch),
            'yaw': float(yaw),
        }

    def nav_pose_from_world(self, world_pose):
        x_world = world_pose['x']
        y_world = world_pose['y']
        yaw_world = world_pose['yaw']

        if self.zero_start:
            if self.origin_pose is None:
                self.origin_pose = (x_world, y_world, yaw_world)

            origin_x, origin_y, origin_yaw = self.origin_pose
            dx = x_world - origin_x
            dy = y_world - origin_y
            cos_origin = math.cos(origin_yaw)
            sin_origin = math.sin(origin_yaw)
            nav_x = cos_origin * dx + sin_origin * dy
            nav_y = -sin_origin * dx + cos_origin * dy
            nav_yaw = self.normalize_angle(yaw_world - origin_yaw)
        else:
            nav_x = x_world + self.map_to_world_x
            nav_y = y_world + self.map_to_world_y
            nav_yaw = self.normalize_angle(yaw_world + self.map_to_world_yaw)

        if self.smooth_pose and self.pose_lowpass_alpha > 0.0 and self.smoothed_nav_pose is not None:
            alpha = min(max(self.pose_lowpass_alpha, 0.0), 0.99)
            prev_x, prev_y, prev_yaw = self.smoothed_nav_pose
            nav_x = alpha * prev_x + (1.0 - alpha) * nav_x
            nav_y = alpha * prev_y + (1.0 - alpha) * nav_y
            dyaw = self.normalize_angle(nav_yaw - prev_yaw)
            nav_yaw = self.normalize_angle(prev_yaw + (1.0 - alpha) * dyaw)

        self.smoothed_nav_pose = (nav_x, nav_y, nav_yaw)
        return nav_x, nav_y, nav_yaw

    def velocity_from_delta(self, nav_x, nav_y, nav_yaw, now_sec):
        vx_body = 0.0
        vy_body = 0.0
        wz = 0.0

        if self.prev_nav_pose is not None and self.prev_publish_time is not None:
            dt = now_sec - self.prev_publish_time
            if dt > 1e-6:
                prev_x, prev_y, prev_yaw = self.prev_nav_pose
                vx_world = (nav_x - prev_x) / dt
                vy_world = (nav_y - prev_y) / dt
                vx_body = math.cos(nav_yaw) * vx_world + math.sin(nav_yaw) * vy_world
                vy_body = -math.sin(nav_yaw) * vx_world + math.cos(nav_yaw) * vy_world
                wz = self.normalize_angle(nav_yaw - prev_yaw) / dt

        return vx_body, vy_body, wz

    def publish_odometry(self, stamp, nav_x, nav_y, nav_yaw, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_footprint_frame_id
        odom.pose.pose.position.x = float(nav_x)
        odom.pose.pose.position.y = float(nav_y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.rpy_to_quaternion(0.0, 0.0, nav_yaw)
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(wz)

        if self.odom_pub is not None and self.filtered_pub is not None:
            self.odom_pub.publish(odom)
            self.filtered_pub.publish(odom)

    def publish_tf(self, stamp, nav_x, nav_y, nav_yaw, world_pose):
        if self.tf_broadcaster is None:
            return

        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = stamp
        odom_to_base_footprint.header.frame_id = self.odom_frame_id
        odom_to_base_footprint.child_frame_id = self.base_footprint_frame_id
        odom_to_base_footprint.transform.translation.x = float(nav_x)
        odom_to_base_footprint.transform.translation.y = float(nav_y)
        odom_to_base_footprint.transform.translation.z = 0.0
        odom_to_base_footprint.transform.rotation = self.rpy_to_quaternion(0.0, 0.0, nav_yaw)

        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = stamp
        base_footprint_to_base_link.header.frame_id = self.base_footprint_frame_id
        base_footprint_to_base_link.child_frame_id = self.base_link_frame_id
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = float(world_pose['z'])
        base_footprint_to_base_link.transform.rotation = self.rpy_to_quaternion(
            world_pose['roll'],
            world_pose['pitch'],
            0.0
        )

        self.tf_broadcaster.sendTransform([
            odom_to_base_footprint,
            base_footprint_to_base_link,
        ])

    def publish_debug(self, world_pose, nav_x, nav_y, nav_yaw, vx, vy, wz):
        if self.debug_pub is None:
            return

        msg = String()
        msg.data = json.dumps({
            'source_frame': world_pose['source_frame'],
            'source_child': world_pose['source_child'],
            'world_x': world_pose['x'],
            'world_y': world_pose['y'],
            'world_z': world_pose['z'],
            'world_roll': world_pose['roll'],
            'world_pitch': world_pose['pitch'],
            'world_yaw': world_pose['yaw'],
            'nav_x': float(nav_x),
            'nav_y': float(nav_y),
            'nav_yaw': float(nav_yaw),
            'vx': float(vx),
            'vy': float(vy),
            'wz': float(wz),
        })
        self.debug_pub.publish(msg)

    def timer_callback(self):
        if self.latest_world_pose is None:
            self.warn_throttle('no_pose', 'No exact Gazebo model pose has been received yet.')
            return

        now = self.steady_clock.now()
        now_sec = now.nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()

        world_pose = self.latest_world_pose
        nav_x, nav_y, nav_yaw = self.nav_pose_from_world(world_pose)
        vx, vy, wz = self.velocity_from_delta(nav_x, nav_y, nav_yaw, now_sec)

        self.publish_odometry(stamp, nav_x, nav_y, nav_yaw, vx, vy, wz)
        self.publish_tf(stamp, nav_x, nav_y, nav_yaw, world_pose)
        self.publish_debug(world_pose, nav_x, nav_y, nav_yaw, vx, vy, wz)

        self.prev_nav_pose = (float(nav_x), float(nav_y), float(nav_yaw))
        self.prev_publish_time = now_sec


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTruthOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
