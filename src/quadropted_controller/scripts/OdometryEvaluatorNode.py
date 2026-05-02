#!/usr/bin/env python3
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf_transformations


class OdometryEvaluator(Node):
    def __init__(self):
        super().__init__('odometry_evaluator')

        self.declare_parameter('estimated_odom_topic', 'leg_odom_raw')
        self.declare_parameter('filtered_odom_topic', 'odometry/filtered')
        self.declare_parameter('legacy_odom_topic', 'odom')
        self.declare_parameter('ground_truth_topic', 'ground_truth/odom')
        self.declare_parameter('amcl_pose_topic', 'amcl_pose')
        self.declare_parameter('goal_pose_topic', 'goal_pose')
        self.declare_parameter('leg_odom_debug_topic', 'leg_odom_debug')
        self.declare_parameter('imu_topic', 'imu_plugin/out')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('rpe_interval_sec', 1.0)
        self.declare_parameter('goal_tolerance_xy', 0.35)
        self.declare_parameter('fall_z_threshold', 0.18)
        self.declare_parameter('csv_path', '/tmp/go_sim3_odometry_metrics_robot1.csv')
        self.declare_parameter('verbose', False)

        self.estimated_odom_topic = self.get_parameter('estimated_odom_topic').value
        self.filtered_odom_topic = self.get_parameter('filtered_odom_topic').value
        self.legacy_odom_topic = self.get_parameter('legacy_odom_topic').value
        self.ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.leg_odom_debug_topic = self.get_parameter('leg_odom_debug_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.rpe_interval_sec = self.get_parameter('rpe_interval_sec').value
        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').value
        self.fall_z_threshold = self.get_parameter('fall_z_threshold').value
        self.csv_path = self.get_parameter('csv_path').value
        self.verbose = self.get_parameter('verbose').value

        self.sources = {
            'estimated': self.make_source_state(),
            'filtered': self.make_source_state(),
            'legacy': self.make_source_state(),
        }
        self.ground_truth_pose = None
        self.prev_ground_truth_pose = None
        self.ground_truth_path_length = 0.0
        self.amcl_pose = None
        self.goal_pose = None
        self.goal_received_time = None
        self.delivery_success = False
        self.time_to_goal = None
        self.start_time = None
        self.movement_start_time = None
        self.slip_events_count = 0
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.fall_count = 0
        self.currently_fallen = False
        self.last_warn_times = {}

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
            history=HistoryPolicy.KEEP_LAST
        )

        self.metrics_pub = self.create_publisher(String, 'odometry_metrics', reliable_qos)
        self.csv_path_pub = self.create_publisher(String, 'odometry_metrics_csv_path', reliable_qos)
        self.create_subscription(Odometry, self.estimated_odom_topic, self.estimated_callback, reliable_qos)
        self.create_subscription(Odometry, self.filtered_odom_topic, self.filtered_callback, reliable_qos)
        self.create_subscription(Odometry, self.legacy_odom_topic, self.legacy_callback, reliable_qos)
        self.create_subscription(Odometry, self.ground_truth_topic, self.ground_truth_callback, reliable_qos)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_pose_callback, reliable_qos)
        self.create_subscription(PoseStamped, self.goal_pose_topic, self.goal_pose_callback, reliable_qos)
        self.create_subscription(String, self.leg_odom_debug_topic, self.leg_debug_callback, reliable_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, sensor_qos)

        self.ensure_csv_header()
        self.timer = self.create_timer(1.0 / float(publish_rate), self.timer_callback)
        self.get_logger().info('Odometry Evaluator Node has been started.')

    @staticmethod
    def make_source_state():
        return {
            'pose': None,
            'prev_pose': None,
            'path_length': 0.0,
            'alignment': None,
            'squared_errors': [],
            'yaw_errors': [],
            'history': [],
        }

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
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def message_time(self, header):
        stamp_sec = self.stamp_to_sec(header.stamp)
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        return stamp_sec

    @staticmethod
    def yaw_from_orientation(orientation):
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        return yaw

    def pose_from_odom(self, msg):
        position = msg.pose.pose.position
        yaw = self.yaw_from_orientation(msg.pose.pose.orientation)
        return (
            float(position.x),
            float(position.y),
            float(position.z),
            float(yaw),
            self.message_time(msg.header),
        )

    def update_source_pose(self, name, msg):
        source = self.sources[name]
        pose = self.pose_from_odom(msg)
        if source['pose'] is not None:
            dx = pose[0] - source['pose'][0]
            dy = pose[1] - source['pose'][1]
            source['path_length'] += math.hypot(dx, dy)
        source['prev_pose'] = source['pose']
        source['pose'] = pose

    def estimated_callback(self, msg):
        self.update_source_pose('estimated', msg)

    def filtered_callback(self, msg):
        self.update_source_pose('filtered', msg)

    def legacy_callback(self, msg):
        self.update_source_pose('legacy', msg)

    def ground_truth_callback(self, msg):
        pose = self.pose_from_odom(msg)
        if self.ground_truth_pose is not None:
            dx = pose[0] - self.ground_truth_pose[0]
            dy = pose[1] - self.ground_truth_pose[1]
            step = math.hypot(dx, dy)
            self.ground_truth_path_length += step
            if self.movement_start_time is None and self.ground_truth_path_length > 0.02:
                self.movement_start_time = pose[4]
        self.prev_ground_truth_pose = self.ground_truth_pose
        self.ground_truth_pose = pose
        if self.start_time is None:
            self.start_time = pose[4]

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        yaw = self.yaw_from_orientation(msg.pose.pose.orientation)
        self.amcl_pose = (float(position.x), float(position.y), float(position.z), float(yaw), self.message_time(msg.header))

    def goal_pose_callback(self, msg):
        position = msg.pose.position
        yaw = self.yaw_from_orientation(msg.pose.orientation)
        self.goal_pose = (float(position.x), float(position.y), float(position.z), float(yaw))
        self.goal_received_time = self.message_time(msg.header)
        if self.start_time is None:
            self.start_time = self.goal_received_time

    def leg_debug_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.slip_events_count = int(data.get('slip_events_count', self.slip_events_count))

    def imu_callback(self, msg):
        roll, pitch, _ = tf_transformations.euler_from_quaternion([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ])
        self.imu_roll = float(roll)
        self.imu_pitch = float(pitch)

    def ensure_csv_header(self):
        header = [
            'stamp',
            'elapsed_time',
            'ate',
            'rpe_1s',
            'drift_rate',
            'final_position_error',
            'yaw_error_current',
            'yaw_error_mean',
            'slip_events_count',
            'delivery_success',
            'time_to_goal',
            'fall_count',
            'mean_speed',
            'path_length_gt',
            'path_length_est',
            'amcl_error',
        ]
        try:
            needs_header = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
            if needs_header:
                with open(self.csv_path, 'w', newline='') as csv_file:
                    csv.writer(csv_file).writerow(header)
        except OSError as exc:
            self.warn_throttle('csv_header', f'Could not initialize metrics CSV: {exc}')

    def append_csv_row(self, metrics):
        keys = [
            'stamp',
            'elapsed_time',
            'ate',
            'rpe_1s',
            'drift_rate',
            'final_position_error',
            'yaw_error_current',
            'yaw_error_mean',
            'slip_events_count',
            'delivery_success',
            'time_to_goal',
            'fall_count',
            'mean_speed',
            'path_length_gt',
            'path_length_est',
            'amcl_error',
        ]
        try:
            with open(self.csv_path, 'a', newline='') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([metrics.get(key, '') if metrics.get(key) is not None else '' for key in keys])
        except OSError as exc:
            self.warn_throttle('csv_append', f'Could not append metrics CSV: {exc}')

    def aligned_ground_truth_for(self, source_name):
        source = self.sources[source_name]
        if source['pose'] is None or self.ground_truth_pose is None:
            return None

        if source['alignment'] is None:
            source['alignment'] = {
                'dx': source['pose'][0] - self.ground_truth_pose[0],
                'dy': source['pose'][1] - self.ground_truth_pose[1],
                'dyaw': self.normalize_angle(source['pose'][3] - self.ground_truth_pose[3]),
            }

        alignment = source['alignment']
        return (
            self.ground_truth_pose[0] + alignment['dx'],
            self.ground_truth_pose[1] + alignment['dy'],
            self.normalize_angle(self.ground_truth_pose[3] + alignment['dyaw']),
        )

    def source_position_error(self, source_name, update_history=False, elapsed_time=0.0):
        source = self.sources[source_name]
        aligned_gt = self.aligned_ground_truth_for(source_name)
        if source['pose'] is None or aligned_gt is None:
            return None, None

        error = math.hypot(source['pose'][0] - aligned_gt[0], source['pose'][1] - aligned_gt[1])
        yaw_error = abs(self.normalize_angle(source['pose'][3] - aligned_gt[2]))

        if update_history:
            source['squared_errors'].append(error ** 2)
            source['yaw_errors'].append(yaw_error)
            source['history'].append((
                elapsed_time,
                source['pose'][0],
                source['pose'][1],
                aligned_gt[0],
                aligned_gt[1],
            ))
            while len(source['history']) > 1000:
                source['history'].pop(0)

        return error, yaw_error

    def rpe_for_source(self, source_name, elapsed_time):
        source = self.sources[source_name]
        if len(source['history']) < 2:
            return None

        target_time = elapsed_time - self.rpe_interval_sec
        older = None
        for sample in reversed(source['history']):
            if sample[0] <= target_time:
                older = sample
                break
        if older is None:
            return None

        current = source['history'][-1]
        est_dx = current[1] - older[1]
        est_dy = current[2] - older[2]
        gt_dx = current[3] - older[3]
        gt_dy = current[4] - older[4]
        return math.hypot(est_dx - gt_dx, est_dy - gt_dy)

    def update_delivery_success(self, elapsed_time):
        if self.delivery_success or self.goal_pose is None or self.ground_truth_pose is None:
            return

        distance_to_goal = math.hypot(
            self.ground_truth_pose[0] - self.goal_pose[0],
            self.ground_truth_pose[1] - self.goal_pose[1]
        )
        if distance_to_goal <= self.goal_tolerance_xy:
            self.delivery_success = True
            start = self.goal_received_time
            if start is None:
                start = self.movement_start_time if self.movement_start_time is not None else self.start_time
            self.time_to_goal = elapsed_time if start is None else max(0.0, self.ground_truth_pose[4] - start)

    def update_fall_count(self):
        fallen = False
        if abs(self.imu_roll) > 0.8 or abs(self.imu_pitch) > 0.8:
            fallen = True
        if self.ground_truth_pose is not None and self.ground_truth_pose[2] < self.fall_z_threshold:
            fallen = True

        if fallen and not self.currently_fallen:
            self.fall_count += 1
        self.currently_fallen = fallen

    def timer_callback(self):
        if self.ground_truth_pose is None or self.sources['estimated']['pose'] is None:
            self.warn_throttle('waiting', 'Waiting for estimated odometry and ground truth odometry.')
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = now_sec
        elapsed_time = max(0.0, now_sec - self.start_time)

        final_error, yaw_error = self.source_position_error('estimated', update_history=True, elapsed_time=elapsed_time)
        legacy_error, _ = self.source_position_error('legacy')
        filtered_error, _ = self.source_position_error('filtered')

        source = self.sources['estimated']
        ate = None
        if source['squared_errors']:
            ate = math.sqrt(sum(source['squared_errors']) / len(source['squared_errors']))

        yaw_error_mean = None
        if source['yaw_errors']:
            yaw_error_mean = sum(source['yaw_errors']) / len(source['yaw_errors'])

        rpe_1s = self.rpe_for_source('estimated', elapsed_time)
        drift_rate = None
        if final_error is not None:
            drift_rate = final_error / max(self.ground_truth_path_length, 1e-6)

        amcl_error = None
        if self.amcl_pose is not None and self.ground_truth_pose is not None:
            amcl_error = math.hypot(
                self.amcl_pose[0] - self.ground_truth_pose[0],
                self.amcl_pose[1] - self.ground_truth_pose[1]
            )

        self.update_delivery_success(elapsed_time)
        self.update_fall_count()
        mean_speed = self.ground_truth_path_length / max(elapsed_time, 1e-6)

        metrics = {
            'stamp': float(now_sec),
            'elapsed_time': float(elapsed_time),
            'ate': ate,
            'rpe_1s': rpe_1s,
            'drift_rate': drift_rate,
            'final_position_error': final_error,
            'yaw_error_current': yaw_error,
            'yaw_error_mean': yaw_error_mean,
            'slip_events_count': int(self.slip_events_count),
            'delivery_success': bool(self.delivery_success),
            'time_to_goal': self.time_to_goal,
            'fall_count': int(self.fall_count),
            'mean_speed': float(mean_speed),
            'path_length_gt': float(self.ground_truth_path_length),
            'path_length_est': float(source['path_length']),
            'amcl_error': amcl_error,
            'legacy_position_error': legacy_error,
            'filtered_position_error': filtered_error,
        }

        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)

        csv_msg = String()
        csv_msg.data = self.csv_path
        self.csv_path_pub.publish(csv_msg)
        self.append_csv_row(metrics)

        if self.verbose:
            self.get_logger().info(f'Odometry metrics: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
