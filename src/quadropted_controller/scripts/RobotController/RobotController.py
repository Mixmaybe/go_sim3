#!/usr/bin/env python3
# Author: lnotspotl, abutalipovvv
import numpy as np
import tf_transformations  # Используем tf2 вместо tf
from .StateCommand import State, Command, BehaviorState
from .RestController import RestController
from .TrotGaitController import TrotGaitController
from .CrawlGaitController import CrawlGaitController
from .StandController import StandController
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand, RobotVelocity
from quadropted_msgs.srv import RobotBehaviorCommand
class Robot:
    def __init__(self, node, body, legs, imu, robot_id):
        self.body = body
        self.legs = legs
        self.node = node  
        self.robot_id = robot_id  

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.52 + self.legs[1]
        self.x_shift_front = 0.02
        self.x_shift_back = -0.0
        self.default_height = 0.30

        self.trotGaitController = TrotGaitController(
            self.node, self.default_stance,
            stance_time=0.04, swing_time=0.14, time_step=0.02,
            use_imu=imu
        )

        self.crawlGaitController = CrawlGaitController(
            self.node, self.default_stance,
            stance_time=0.55, swing_time=0.45, time_step=0.02
        )

        self.standController = StandController(self.node, self.default_stance)  # Передаем node
        self.restController = RestController(self.default_stance)
        self.currentController = self.restController

        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)
        self.control_dt = 1.0 / 60.0

        # Filter targets for smoother transitions and locomotion commands.
        self.command_velocity_target = np.zeros(3)
        self.command_yaw_rate_target = np.zeros(3)
        self.body_z_target = 0.0

        self.command_accel_limits = np.array([0.10, 0.05, 0.10])   # [m/s^2]
        self.command_brake_limits = np.array([0.10, 0.05, 0.10])   # [m/s^2]
        self.yaw_accel_limits = np.array([0.50, 0.50, 1.20])       # [rad/s^2]
        self.yaw_brake_limits = np.array([0.80, 0.80, 1.80])       # [rad/s^2]
        self.body_z_rise_rate = 0.6                               # [m/s]
        self.body_z_fall_rate = 0.9                               # [m/s]
        self.velocity_deadband = 1e-4
        self.yaw_deadband = 1e-4

                # Установить режим TROT по умолчанию
        self.command.trot_event = True
        self.command.rest_event = False
        self.command.crawl_event = False
        self.command.stand_event = False

        # Переключиться на TROT
        self.change_controller()



        
        self.node.create_subscription(RobotModeCommand, 'robot_mode', self.mode_callback, 10)
        self.node.create_subscription(RobotVelocity, 'robot_velocity', self.velocity_callback, 10)

        self.behavior_service = self.node.create_service(
            RobotBehaviorCommand,
            'robot_behavior_command',
            self.handle_behavior_command
        )

    def mode_callback(self, msg):
        
        if msg.robot_id == self.robot_id:
            if self.node.verbose:
                self.node.get_logger().info(f"Received mode command: {msg.mode} for robot_id: {msg.robot_id}")
            if msg.mode == "REST":
                self.set_motion_targets_zero()
                self.body_z_target = 0.0
                self.command.rest_event = True
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "TROT":
                self.body_z_target = 0.0
                self.command.rest_event = False
                self.command.trot_event = True
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "CRAWL":
                self.body_z_target = 0.0
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = True
                self.command.stand_event = False
            elif msg.mode == "STAND":
                self.set_motion_targets_zero()
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = True
            
            self.change_controller()

    def velocity_callback(self, msg):
        
        if msg.robot_id == self.robot_id:
            self.command_velocity_target = np.array([
                msg.cmd_vel.linear.x,
                msg.cmd_vel.linear.y,
                msg.cmd_vel.linear.z
            ])  #  [x, y, z]
            
            self.command_yaw_rate_target = np.array([
                msg.cmd_vel.angular.x,
                msg.cmd_vel.angular.y,
                msg.cmd_vel.angular.z
            ])  # numpy  [roll, pitch, yaw]
            
            if self.node.verbose:
                self.node.get_logger().info(
                    f"Velocity target updated: linear={self.command_velocity_target}, angular={self.command_yaw_rate_target}"
                )

    def handle_behavior_command(self, request, response):
        command = request.command.lower()
        self.node.get_logger().info(f"Получена команда поведения: {command}")

        if command == 'sit':
            self.set_motion_targets_zero()
            self.command.stand_event = True
            self.command.rest_event = False
            self.command.trot_event = False
            self.command.crawl_event = False

            self.body_z_target = -0.23
            self.change_controller()

            response.success = True
            response.message = "Робот сел."
        
        elif command == 'up':
            self.set_motion_targets_zero()
            self.command.rest_event = True
            self.command.stand_event = False
            self.command.trot_event = False
            self.command.crawl_event = False

            self.body_z_target = 0.0
            self.change_controller()

            response.success = True
            response.message = "Робот встал."
        
        elif command == 'walk':
            self.body_z_target = 0.0
            self.command.rest_event = False
            self.command.trot_event = True
            self.command.stand_event = False
            self.command.crawl_event = False

            self.change_controller()

            response.success = True
            response.message = "Робот начал ходить."
        
        else:
            response.success = False
            response.message = f"Неизвестная команда: {command}"

        return response

    def change_controller(self):
        if self.command.trot_event:
            if self.state.behavior_state != BehaviorState.TROT:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.currentController.trotNeeded = True
                self.state.ticks = 0
            self.command.trot_event = False
            self.command.rest_event = False
            self.node.get_logger().info("Переключено на TROT контроллер")
        
        elif self.command.crawl_event:
            if self.state.behavior_state != BehaviorState.CRAWL:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False
            self.node.get_logger().info("Переключено на CRAWL контроллер")
        
        elif self.command.stand_event:
            if self.state.behavior_state != BehaviorState.STAND:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
                self.node.get_logger().info("Переключено на STAND контроллер")
            self.command.stand_event = False
        
        elif self.command.rest_event:
            if self.state.behavior_state != BehaviorState.REST:
                self.state.behavior_state = BehaviorState.REST
                self.currentController = self.restController
                self.currentController.pid_controller.reset()
            self.command.rest_event = False
            self.node.get_logger().info("Переключено на REST контроллер")

    def set_motion_targets_zero(self):
        self.command_velocity_target = np.zeros(3)
        self.command_yaw_rate_target = np.zeros(3)

    def slew_towards(self, current, target, accel_limit, brake_limit):
        delta = target - current
        if abs(delta) < 1e-9:
            return target

        reversing = abs(target) < abs(current) or (abs(current) > 1e-9 and np.sign(current) != np.sign(target))
        rate_limit = brake_limit if reversing else accel_limit
        max_delta = rate_limit * self.control_dt
        return current + np.clip(delta, -max_delta, max_delta)

    def update_motion_profile(self):
        for idx in range(3):
            self.command.velocity[idx] = self.slew_towards(
                self.command.velocity[idx],
                self.command_velocity_target[idx],
                self.command_accel_limits[idx],
                self.command_brake_limits[idx]
            )
            self.command.yaw_rate[idx] = self.slew_towards(
                self.command.yaw_rate[idx],
                self.command_yaw_rate_target[idx],
                self.yaw_accel_limits[idx],
                self.yaw_brake_limits[idx]
            )

        body_z_delta = self.body_z_target - self.state.body_local_position[2]
        if abs(body_z_delta) < 1e-9:
            self.state.body_local_position[2] = self.body_z_target
        else:
            body_z_rate = self.body_z_rise_rate if body_z_delta > 0.0 else self.body_z_fall_rate
            max_body_z_delta = body_z_rate * self.control_dt
            self.state.body_local_position[2] += np.clip(body_z_delta, -max_body_z_delta, max_body_z_delta)

        self.command.velocity[np.abs(self.command.velocity) < self.velocity_deadband] = 0.0
        self.command.yaw_rate[np.abs(self.command.yaw_rate) < self.yaw_deadband] = 0.0

    def run(self):
        self.update_motion_profile()
        # Возвращаем данные текущего контроллера
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([
            [self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front, -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [0, 0, 0, 0]
        ])
