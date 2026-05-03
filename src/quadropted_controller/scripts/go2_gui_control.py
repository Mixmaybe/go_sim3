#!/usr/bin/env python3
import queue
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand
from quadropted_msgs.srv import RobotBehaviorCommand


class Go2GuiNode(Node):
    def __init__(self, log_queue: queue.Queue):
        super().__init__('go2_gui_control')

        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('robot_id', 1)

        namespace = self.get_parameter('robot_namespace').value.strip('/')
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.ns = f'/{namespace}'
        self.log_queue = log_queue

        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{self.ns}/cmd_vel', 10
        )
        self.mode_pub = self.create_publisher(
            RobotModeCommand, f'{self.ns}/robot_mode', 10
        )
        self.behavior_cli = self.create_client(
            RobotBehaviorCommand,
            f'{self.ns}/robot_behavior_command'
        )

        self.current_behavior = None
        self.current_mode = None
        self._last_twist = None

        self.log(f'GUI node started for {self.ns}, robot_id={self.robot_id}')

    def log(self, text: str):
        self.get_logger().info(text)
        self.log_queue.put(text)

    def publish_twist(self, lx: float = 0.0, ly: float = 0.0, az: float = 0.0):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.angular.z = float(az)

        signature = (msg.linear.x, msg.linear.y, msg.angular.z)
        if signature != self._last_twist:
            self.cmd_vel_pub.publish(msg)
            self._last_twist = signature
            self.log(f'cmd_vel: x={lx:.2f}, y={ly:.2f}, yaw={az:.2f}')

    def stop(self):
        self.publish_twist(0.0, 0.0, 0.0)

    def publish_mode(self, mode: str):
        if self.current_mode == mode:
            return
        msg = RobotModeCommand()
        msg.mode = mode
        msg.robot_id = self.robot_id
        self.mode_pub.publish(msg)
        self.current_mode = mode
        self.log(f'robot_mode: {mode}')

    def call_behavior(self, command: str):
        if self.current_behavior == command and command != 'sit':
            return

        if not self.behavior_cli.wait_for_service(timeout_sec=0.5):
            self.log('robot_behavior_command service is unavailable')
            return

        req = RobotBehaviorCommand.Request()
        req.command = command
        future = self.behavior_cli.call_async(req)
        future.add_done_callback(self._behavior_done)

        self.current_behavior = command
        self.log(f'behavior request: {command}')

    def _behavior_done(self, future):
        try:
            response = future.result()
            self.log(
                f'behavior response: success={response.success}, message={response.message}'
            )
        except Exception as exc:
            self.log(f'behavior call failed: {exc}')

    def ensure_walk(self):
        self.call_behavior('walk')
        self.publish_mode('TROT')

    def ensure_stand(self):
        self.call_behavior('up')
        self.publish_mode('STAND')

    def sit(self):
        self.stop()
        self.call_behavior('sit')
        self.current_mode = None


class Go2GuiApp:
    def __init__(self, root: tk.Tk, node: Go2GuiNode, log_queue: queue.Queue):
        self.root = root
        self.node = node
        self.log_queue = log_queue

        self.root.title('Go2 Control Panel')
        self.root.geometry('540x560')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.forward_speed = tk.DoubleVar(value=0.50)
        self.strafe_speed = tk.DoubleVar(value=0.90)
        self.turn_speed = tk.DoubleVar(value=0.70)

        self._build_ui()
        self._drain_logs()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill='both', expand=True)

        posture = ttk.LabelFrame(main, text='Положение и режим', padding=8)
        posture.pack(fill='x', pady=(0, 10))

        ttk.Button(
            posture, text='Поднять', command=self.node.ensure_walk
        ).grid(row=0, column=0, padx=4, pady=4, sticky='ew')

        ttk.Button(
            posture, text='Опустить', command=self.node.sit
        ).grid(row=0, column=1, padx=4, pady=4, sticky='ew')

        for i in range(2):
            posture.columnconfigure(i, weight=1)

        speed = ttk.LabelFrame(main, text='Скорости', padding=8)
        speed.pack(fill='x', pady=(0, 10))

        self._add_scale(speed, 0, 'Вперёд / назад', self.forward_speed, 0.05, 2.0)
        self._add_scale(speed, 1, 'Шаг вбок', self.strafe_speed, 0.05, 1.0)
        self._add_scale(speed, 2, 'Поворот', self.turn_speed, 0.05, 2.5)

        controls = ttk.LabelFrame(main, text='Движение', padding=8)
        controls.pack(fill='x', pady=(0, 10))

        self._motion_button(
            controls, 'Поворот влево', 0, 0,
            lambda: self._rotate(self.turn_speed.get())
        )
        self._motion_button(
            controls, 'Вперёд', 0, 1,
            lambda: self._move(lx=self.forward_speed.get())
        )
        self._motion_button(
            controls, 'Поворот вправо', 0, 2,
            lambda: self._rotate(-self.turn_speed.get())
        )

        self._motion_button(
            controls, 'Влево', 1, 0,
            lambda: self._move(ly=self.strafe_speed.get())
        )
        self._motion_button(
            controls, 'Стоп', 1, 1,
            self.node.stop, release_stop=False
        )
        self._motion_button(
            controls, 'Вправо', 1, 2,
            lambda: self._move(ly=-self.strafe_speed.get())
        )

        self._motion_button(
            controls, 'Назад', 2, 1,
            lambda: self._move(lx=-self.forward_speed.get())
        )

        for r in range(3):
            controls.rowconfigure(r, weight=1)
        for c in range(3):
            controls.columnconfigure(c, weight=1)

        log_frame = ttk.LabelFrame(main, text='Лог', padding=8)
        log_frame.pack(fill='both', expand=True)

        self.log_text = tk.Text(log_frame, height=12, state='disabled')
        self.log_text.pack(fill='both', expand=True)

    def _add_scale(self, parent, row, label, var, min_v, max_v):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=4, pady=4)
        ttk.Scale(
            parent, from_=min_v, to=max_v, variable=var, orient='horizontal'
        ).grid(row=row, column=1, sticky='ew', padx=4, pady=4)

        value_label = ttk.Label(parent, text='')
        value_label.grid(row=row, column=2, sticky='e', padx=4, pady=4)

        def update_label(*_):
            value_label.config(text=f'{var.get():.2f}')

        var.trace_add('write', update_label)
        update_label()
        parent.columnconfigure(1, weight=1)

    def _motion_button(self, parent, text, row, col, command, release_stop=True):
        btn = ttk.Button(parent, text=text)
        btn.grid(row=row, column=col, padx=4, pady=4, sticky='nsew')
        btn.bind('<ButtonPress-1>', lambda _e: command())

        if release_stop:
            btn.bind('<ButtonRelease-1>', lambda _e: self.node.stop())
            btn.bind('<Leave>', lambda _e: self.node.stop())

        return btn

    def _move(self, lx=0.0, ly=0.0):
        self.node.ensure_walk()
        self.node.publish_twist(lx=lx, ly=ly, az=0.0)

    def _rotate(self, az=0.0):
        self.node.ensure_walk()
        self.node.publish_twist(lx=0.0, ly=0.0, az=az)

    def _drain_logs(self):
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self.log_text.configure(state='normal')
            self.log_text.insert('end', line + '\n')
            self.log_text.see('end')
            self.log_text.configure(state='disabled')
        self.root.after(100, self._drain_logs)

    def on_close(self):
        self.node.stop()
        self.root.after(150, self.root.destroy)


def spin_node(node: Go2GuiNode, stop_event: threading.Event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)

    logs = queue.Queue()
    node = Go2GuiNode(logs)

    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=spin_node, args=(node, stop_event), daemon=True
    )
    spin_thread.start()

    root = tk.Tk()
    Go2GuiApp(root, node, logs)

    try:
        root.mainloop()
    finally:
        stop_event.set()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()