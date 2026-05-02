import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)
    robots_file_path = os.path.join(pkg_path, 'config', 'robots.yaml')

    # Загрузка данных из YAML файла
    with open(robots_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    robots = yaml_data['robots']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Использовать симуляционное время'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value='true', description='Enable rviz launch'
    )
    enable_gui = LaunchConfiguration('enable_gui', default='true')
    declare_enable_gui = DeclareLaunchArgument(
        name='enable_gui', default_value='true', description='Enable Go2 GUI control'
    )
    enable_odom_debug = LaunchConfiguration('enable_odom_debug', default='true')
    declare_enable_odom_debug = DeclareLaunchArgument(
        name='enable_odom_debug',
        default_value='true',
        description='Enable ground truth odometry and odometry metrics debug nodes'
    )
    enable_ekf = LaunchConfiguration('enable_ekf', default='false')
    declare_enable_ekf = DeclareLaunchArgument(
        name='enable_ekf',
        default_value='false',
        description='Enable optional robot_localization EKF for leg odometry'
    )
    use_ground_truth_odom = LaunchConfiguration('use_ground_truth_odom', default='true')
    declare_use_ground_truth_odom = DeclareLaunchArgument(
        name='use_ground_truth_odom',
        default_value='true',
        description='Use Gazebo ground truth as simulation navigation odometry'
    )
    robot_model = LaunchConfiguration('robot_model', default='model_0')
    declare_robot_model = DeclareLaunchArgument(
        name='robot_model',
        default_value='model_0',
        description='Модель робота: model_0, model_1, model_3, model_5, model_7',
        choices=['model_0', 'model_1', 'model_3', 'model_5', 'model_7']
    )

    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_enable_gui)
    ld.add_action(declare_enable_odom_debug)
    ld.add_action(declare_enable_ekf)
    ld.add_action(declare_use_ground_truth_odom)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_model)

    remappings_initial = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/scan", "scan"),
        ("/odom", "odometry/filtered")
    ]
    
    map_server = Node(package='nav2_map_server',
                      executable='map_server',
                      name='map_server',
                      output='screen',
                      parameters=[{'yaml_filename': os.path.join(pkg_path, 'maps', 'cambridge.yaml'),
                                   }, ],
                      remappings=remappings_initial)

    map_server_lifecycle = Node(package='nav2_lifecycle_manager',
                                executable='lifecycle_manager',
                                name='lifecycle_manager_map_server',
                                output='screen',
                                parameters=[{'use_sim_time': use_sim_time},
                                            {'autostart': True},
                                            {'node_names': ['map_server']}])

    # ld.add_action(map_server)
    # ld.add_action(map_server_lifecycle)


    remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/scan", "scan"),
            ("/odom", "odometry/filtered")
        ]
    

    bridge_params = os.path.join(pkg_path,'config','gz_bridge.yaml')
    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ld.add_action(ros_gz_bridge_clock)   

    
    last_action = None
    xacro_file = os.path.join(get_package_share_directory('go2_description'), 'xacro', 'robot.xacro')

    for i, robot in enumerate(robots):
        namespace = robot['name']
        robot_name = robot['name']
        robot1_condition = IfCondition(PythonExpression(["'", namespace, "' == 'robot1'"]))
        robot1_debug_condition = IfCondition(
            PythonExpression(["'", enable_odom_debug, "' == 'true' and '", namespace, "' == 'robot1'"])
        )
        robot1_ground_truth_condition = IfCondition(
            PythonExpression(
                [
                    "('", enable_odom_debug, "' == 'true' or '",
                    use_ground_truth_odom, "' == 'true') and '",
                    namespace, "' == 'robot1'"
                ]
            )
        )
        robot1_ekf_condition = IfCondition(
            PythonExpression(
                [
                    "'", enable_ekf, "' == 'true' and '",
                    use_ground_truth_odom, "' != 'true' and '",
                    namespace, "' == 'robot1'"
                ]
            )
        )
        robot_desc = Command([
            'xacro ', xacro_file,
            ' robot_name:=', robot_name,
            ' robot_model:=', robot_model
        ])
        params_robot_state_publisher = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}

        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[params_robot_state_publisher],
            remappings=remappings
        )

        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-world', 'city_second',
                '-topic', f'/{namespace}/robot_description',
                '-name', f'{namespace}_my_bot',
                '-allow_renaming', 'true',
                '-x', robot['x_pose'],
                '-y', robot['y_pose'],
                '-z', robot['z_pose'],
                # '-Y', robot['Y_pose']
            ],
            output='screen'
        )

        ros_gz_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=namespace,
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                f'/{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
                f'/{namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/model/{namespace}_my_bot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # f'/{namespace}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/{namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                f'/{namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                f'/{namespace}/color/image_rect@sensor_msgs/msg/Image@gz.msgs.Image',
                # f'/{namespace}/ground_truth_tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                # f'/{namespace}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
            ]
        )
        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            namespace=namespace,
            arguments=['color/image_raw', 'color/image_rect'],
            output='screen',
        )


        joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_state_broadcaster',
            arguments=['joint_state_broadcaster'],
            output='screen',
            remappings=remappings
        )

        joint_group_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            name='joint_group_controller',
            arguments=['joint_group_controller'],
            output='screen',
            remappings=remappings
        )

        controller = Node(
            package='quadropted_controller',
            executable='robot_controller_gazebo.py',
            name='quadruped_controller',
            namespace=namespace,
            output='screen',
            remappings=remappings
        )

        foot_contact_estimator = Node(
            package='quadropted_controller',
            executable='FootContactEstimatorNode.py',
            name='foot_contact_estimator',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'joint_states_topic': 'joint_states',
                'ground_truth_topic': f'/model/{namespace}_my_bot/pose',
                'model_frame_id': f'{namespace}_my_bot',
                'world_frame_id': 'city_second',
                'allow_fallback_first_transform': False,
                'imu_topic': 'imu_plugin/out',
                'expected_contacts_topic': 'foot_contacts_expected',
                'use_adaptive_ground_z': True,
                'publish_rate': 50,
                'verbose': False,
            }],
            condition=robot1_condition,
            remappings=remappings
        )

        # apriltag_launch_file = os.path.join(get_package_share_directory('yahboom_rosmaster_docking'), 'launch', 'apriltag_dock_pose_publisher.launch.py')
        # aprilTag = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(apriltag_launch_file),
        #     launch_arguments={
        #         'camera_namespace': namespace,
        #         'camera_frame_type': 'camera_face'
        #     }.items()
        # )



        odom = Node(
            package='quadropted_controller',
            executable='QuadrupedOdometryNode.py',
            name='odom',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                "verbose": False,
                'publish_rate': 50,
                'use_imu_heading': True,
                'is_gazebo': True,
                'joint_states_topic': 'joint_states',
                'measured_contacts_topic': 'foot_contacts_measured',
                'expected_contacts_topic': 'foot_contacts_expected',
                'imu_topic': 'imu_plugin/out',
                'robot_velocity_topic': 'robot_velocity',
                'amcl_pose_topic': 'amcl_pose',
                'base_frame_id': "base_link",
                'odom_frame_id': "odom",
                'clock_topic': f'/clock',
                'enable_odom_tf': ParameterValue(
                    PythonExpression(
                        [
                            "'", use_ground_truth_odom, "' != 'true' and '",
                            enable_ekf, "' != 'true'"
                        ]
                    ),
                    value_type=bool
                ),
                'publish_legacy_odom': ParameterValue(
                    PythonExpression(
                        [
                            "'", use_ground_truth_odom, "' != 'true' and '",
                            enable_ekf, "' != 'true'"
                        ]
                    ),
                    value_type=bool
                ),
                'publish_filtered_odom': ParameterValue(
                    PythonExpression(
                        [
                            "'", use_ground_truth_odom, "' != 'true' and '",
                            enable_ekf, "' != 'true'"
                        ]
                    ),
                    value_type=bool
                ),
                'min_stable_contacts': 1,
                'no_contact_mode': 'freeze_xy',
                'use_amcl_soft_correction': False,
            }],
            remappings=remappings
        )

        ground_truth_odom = Node(
            package='quadropted_controller',
            executable='GroundTruthOdometryNode.py',
            name='ground_truth_odometry',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ground_truth_pose_topic': f'/model/{namespace}_my_bot/pose',
                'model_frame_id': f'{namespace}_my_bot',
                'world_frame_id': 'city_second',
                'allow_fallback_first_transform': False,
                'ground_truth_frame_id': 'map',
                'child_frame_id': 'ground_truth_base_link',
                'publish_tf': False,
                'publish_nav_odom': ParameterValue(
                    PythonExpression(["'", use_ground_truth_odom, "' == 'true'"]),
                    value_type=bool
                ),
                'publish_nav_tf': ParameterValue(
                    PythonExpression(["'", use_ground_truth_odom, "' == 'true'"]),
                    value_type=bool
                ),
                'nav_odom_topic': 'odom',
                'nav_filtered_topic': 'odometry/filtered',
                'nav_odom_frame_id': 'odom',
                'nav_base_frame_id': 'base_link',
                'zero_start': True,
                'verbose': False,
            }],
            condition=robot1_ground_truth_condition,
            remappings=remappings
        )

        odometry_evaluator = Node(
            package='quadropted_controller',
            executable='OdometryEvaluatorNode.py',
            name='odometry_evaluator',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'estimated_odom_topic': 'leg_odom_raw',
                'filtered_odom_topic': 'odometry/filtered',
                'legacy_odom_topic': 'odom',
                'ground_truth_topic': 'ground_truth/odom',
                'amcl_pose_topic': 'amcl_pose',
                'goal_pose_topic': 'goal_pose',
                'leg_odom_debug_topic': 'leg_odom_debug',
                'imu_topic': 'imu_plugin/out',
                'publish_rate': 2.0,
                'csv_path': '/tmp/go_sim3_odometry_metrics_robot1.csv',
                'verbose': False,
            }],
            condition=robot1_debug_condition,
            remappings=remappings
        )

        robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf_leg_odom.yaml')
        start_robot_localization_cmd = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}],
            condition=robot1_ekf_condition,
            remappings=remappings
        )

        nav2_launch_file = os.path.join(pkg_path, 'launch', 'nav2', 'bringup_launch.py')
        map_yaml_file = os.path.join(pkg_path, 'maps', 'city_second_new.yaml')
        params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

        message = f"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {robot['x_pose']}, y: {robot['y_pose']}, z: 0.1}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, }} }}"

        initial_pose_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable',
                f'/{namespace}/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message
            ],
            output='screen'
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_yaml_file,
                'use_namespace': 'True',
                'namespace': namespace,
                'params_file': params_file,  
                'autostart': 'true',
                'use_sim_time': 'true',
                'log_level': 'warn',
                'map_server': 'True'
            }.items()
        )

        nav2_actions = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            bringup_cmd,
            initial_pose_cmd,
        ])

        rviz_launch_file = os.path.join(pkg_path, 'launch', 'rviz_launch.py')
        rviz_config_file = os.path.join(pkg_path, 'rviz', 'nav2_default_view.rviz')

        rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": rviz_config_file,
            }.items(),
            condition=IfCondition(enable_rviz)
        )

        cmd_vel_pub = Node(
            package='quadropted_controller',
            executable='cmd_vel_pub.py',
            namespace=namespace,
            name='cmd_vel_pub',
            output='screen',
            remappings=remappings
        )

        gui_control = Node(
            package='quadropted_controller',
            executable='go2_gui_control.py',
            namespace=namespace,
            name='go2_gui_control',
            output='screen',
            parameters=[{
                'robot_namespace': namespace,
                'robot_id': i + 1,
            }],
            condition=IfCondition(enable_gui)
        )

        fake_bms = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', f'/{namespace}/battery_state', 'sensor_msgs/msg/BatteryState',
                "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, voltage: 24.0, percentage: 0.8, capacity: 10.0}",
                '-r', '1'
            ],
            output='log'
        )
        # robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
        # # Start robot localization using an Extended Kalman filter
        # start_robot_localization_cmd = Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     namespace=namespace,
        #     output='screen',
        #     parameters=[robot_localization_file_path, 
        #     {'use_sim_time': use_sim_time}],
        #     remappings=remappings)


        robot_control = GroupAction([
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            joint_state_broadcaster,
            joint_group_controller,
            foot_contact_estimator,
            controller,
            cmd_vel_pub,
            gui_control,
            ground_truth_odom,
            odom,
            odometry_evaluator,
            start_robot_localization_cmd,
            # aprilTag,
            fake_bms,
        ])
        test_action = Node(
            package='gazebo_sim',
            executable='test_action.py',
            namespace=namespace,
            name='test_action',
            output='screen',
            remappings=remappings
        )

        # Группировка всех действий для робота
        robot_group = GroupAction([
            node_robot_state_publisher,
            spawn_entity,
            ros_gz_bridge,
            start_gazebo_ros_image_bridge_cmd,
            robot_control,
            nav2_actions,
            rviz,
            # test_action
        ])

        if last_action is None:
            ld.add_action(robot_group)
        else:
            spawn_robot_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[robot_group]
                )
            )
            ld.add_action(spawn_robot_event)

        last_action = joint_group_controller

    return ld
