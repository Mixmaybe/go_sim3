from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_share = FindPackageShare(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='city_cv.sdf')
    robot_model = LaunchConfiguration('robot_model', default='model_5')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_gui = LaunchConfiguration('enable_gui', default='true')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default='true')
    enable_odom_debug = LaunchConfiguration('enable_odom_debug', default='true')
    enable_ekf = LaunchConfiguration('enable_ekf', default='false')
    use_ground_truth_odom = LaunchConfiguration('use_ground_truth_odom', default='false')
    use_gazebo_truth_odom = LaunchConfiguration('use_gazebo_truth_odom', default='true')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                       description='Использовать симуляционное время'))
    ld.add_action(DeclareLaunchArgument('world', default_value='city_edit2.sdf',
                                       description='SDF world file from gazebo_sim/world'))
    ld.add_action(DeclareLaunchArgument(
        'robot_model',
        default_value='model_0',
        description='Модель робота: model_0, model_1, model_3, model_5, model_7',
        choices=['model_0', 'model_1', 'model_3', 'model_5', 'model_7']
    ))
    ld.add_action(DeclareLaunchArgument('enable_rviz', default_value='true',
                                       description='Запускать RViz'))
    ld.add_action(DeclareLaunchArgument('enable_gui', default_value='true',
                                       description='Запускать GUI управления'))
    ld.add_action(DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Запускать Gazebo GUI. Если false, Gazebo запускается только как server: gz sim -s'
    ))
    ld.add_action(DeclareLaunchArgument('enable_odom_debug', default_value='true',
                                       description='Запускать debug-узлы одометрии'))
    ld.add_action(DeclareLaunchArgument('enable_ekf', default_value='false',
                                       description='Запускать robot_localization EKF'))
    ld.add_action(DeclareLaunchArgument('use_ground_truth_odom', default_value='false',
                                       description='Legacy ground truth odometry mode'))
    ld.add_action(DeclareLaunchArgument('use_gazebo_truth_odom', default_value='true',
                                       description='Использовать Gazebo model pose как рабочую odometry в симуляции'))
    ld.add_action(DeclareLaunchArgument('use_gazebo_truth_odom.py', default_value='true',
                                       description='Compatibility alias for use_gazebo_truth_odom'))

    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))


    world_file = PathJoinSubstitution([pkg_share, 'world', world])
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=IfCondition(use_gazebo_gui)
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        ])),
        launch_arguments={
            'gz_args': ['-s -r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items(),
        condition=UnlessCondition(use_gazebo_gui)
    )

    ld.add_action(gazebo_gui)
    ld.add_action(gazebo_server)

    pause = ExecuteProcess(
        cmd=['sleep', '15'],
        output='screen'
    )
    ld.add_action(pause)


    multi_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'gazebo_multi_nav2_world.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'enable_rviz': enable_rviz,
            'enable_gui': enable_gui,
            'enable_odom_debug': enable_odom_debug,
            'enable_ekf': enable_ekf,
            'use_ground_truth_odom': use_ground_truth_odom,
            'use_gazebo_truth_odom': use_gazebo_truth_odom,
        }.items()
    )

    launch_after_pause = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pause,
            on_exit=[multi_nav2_launch]
        )
    )

    ld.add_action(launch_after_pause)

    return ld
