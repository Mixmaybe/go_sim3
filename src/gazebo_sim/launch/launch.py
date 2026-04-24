import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter
def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_model = LaunchConfiguration('robot_model', default='model_0')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    enable_gui = LaunchConfiguration('enable_gui', default='true')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                       description='Использовать симуляционное время'))
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

    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))


    world_file = os.path.join(pkg_path, 'world', 'city_second_new.sdf') 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )
    ld.add_action(gazebo)

    pause = ExecuteProcess(
        cmd=['sleep', '15'],
        output='screen'
    )
    ld.add_action(pause)


    multi_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo_multi_nav2_world.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model,
            'enable_rviz': enable_rviz,
            'enable_gui': enable_gui,
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
