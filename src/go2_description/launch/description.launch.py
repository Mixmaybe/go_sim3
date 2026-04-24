import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to files and directories
    go1_description_pkg = get_package_share_directory('go2_description')
    xacro_path = os.path.join(go1_description_pkg, 'xacro', 'robot.xacro')
    
    # Declare launch arguments
    user_debug_arg = DeclareLaunchArgument(
        'user_debug',
        default_value='false',
        description='Enable debug mode'
    )
    robot_model = LaunchConfiguration('robot_model', default='model_0')
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='model_0',
        description='Модель робота: model_0, model_1, model_3, model_5, model_7',
        choices=['model_0', 'model_1', 'model_3', 'model_5', 'model_7']
    )
    robot_name = 'robot1'
    
    # Command to convert xacro to robot_description
    robot_desc = Command([
        'xacro ', xacro_path,
        ' robot_name:=', robot_name,
        ' robot_model:=', robot_model
    ])
    params_robot_state_publisher = {'robot_description': robot_desc, 'use_sim_time': True}

    # Node to spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_robot_state_publisher],
    )
    
    # Joint state publisher GUI
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}],
        output='screen'
    )

    # Node to spawn joint group controller
    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Node for controlling the robot
    controller = Node(
        package='quadropted_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        output='screen',
    )

    # IMU bridge node
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_imu",
        arguments=["/imu_plugin/out@sensor_msgs/msg/Imu@ignition.msgs.IMU"],
        output='screen',
    )

    # Command velocity publisher
    cmd_vel_pub = Node(
        package='quadropted_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_pub',
        output='screen',
    )

    # Launch description
    return LaunchDescription([
        # Declare user_debug argument
        user_debug_arg,
        robot_model_arg,
        node_robot_state_publisher,
        joint_state_gui,
        spawn_controller,
        # Uncomment if needed
        # controller,
        # imu_bridge,
        # cmd_vel_pub,
        joint_state_broadcaster_spawner
    ])
