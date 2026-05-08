import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_output_dir():
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        if (parent / "src" / "gazebo_sim" / "world" / "city_cv.sdf").exists():
            return str(parent / "datasets" / "city_cv_yolo_raw")
    return os.path.join(os.getcwd(), "datasets", "city_cv_yolo_raw")


def _default_world_path():
    launch_file = Path(__file__).resolve()
    for parent in launch_file.parents:
        world_path = parent / "src" / "gazebo_sim" / "world" / "city_cv.sdf"
        if world_path.exists():
            return str(world_path)
    return PathJoinSubstitution([FindPackageShare("gazebo_sim"), "world", "city_cv.sdf"])


def generate_launch_description():
    package_share = FindPackageShare("go2_dataset_tools")
    gazebo_share = FindPackageShare("gazebo_sim")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")
    output_dir = LaunchConfiguration("output_dir")
    num_images = LaunchConfiguration("num_images")
    width = LaunchConfiguration("width")
    height_image = LaunchConfiguration("height_image")
    camera_height = LaunchConfiguration("camera_height")
    horizontal_fov = LaunchConfiguration("horizontal_fov")
    seed = LaunchConfiguration("seed")
    start_gazebo = LaunchConfiguration("start_gazebo")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    camera_model_name = LaunchConfiguration("camera_model_name")

    default_world_path = _default_world_path()
    default_output_dir = _default_output_dir()
    gazebo_models_path = PathJoinSubstitution([gazebo_share, "models"])
    dataset_models_path = PathJoinSubstitution([package_share, "models"])
    camera_model_path = PathJoinSubstitution(
        [package_share, "models", "dataset_camera", "model.sdf"]
    )

    gz_resource_path = [
        gazebo_models_path,
        ":",
        dataset_models_path,
        ":",
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
    ]

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", start_gazebo, "' == 'true' and '", use_gazebo_gui, "' == 'true'"]
            )
        ),
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-s -r -v4 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                ["'", start_gazebo, "' == 'true' and '", use_gazebo_gui, "' != 'true'"]
            )
        ),
    )

    spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dataset_camera",
        output="screen",
        arguments=[
            "-world",
            world_name,
            "-file",
            camera_model_path,
            "-name",
            camera_model_name,
            "-allow_renaming",
            "false",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            camera_height,
        ],
    )

    dataset_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="dataset_camera_bridge",
        output="screen",
        arguments=[
            "/dataset_camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/dataset_camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
    )

    collector = Node(
        package="go2_dataset_tools",
        executable="collect_city_cv_dataset",
        name="collect_city_cv_dataset",
        output="screen",
        arguments=[
            "--world-path",
            world_path,
            "--world-name",
            world_name,
            "--output-dir",
            output_dir,
            "--num-images",
            num_images,
            "--width",
            width,
            "--height-image",
            height_image,
            "--camera-height",
            camera_height,
            "--horizontal-fov",
            horizontal_fov,
            "--seed",
            seed,
            "--camera-model-name",
            camera_model_name,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_path",
                default_value=default_world_path,
                description="Absolute path to city_cv.sdf.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="city_second",
                description="Gazebo Sim world name inside the SDF.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value=default_output_dir,
                description="Dataset root directory. Images go into output_dir/images.",
            ),
            DeclareLaunchArgument(
                "num_images",
                default_value="6000",
                description="Number of images to collect.",
            ),
            DeclareLaunchArgument(
                "width",
                default_value="640",
                description="Dataset camera image width.",
            ),
            DeclareLaunchArgument(
                "height_image",
                default_value="360",
                description="Dataset camera image height.",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="0.35",
                description="Dataset camera height above ground.",
            ),
            DeclareLaunchArgument(
                "horizontal_fov",
                default_value="1.6",
                description="Dataset camera horizontal field of view in radians.",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="42",
                description="Random seed for reproducible camera poses.",
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Start Gazebo with world_path before collecting.",
            ),
            DeclareLaunchArgument(
                "use_gazebo_gui",
                default_value="true",
                description="Start Gazebo GUI. If false, run gz sim -s headless server.",
            ),
            DeclareLaunchArgument(
                "camera_model_name",
                default_value="dataset_camera",
                description="Gazebo model name used by /world/<world>/set_pose.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", gz_resource_path),
            gazebo_gui,
            gazebo_server,
            dataset_camera_bridge,
            TimerAction(period=5.0, actions=[spawn_camera]),
            TimerAction(period=6.0, actions=[collector]),
        ]
    )
