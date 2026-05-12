from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_yolo = LaunchConfiguration("enable_yolo", default="true")
    image_topic = LaunchConfiguration("image_topic", default="/robot1/color/image_raw")
    yolo_model_path = LaunchConfiguration(
        "yolo_model_path", default="models/yolo/city_cv_yolo11n_best.pt"
    )
    yolo_period_sec = LaunchConfiguration("yolo_period_sec", default="2.0")
    yolo_conf = LaunchConfiguration("yolo_conf", default="0.5")
    yolo_device = LaunchConfiguration("yolo_device", default="0")
    show_yolo_window = LaunchConfiguration("show_yolo_window", default="true")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_yolo", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/robot1/color/image_raw"),
            DeclareLaunchArgument(
                "yolo_model_path", default_value="models/yolo/city_cv_yolo11n_best.pt"
            ),
            DeclareLaunchArgument("yolo_period_sec", default_value="2.0"),
            DeclareLaunchArgument("yolo_conf", default_value="0.5"),
            DeclareLaunchArgument("yolo_device", default_value="0"),
            DeclareLaunchArgument("show_yolo_window", default_value="true"),
            Node(
                package="go2_vision",
                executable="yolo_detector_node",
                name="yolo_detector_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "model_path": yolo_model_path,
                        "inference_period_sec": ParameterValue(
                            yolo_period_sec, value_type=float
                        ),
                        "confidence_threshold": ParameterValue(
                            yolo_conf, value_type=float
                        ),
                        "imgsz": 640,
                        "device": yolo_device,
                        "publish_annotated_image": True,
                        "enable_yolo": ParameterValue(enable_yolo, value_type=bool),
                        "show_yolo_window": ParameterValue(
                            show_yolo_window, value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(enable_yolo),
            ),
        ]
    )
