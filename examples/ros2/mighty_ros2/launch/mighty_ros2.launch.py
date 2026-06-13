from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_url", default_value=""),
        DeclareLaunchArgument("start_vio", default_value="true"),
        DeclareLaunchArgument("stop_vio_on_shutdown", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("use_device_time", default_value="false"),
        DeclareLaunchArgument("frame_id", default_value="odom"),
        DeclareLaunchArgument("child_frame_id", default_value="base_link"),
        Node(
            package="mighty_ros2",
            executable="mighty_ros2_publisher",
            name="mighty_ros2_publisher",
            output="screen",
            parameters=[{
                "base_url": LaunchConfiguration("base_url"),
                "start_vio": LaunchConfiguration("start_vio"),
                "stop_vio_on_shutdown": LaunchConfiguration("stop_vio_on_shutdown"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "use_device_time": LaunchConfiguration("use_device_time"),
                "frame_id": LaunchConfiguration("frame_id"),
                "child_frame_id": LaunchConfiguration("child_frame_id"),
            }],
        ),
    ])
