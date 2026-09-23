"""Launch the MTS160 driver.

    ros2 launch naviq_mts160 driver.launch.py                       # socketcan can0, node 10
    ros2 launch naviq_mts160 driver.launch.py can_interface_type:=gs_usb can_channel:=0   # WSL2 bench
    ros2 launch naviq_mts160 driver.launch.py params_file:=/path/to/my.yaml

An example static transform from base_link to the sensor is included
(disable with publish_static_tf:=false); the driver itself publishes no TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("can_interface_type", default_value="socketcan",
                              description="python-can backend: socketcan (Linux robot) or gs_usb (WSL2 bench)"),
        DeclareLaunchArgument("can_channel", default_value="can0",
                              description="socketcan interface name, or gs_usb device index / bus:address / serial"),
        DeclareLaunchArgument("can_bitrate", default_value="500000"),
        DeclareLaunchArgument("node_id", default_value="10"),
        DeclareLaunchArgument("frame_id", default_value="mts160_link"),
        DeclareLaunchArgument("publish_raw", default_value="false"),
        DeclareLaunchArgument("params_file", default_value="",
                              description="optional YAML with further parameters (overrides the arguments above)"),
        DeclareLaunchArgument("publish_static_tf", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    params = [{
        "can_interface_type": LaunchConfiguration("can_interface_type"),
        "can_channel": LaunchConfiguration("can_channel"),
        "can_bitrate": LaunchConfiguration("can_bitrate"),
        "node_id": LaunchConfiguration("node_id"),
        "frame_id": LaunchConfiguration("frame_id"),
        "publish_raw": LaunchConfiguration("publish_raw"),
    }]
    # A params_file, if given, is applied on top of the launch arguments.
    params_file = LaunchConfiguration("params_file")

    driver = Node(
        package="naviq_mts160",
        executable="mts160",
        name="mts160",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=params + [params_file],
    )

    # Example only: sensor 0.30 m ahead of base_link, 0.02 m above the floor, facing down.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mts160_static_tf",
        arguments=["--x", "0.30", "--y", "0.0", "--z", "0.02",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "base_link", "--child-frame-id", LaunchConfiguration("frame_id")],
        condition=IfCondition(LaunchConfiguration("publish_static_tf")),
    )

    return LaunchDescription(args + [driver, static_tf])
