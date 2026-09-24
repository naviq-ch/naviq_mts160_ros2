"""Launch the MTS160 driver.

    ros2 launch naviq_mts160 driver.launch.py                       # socketcan can0, node 10
    ros2 launch naviq_mts160 driver.launch.py can_interface_type:=gs_usb can_channel:=0   # WSL2 bench
    ros2 launch naviq_mts160 driver.launch.py params_file:=/path/to/my.yaml

The driver publishes no TF; publish the sensor's mounting transform from your robot description.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument("namespace", default_value=""),
    ]

    # can_channel is a string parameter but "0" or "1:4" would be YAML-parsed as a number: force the type.
    params = [{
        "can_interface_type": ParameterValue(LaunchConfiguration("can_interface_type"), value_type=str),
        "can_channel": ParameterValue(LaunchConfiguration("can_channel"), value_type=str),
        "can_bitrate": ParameterValue(LaunchConfiguration("can_bitrate"), value_type=int),
        "node_id": ParameterValue(LaunchConfiguration("node_id"), value_type=int),
        "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
        "publish_raw": ParameterValue(LaunchConfiguration("publish_raw"), value_type=bool),
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

    return LaunchDescription(args + [driver])
