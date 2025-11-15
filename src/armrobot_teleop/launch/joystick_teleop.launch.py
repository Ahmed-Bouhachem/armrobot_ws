from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )
    use_sim_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation clock",
    )

    joy_dev = LaunchConfiguration("joy_dev")
    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"dev": joy_dev}],
    )

    teleop_node = Node(
        package="armrobot_teleop",
        executable="joystick_teleop",
        name="armrobot_joystick_teleop",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        joystick_dev_arg,
        use_sim_arg,
        joy_node,
        teleop_node,
    ])
