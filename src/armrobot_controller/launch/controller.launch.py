import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Spawn armrobot controllers from an existing controller manager instance.

    This launch file assumes the Gazebo simulation (from
    `armrobot_description/launch/gazebo.launch.py`) is already running
    and has loaded the ros2_control hardware interface. Only the spawners
    are started here so that the controllers can be reloaded without
    restarting the entire simulator.
    """

    rmw_default = os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

    rmw_arg = DeclareLaunchArgument(
        "rmw_implementation",
        default_value=rmw_default,
        description=(
            "RMW middleware to use when talking to the controller manager. "
            "This must match the middleware used by the running Gazebo instance."
        ),
    )
    controller_manager_arg = DeclareLaunchArgument(
        "controller_manager",
        default_value="/controller_manager",
        description="Fully qualified controller manager namespace.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Propagate simulated time to the spawner nodes.",
    )

    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value=LaunchConfiguration("rmw_implementation"),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            LaunchConfiguration("controller_manager"),
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "arm_controller",
            "--controller-manager",
            LaunchConfiguration("controller_manager"),
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "gripper_controller",
            "--controller-manager",
            LaunchConfiguration("controller_manager"),
        ],
    )

    return LaunchDescription(
        [
            rmw_arg,
            controller_manager_arg,
            use_sim_time_arg,
            set_rmw,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
