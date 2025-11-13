from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("armrobot_description")
    default_model = LaunchConfiguration("model")

    declare_model = DeclareLaunchArgument(
        "model",
        default_value=f"{package_share}/urdf/armrobot_simple.urdf.xacro",
        description="Absolute path to the simplified robot xacro file.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", default_model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{get_package_share_directory('gazebo_ros')}/launch/gazebo.launch.py"
        )
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-topic", "robot_description", "-entity", "armrobot_simple"],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            declare_model,
            robot_state_publisher,
            gazebo,
            spawn_robot,
        ]
    )
