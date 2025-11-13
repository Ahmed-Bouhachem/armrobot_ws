import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    description_dir = get_package_share_directory("armrobot_description")

    default_model = os.path.join(description_dir, "urdf", "armrobot.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model,
        description="Absolute path to the robot xacro file to load."
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    physics_engine = "" if ros_distro == "humble" else "--physics-engine gz-physics-buller-featherstone-plugin"

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                LaunchConfiguration("model"),
                f"is_ignition:={is_ignition}",
            ]
        ),
        value_type=str,
    )

    resource_paths = [
        os.path.join(description_dir, "models"),
        os.path.join(description_dir, "meshes"),
        description_dir,
    ]

    # Preserve existing paths if users already exported them.
    gz_existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ign_existing = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")

    def _combine(existing: str) -> str:
        combined = os.pathsep.join(path for path in resource_paths if path)
        return f"{combined}{os.pathsep}{existing}" if existing else combined

    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=_combine(gz_existing)
    )

    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=_combine(ign_existing)
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments=[("gz_args", "-r -v 3 empty.sdf")]
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "armrobot",
                ],
            )
        ],
    )

    auto_start_controllers_arg = DeclareLaunchArgument(
        "auto_start_controllers",
        default_value="True",
        description="Automatically spawn ros2_control controllers once the robot is in Gazebo.",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        auto_start_controllers_arg,
        model_arg,
        set_gz_resource,
        set_ign_resource,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            ),
            condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            ),
            condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner],
            ),
            condition=IfCondition(LaunchConfiguration("auto_start_controllers")),
        ),
    ])
