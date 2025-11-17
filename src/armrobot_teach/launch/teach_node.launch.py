from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock when teaching/replaying.'
    )

    group_arg = DeclareLaunchArgument(
        'group_name',
        default_value='arm',
        description='MoveIt planning group to command.'
    )

    use_sim = LaunchConfiguration('use_sim_time')
    group_name = LaunchConfiguration('group_name')

    teach_node = Node(
        package='armrobot_teach',
        executable='teach_node',
        name='armrobot_teach_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim},
            {'group_name': group_name},
        ],
    )

    return LaunchDescription([
        use_sim_arg,
        group_arg,
        teach_node,
    ])
