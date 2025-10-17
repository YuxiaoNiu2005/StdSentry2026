from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # ͨѶ�ڵ�
    sentry_communication_node = Node(
        package="sentry_control",
        executable="sentry_communication",
        output = "screen",
        respawn = True
    )

    # ͨѶ�ڵ�
    sentry_nav_goal_node = Node(
        package="sentry_control",
        executable="sentry_nav_goal",
        output = "screen",
        respawn = True
    )

    navigation_launch_file = os.path.join(
        get_package_share_directory("rm_nav_bringup"),
        "launch",
        "bringup_real.launch.py"
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'world': "RMUL2025",
            'mode': 'mapping',
            # 'world': "RMUL",
            # 'mode': 'nav',
            'lio': 'pointlio',
            'localization': 'slam_toolbox',
            'lio_rviz': 'True',
            'nav_rviz': 'True',
        }.items()
    )
        

    launch_description = LaunchDescription()
    launch_description.add_action(sentry_communication_node)
    launch_description.add_action(sentry_nav_goal_node)
    launch_description.add_action(navigation_launch)

    return launch_description