import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    raspi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('raspimouse'), 'launch'),
            '/raspimouse.launch.py'])
    )

    sweep_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rs_teleop_joy'), 'launch'),
            '/teleop.py'])
    )


    ld.add_action(raspi_launch)
    ld.add_action(sweep_launch)
    return ld
