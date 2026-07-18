from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    return LaunchDescription([gzclient])
