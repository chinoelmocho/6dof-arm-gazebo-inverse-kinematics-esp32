import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join

def generate_launch_description():

    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros_gz_rbot = get_package_share_directory('BRAZOROBOTICO_description')

    # Let Gazebo find this package's meshes/models even when referenced
    # with a relative model:// or package-relative URI.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=pkg_ros_gz_rbot + ':' + os.path.join(pkg_ros_gz_rbot, '..') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_ros_gz_rbot + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'BRAZOROBOTICO.xacro')

    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description_xml = robot_description_config.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # Write the processed URDF to a file so spawn_entity.py can load it
    # directly (-file) instead of waiting on the /robot_description topic,
    # which avoids relying on ROS 2 node discovery at spawn time.
    urdf_tmp_path = os.path.join(tempfile.gettempdir(), 'BRAZOROBOTICO.urdf')
    with open(urdf_tmp_path, 'w') as f:
        f.write(robot_description_xml)

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Start Gazebo Classic server and client (GUI)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Spawn Robot in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-file", urdf_tmp_path,
            "-entity", "BRAZOROBOTICO",
            "-timeout", "60",
            "-z", "0.32",
            "-x", "0.0",
            "-y", "0.0",
            "-Y", "0.0"
        ],
        output='screen',
    )

    # Load controllers via the controller_manager once the robot has been
    # spawned (the gazebo_ros2_control plugin only creates the
    # controller_manager services after the entity is loaded into gzserver).
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            set_gazebo_model_path,
            set_gazebo_resource_path,
            # Nodes and Launches
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn,
            load_joint_state_broadcaster,
            load_arm_controller,
        ]
    )
