import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Path setup (Change 'marble_control' to your actual package name)
    pkg_path = get_package_share_directory('Marble_control')
    xacro_file = os.path.join(pkg_path, 'urdf', 'marble_robot.urdf.xacro')

    # This command executes xacro and converts it to a raw URDF string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Gazebo Launch
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + os.path.join(get_package_share_directory('gazebo_ros'), 'config', 'gazebo_ros_pkgs_params.yaml')}.items()
    )

    # 4. Spawn the robot into Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'ur7e_robot'],
                        output='screen')
                        

    # 5. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 6. Controller Manager Loaders
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur7e_arm_controller"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
    ])