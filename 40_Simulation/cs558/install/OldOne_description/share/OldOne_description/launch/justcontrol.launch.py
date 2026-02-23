from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('OldOne_description')

    # Build robot_description from xacro
    xacro_file = os.path.join(share_dir, 'urdf', 'OldOne.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # RViz config (optional)
    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    # Publishes TF using robot_description + incoming /joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        output='screen',
    )

    # NOTE:
    # We intentionally do NOT launch joint_state_publisher / joint_state_publisher_gui.
    # You will publish /joint_states from the terminal.

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])