import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths
    pkg_name = 'Marble_control'
    ur_description_dir = get_package_share_directory('ur_description')
    
    # 2. Process URDF (Xacro) manually so we don't need the UR launch files
    xacro_file = os.path.join(ur_description_dir, 'urdf', 'ur.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={'ur_type': 'ur7e', 'name': 'ur7e_robot'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 3. Robot State Publisher (Essential)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. RViz2
    rviz_config_file = os.path.join(ur_description_dir, 'rviz', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # 5. YOUR Joint Publisher (No more GUI to fight!)
    my_publisher = Node(
        package=pkg_name,
        executable='joint_publisher',
        name='marble_joint_pub',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        my_publisher
    ])