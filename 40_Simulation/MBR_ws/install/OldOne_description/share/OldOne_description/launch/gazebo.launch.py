from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('OldOne_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'OldOne.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()


        # Absolute path to the STL
    zeball_stl = os.path.join(share_dir, 'meshes', 'zEBALL_1.stl')
    zeball_uri = 'file://' + zeball_stl  # absolute file URI

    # Read the template URDF and replace the placeholder
    zeball_template = os.path.join(share_dir, 'urdf', 'zeball.urdf')
    zeball_resolved = os.path.join(share_dir, 'urdf', 'zeball_resolved.urdf')

    with open(zeball_template, 'r') as f:
        urdf_txt = f.read().replace('REPLACE_WITH_ABSOLUTE_FILE_URI', zeball_uri)

    with open(zeball_resolved, 'w') as f:
        f.write(urdf_txt)







    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'OldOne',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    
    ball_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ZEBALL',
            '-file', zeball_resolved,

            # ball CENTER pose so it sits on top center of the plate at zero joints
            '-x', '-0.0997',
            '-y', '-0.120815',
            '-z', '1.1441',
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        ball_spawn_node,
    ])
