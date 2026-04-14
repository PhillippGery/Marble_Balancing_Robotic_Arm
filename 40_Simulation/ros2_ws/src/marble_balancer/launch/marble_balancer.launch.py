import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = get_package_share_directory('marble_balancer')

    # ── Arguments ─────────────────────────────────────────────────────────────
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='UR robot type (ur5e, ur7e, ...)',
    )

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for robot visualisation.',
    )

    enable_lqr_arg = DeclareLaunchArgument(
        'enable_lqr',
        default_value='false',
        description='Set true to spawn the marble and start the LQR controller.',
    )

    # ── 1. UR simulation ──────────────────────────────────────────────────────
    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gazebo'),
            '/launch/ur_sim_control.launch.py',
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('enable_rviz'),
            'controllers_file': os.path.join(pkg_share, 'config', 'ur_controllers.yaml'),
            'initial_joint_controller': 'ur7e_arm_controller',
            # USE STRING FORMATTING TO ENSURE ROS2 PARSES THE DICT CORRECTLY
            'initial_joint_positions': '{shoulder_pan_joint: 0.0, shoulder_lift_joint: -1.8, elbow_joint: -1.8, wrist_1_joint: -1.1, wrist_2_joint: 1.57, wrist_3_joint: 0.0}',
        }.items(),
    )

    # ── 2. Nodes ──────────────────────────────────────────────────────────────
    velocity_bridge_node = Node(
        package='marble_balancer',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )

    marble_sdf_path = os.path.join(pkg_share, 'urdf', 'marble.sdf')
    spawn_marble = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_marble',
        arguments=[
            '-file', marble_sdf_path, 
            '-entity', 'marble',
            '-x', '0.487', '-y', '0.133', '-z', '0.6'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lqr')),
    )

    lqr_node = Node(
        package='marble_balancer',
        executable='lqr_controller',
        name='marble_lqr_controller',
        parameters=[os.path.join(pkg_share, 'config', 'lqr_params.yaml')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_lqr')),
    )

    # ── 3. Delayed Actions (The Stable Sequence) ─────────────────────────────
    
    # 10s delay for Bridge: ensures joint_state_broadcaster is active first.
    bridge_delayed = TimerAction(
        period=10.0,
        actions=[velocity_bridge_node]
    )

    # 15s delay for Marble/LQR: ensures Bridge is holding the robot upright.
    lqr_and_marble_delayed = TimerAction(
        period=15.0,
        actions=[spawn_marble, lqr_node]
    )

    return LaunchDescription([
        ur_type_arg,
        enable_rviz_arg,
        enable_lqr_arg,
        ur_sim_control,
        bridge_delayed,
        lqr_and_marble_delayed,
    ])