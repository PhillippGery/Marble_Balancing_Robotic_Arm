"""
compare_rl_vs_lqr.launch.py
---------------------------
Clean, simple comparison launcher.

This launch file:
  1. Starts Gazebo + UR5e + Controllers
  2. Starts MoveIt + Servo
  3. Homes the arm (go_to_pose)
  4. Runs compare_rl_vs_lqr.py with infrastructure ready

Usage:
  ros2 launch marble_balancer compare_rl_vs_lqr.launch.py \
    model:=models_td3_v2/best_model_td3_v2.zip \
    episodes:=5 \
    tcp_lissajous:=false

Parameters:
  model         - RL model .zip file (REQUIRED)
  episodes      - Number of comparison episodes (default: 5)
  tcp_lissajous - TCP Lissajous movement (default: false)
  spawn_radius  - Marble spawn radius (default: 0.12)
  gui           - Show Gazebo GUI (default: false)
"""

import os
import yaml
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Arguments
    model_arg = DeclareLaunchArgument(
        'model', default_value='',
        description='Path to RL model .zip (REQUIRED)')
    
    episodes_arg = DeclareLaunchArgument(
        'episodes', default_value='5',
        description='Number of episodes (default: 5)')
    
    tcp_lissajous_arg = DeclareLaunchArgument(
        'tcp_lissajous', default_value='false',
        description='TCP Lissajous movement (default: false)')
    
    spawn_radius_arg = DeclareLaunchArgument(
        'spawn_radius', default_value='0.12',
        description='Marble spawn radius (default: 0.12)')
    
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Show Gazebo GUI (default: false)')

    pkg_marble = get_package_share_directory('marble_balancer')
    pkg_moveit = get_package_share_directory('ur_moveit_config')
    PLATE_DIAMETER = 0.40

    # 1. Robot description
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('marble_balancer'),
                'urdf',
                'ur5e_marble_balancer.urdf.xacro',
            ]),
            ' ur_type:=ur5e',
            ' name:=ur',
            ' safety_limits:=true',
            f' plate_diameter:={PLATE_DIAMETER}',
        ]),
        value_type=str,
    )

    # 2. Gazebo + UR5e
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('marble_balancer'),
            '/launch/ur_sim_control_marble.launch.py',
        ]),
        launch_arguments={
            'ur_type':                  'ur5e',
            'launch_rviz':              'false',
            'gazebo_gui':               LaunchConfiguration('gui'),
            'description_package':      'marble_balancer',
            'description_file':         'ur5e_marble_balancer.urdf.xacro',
            'runtime_config_package':   'marble_balancer',
            'controllers_file':         'ur_controllers.yaml',
            'initial_joint_controller': 'ur7e_arm_controller',
            'plate_diameter':           str(PLATE_DIAMETER),
        }.items(),
    )

    # 3. MoveIt configs
    with open(os.path.join(pkg_moveit, 'config', 'ur.srdf'), 'r') as f:
        robot_description_semantic_str = f.read()
    kinematics_yaml = load_yaml(os.path.join(pkg_moveit, 'config', 'kinematics.yaml'))

    moveit_params = [
        {'robot_description': robot_description_content},
        {'robot_description_semantic': robot_description_semantic_str},
        {'robot_description_kinematics': kinematics_yaml},
        {'use_sim_time': True},
        {'allow_trajectory_execution': True},
        {'moveit_manage_controllers': False},
        {'planning_scene_monitor_options': {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
        }},
    ]

    # 4. MoveIt nodes
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        parameters=moveit_params,
        output='screen',
    )

    servo_yaml = load_yaml(os.path.join(pkg_marble, 'config', 'servo_params.yaml'))
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            {'moveit_servo': servo_yaml},
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_str},
            {'robot_description_kinematics': kinematics_yaml},
        ],
        output='screen',
    )

    # 5. Arm homing
    go_to_pose = ExecuteProcess(
        cmd=['ros2', 'run', 'marble_balancer', 'go_to_pose'],
        output='screen',
    )

    # 6. Comparison script
    # Use absolute path: /home/Gery/.../src/marble_balancer/rl_training/compare_rl_vs_lqr.py
    ws_root = '/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws'
    compare_script_path = os.path.join(ws_root, 'src', 'marble_balancer', 
                                        'rl_training', 'compare_rl_vs_lqr.py')
    
    comparison_script = ExecuteProcess(
        cmd=[
            'python3',
            compare_script_path,
            '--model',          LaunchConfiguration('model'),
            '--episodes',       LaunchConfiguration('episodes'),
            '--tcp-lissajous',  LaunchConfiguration('tcp_lissajous'),
            '--spawn-radius',   LaunchConfiguration('spawn_radius'),
        ],
        output='screen',
    )

    # Event: run comparison after go_to_pose completes
    go_to_pose_done = RegisterEventHandler(
        OnProcessExit(
            target_action=go_to_pose,
            on_exit=[
                TimerAction(period=2.0, actions=[comparison_script])
            ]
        )
    )

    # Build launch description
    ld = LaunchDescription([
        model_arg,
        episodes_arg,
        tcp_lissajous_arg,
        spawn_radius_arg,
        gui_arg,
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose,
        go_to_pose_done,
    ])

    return ld
