"""
compare_controllers.launch.py
------------------------------
Infrastructure launcher for model comparison.

This launch file starts:
  1. Gazebo + UR5e + joint controllers
  2. MoveIt move_group
  3. MoveIt Servo node
  4. go_to_pose (arm homing)
  5. compare_controllers.py (model evaluation)

Usage:
  ros2 launch marble_balancer compare_controllers.launch.py model:=models_td3_v2/checkpoint_td3_v2_500000_steps.zip episodes:=3
  ros2 launch marble_balancer compare_controllers.launch.py model:=models_td3_v2/best_model_td3_v2.zip episodes:=10 tcp_lissajous:=true
  ros2 launch marble_balancer compare_controllers.launch.py gui:=true model:=models_td3_v2/best_model_td3_v2.zip episodes:=5

Parameters:
  gui              - Launch Gazebo GUI (default: false for headless)
  model            - Path to model .zip file (REQUIRED)
  episodes         - Number of comparison episodes (default: 5)
  tcp_lissajous    - Enable TCP Lissajous disturbance (default: false)
  spawn_radius     - Marble spawn radius in meters (default: 0.12)
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
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
    # ── Launch arguments ──────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Launch Gazebo GUI (true/false for headless, default: false)')
    
    model_arg = DeclareLaunchArgument(
        'model', default_value='',
        description='Path to RL model .zip file (REQUIRED)')
    
    episodes_arg = DeclareLaunchArgument(
        'episodes', default_value='5',
        description='Number of comparison episodes (default: 5)')
    
    tcp_lissajous_arg = DeclareLaunchArgument(
        'tcp_lissajous', default_value='false',
        description='Enable TCP Lissajous disturbance (default: false)')
    
    spawn_radius_arg = DeclareLaunchArgument(
        'spawn_radius', default_value='0.12',
        description='Marble spawn radius (default: 0.12 m)')

    pkg_marble = get_package_share_directory('marble_balancer')
    pkg_moveit = get_package_share_directory('ur_moveit_config')

    PLATE_DIAMETER = 0.40

    # ── 1. Robot description ──────────────────────────────────────────────────
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

    # ── 2. Gazebo + UR5e + controllers ───────────────────────────────────────
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

    # ── 3. MoveIt configs ─────────────────────────────────────────────────────
    with open(os.path.join(pkg_moveit, 'config', 'ur.srdf'), 'r') as f:
        robot_description_semantic_str = f.read()

    kinematics_yaml = load_yaml(os.path.join(pkg_moveit, 'config', 'kinematics.yaml'))

    moveit_params = [
        {'robot_description':            robot_description_content},
        {'robot_description_semantic':   robot_description_semantic_str},
        {'robot_description_kinematics': kinematics_yaml},
        {'use_sim_time': True},
        {'allow_trajectory_execution': True},
        {'moveit_manage_controllers': False},
        {'planning_scene_monitor_options': {
            'publish_planning_scene':   True,
            'publish_geometry_updates': True,
            'publish_state_updates':    True,
        }},
    ]

    # ── 4a. move_group ────────────────────────────────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        parameters=moveit_params,
        output='screen',
    )

    # ── 4b. MoveIt Servo node ─────────────────────────────────────────────────
    servo_yaml = load_yaml(os.path.join(pkg_marble, 'config', 'servo_params.yaml'))

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            {'moveit_servo': servo_yaml},
            {'robot_description':            robot_description_content},
            {'robot_description_semantic':   robot_description_semantic_str},
            {'robot_description_kinematics': kinematics_yaml},
        ],
        output='screen',
    )

    # ── 5. go_to_pose (initial arm homing) ─────────────────────────────────────
    go_to_pose = ExecuteProcess(
        cmd=['ros2', 'run', 'marble_balancer', 'go_to_pose'],
        output='screen',
    )

    # ── 6. compare_controllers.py (runs after go_to_pose exits) ─────────────────
    # Use environment variable to pass working directory context
    import os
    compare_controllers_cmd = [
        'python3',
        '-c',
        '''import sys; sys.path.insert(0, "/home/Gery/Marble_Balancing_Robotic_Arm/40_Simulation/ros2_ws/src"); '''
        '''from marble_balancer.rl_training.compare_controllers import compare_controllers; '''
        '''import argparse; '''
        '''parser = argparse.ArgumentParser(); '''
        '''parser.add_argument("--model"); parser.add_argument("--episodes"); '''
        '''parser.add_argument("--tcp-lissajous"); parser.add_argument("--spawn-radius"); '''
        '''args = parser.parse_args(); '''
        '''compare_controllers(model_path=args.model, norm_path="models_td3_v2/running_stats_v2.pkl", '''
        '''num_episodes=int(args.episodes), tcp_lissajous=args.tcp_lissajous.lower()=="true", '''
        '''spawn_radius=float(args.spawn_radius))''',
        '--model',          LaunchConfiguration('model'),
        '--episodes',       LaunchConfiguration('episodes'),
        '--tcp-lissajous',  LaunchConfiguration('tcp_lissajous'),
        '--spawn-radius',   LaunchConfiguration('spawn_radius'),
    ]

    compare_controllers = ExecuteProcess(
        cmd=compare_controllers_cmd,
        output='screen',
    )

    # ── Event chain: go_to_pose → compare_controllers ───────────────────────────
    go_to_pose_done = RegisterEventHandler(
        OnProcessExit(
            target_action=go_to_pose,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[compare_controllers]
                )
            ]
        )
    )

    # ── Build launch description ───────────────────────────────────────────────
    ld = LaunchDescription([
        # Declare arguments
        gui_arg,
        model_arg,
        episodes_arg,
        tcp_lissajous_arg,
        spawn_radius_arg,

        # Infrastructure
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose,

        # Event handler for sequencing
        go_to_pose_done,
    ])

    return ld
