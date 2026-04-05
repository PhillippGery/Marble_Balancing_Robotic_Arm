"""
rl_training.launch.py
---------------------
Launches Gazebo + UR5e + MoveIt2 Servo for online TD3 RL training.

Starts ONLY the infrastructure needed by gazebo_rl_env.py:
  - Gazebo + UR5e + controllers  (ur_sim_control_marble.launch.py)
  - move_group
  - servo_node
  - go_to_pose  (initial homing; gazebo_rl_env handles subsequent resets)

Does NOT start:
  - marble_servo_controller  (gazebo_rl_env IS the controller)
  - mux_controller
  - marble_spawner
  - marble_plotter / lissajous / rl_residual_node / marble_visualizer

After go_to_pose exits the training script is started via ExecuteProcess.
Run from the ros2_ws root, with the workspace sourced:

  ros2 launch marble_balancer rl_training.launch.py
  ros2 launch marble_balancer rl_training.launch.py timesteps:=500000 stage:=0
  ros2 launch marble_balancer rl_training.launch.py load:=/abs/path/to/model.zip
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
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    timesteps_arg = DeclareLaunchArgument(
        'timesteps', default_value='500000',
        description='Total TD3 online training steps (default 500 000)')
    stage_arg = DeclareLaunchArgument(
        'stage', default_value='0',
        description='Starting curriculum stage 0-2 (default 0)')
    seed_steps_arg = DeclareLaunchArgument(
        'seed_steps', default_value='20000',
        description='RLPD pre-seeding steps (default 20 000)')
    load_arg = DeclareLaunchArgument(
        'load', default_value='',
        description='Path to existing TD3 model .zip to continue training')
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='Use EKF state from /marble/lqr_state instead of EMA')
    tcp_lissajous_arg = DeclareLaunchArgument(
        'tcp_lissajous', default_value='false',
        description='Move TCP along Lissajous curve during training')
    spawn_radius_arg = DeclareLaunchArgument(
        'spawn_radius', default_value='0.0',
        description='Random marble spawn radius from plate centre (m, 0=centre always)')

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

    # ── 5. go_to_pose: initial homing ─────────────────────────────────────────
    # Subsequent resets are handled internally by gazebo_rl_env.py.
    go_to_pose_node = Node(
        package='marble_balancer',
        executable='go_to_pose',
        name='go_to_pose',
        output='screen',
    )

    # ── 6. Training script (starts after go_to_pose exits + 1 s settle) ──────
    # Resolve rl_training/ directory relative to this launch file's package.
    rl_training_dir = os.path.join(
        os.path.dirname(pkg_marble),   # src/marble_balancer/share → up to share/
        '..', '..', '..',              # up to install/
        '..', 'src', 'marble_balancer', 'rl_training'
    )
    train_script = os.path.abspath(
        os.path.join(pkg_marble, '..', '..', '..', '..', '..', '..', '..', '..', '..', 'xxx'))

    # Use the installed entry-point if registered in setup.py, otherwise fall
    # back to the source path.  We use ros2 run to keep it ROS-context-aware.
    training_process = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('marble_balancer'),
                # Resolve src path relative to share directory:
                # share/marble_balancer → ../../../../src/marble_balancer/rl_training
                '..', '..', '..', '..', 'src', 'marble_balancer',
                'rl_training', 'train_td3_gazebo.py',
            ]),
            '--timesteps',      LaunchConfiguration('timesteps'),
            '--stage',          LaunchConfiguration('stage'),
            '--seed-steps',     LaunchConfiguration('seed_steps'),
            '--load',           LaunchConfiguration('load'),
            '--spawn-radius',   LaunchConfiguration('spawn_radius'),
            '--tcp-lissajous',  LaunchConfiguration('tcp_lissajous'),
        ],
        output='screen',
    )

    # ── Event-driven sequencing ───────────────────────────────────────────────
    #
    #  ur_sim ──► move_group + servo_node
    #                  │
    #             OnProcessStart
    #                  ▼
    #             go_to_pose  (IK → trajectory → exits)
    #                  │
    #             OnProcessExit + TimerAction(1.0 s)
    #                  ▼
    #             train_td3_gazebo.py  (runs for --timesteps steps, then exits)

    go_to_pose_on_moveit_ready = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[go_to_pose_node],
        )
    )

    training_on_homed = RegisterEventHandler(
        OnProcessExit(
            target_action=go_to_pose_node,
            on_exit=[TimerAction(period=1.0, actions=[training_process])],
        )
    )

    return LaunchDescription([
        timesteps_arg,
        stage_arg,
        seed_steps_arg,
        load_arg,
        use_ekf_arg,
        tcp_lissajous_arg,
        spawn_radius_arg,
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose_on_moveit_ready,
        training_on_homed,
    ])
