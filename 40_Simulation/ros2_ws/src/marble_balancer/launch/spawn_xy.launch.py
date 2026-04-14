"""
spawn_xy.launch.py
------------------
Full balancing stack + custom marble spawn position.

Launches the complete servo_balancer stack, but drops the marble at a
user-specified (x, y) plate-frame offset instead of always at centre.

  x / y  are in the PLATE frame:
    (0, 0)   = plate centre
    (+x)     = forward on the plate
    (+y)     = left on the plate
  Clamped automatically to ±0.18 m so the marble stays on the plate.

Usage:
  ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0
  ros2 launch marble_balancer spawn_xy.launch.py x:=-0.15 y:=0.10
  ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.05 plot:=true
  ros2 launch marble_balancer spawn_xy.launch.py x:=0.10 y:=0.0 rl:=true \\
    rl_model:=/path/to/best_model_td3.zip rl_norm:=/path/to/running_stats.pkl

Sequence:
  Sim starts → go_to_pose (arm homes) → marble_spawner_xy (spawns at x,y)
  → controller + mux + optional nodes start
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
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
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='Marble spawn X offset from plate centre, plate frame (m)')
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='Marble spawn Y offset from plate centre, plate frame (m)')
    drop_height_arg = DeclareLaunchArgument(
        'drop_height', default_value='0.05',
        description='Height above plate surface to drop marble from (m)')
    plot_arg = DeclareLaunchArgument(
        'plot', default_value='false',
        description='Record marble data and plot on shutdown')
    lissajous_arg = DeclareLaunchArgument(
        'lissajous', default_value='false',
        description='Drive marble along a Lissajous setpoint curve')
    tcp_lissajous_arg = DeclareLaunchArgument(
        'tcp_lissajous', default_value='false',
        description='Move TCP along a Lissajous curve while balancing')
    rl_arg = DeclareLaunchArgument(
        'rl', default_value='false',
        description='Enable TD3 residual controller')
    rl_model_arg = DeclareLaunchArgument(
        'rl_model', default_value='',
        description='Path to trained TD3 model .zip')
    rl_norm_arg = DeclareLaunchArgument(
        'rl_norm', default_value='',
        description='Path to running_stats.pkl for obs normalisation')
    rl_stage_arg = DeclareLaunchArgument(
        'rl_stage', default_value='2',
        description='TD3 curriculum stage (0-2)')

    pkg_marble = get_package_share_directory('marble_balancer')
    pkg_moveit = get_package_share_directory('ur_moveit_config')

    PLATE_DIAMETER = 0.40

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('marble_balancer'), 'urdf',
                'ur5e_marble_balancer.urdf.xacro',
            ]),
            ' ur_type:=ur5e',
            ' name:=ur',
            ' safety_limits:=true',
            f' plate_diameter:={PLATE_DIAMETER}',
        ]),
        value_type=str,
    )

    # ── Gazebo + UR5e + controllers ───────────────────────────────────────────
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

    # ── MoveIt configs ────────────────────────────────────────────────────────
    with open(os.path.join(pkg_moveit, 'config', 'ur.srdf'), 'r') as f:
        robot_description_semantic_str = f.read()

    kinematics_yaml = load_yaml(
        os.path.join(pkg_moveit, 'config', 'kinematics.yaml'))

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
            {'robot_description':            robot_description_content},
            {'robot_description_semantic':   robot_description_semantic_str},
            {'robot_description_kinematics': kinematics_yaml},
        ],
        output='screen',
    )

    # ── go_to_pose ────────────────────────────────────────────────────────────
    go_to_pose_node = Node(
        package='marble_balancer',
        executable='go_to_pose',
        name='go_to_pose',
        output='screen',
    )

    # ── marble_spawner_xy — spawns at requested (x, y) offset ─────────────────
    spawner_xy_node = Node(
        package='marble_balancer',
        executable='marble_spawner_xy',
        name='marble_spawner_xy',
        output='screen',
        parameters=[{
            'x':           LaunchConfiguration('x'),
            'y':           LaunchConfiguration('y'),
            'drop_height': LaunchConfiguration('drop_height'),
        }],
    )

    # ── LQR controller ────────────────────────────────────────────────────────
    controller_node = Node(
        package='marble_balancer',
        executable='marble_servo_controller',
        name='marble_servo_controller',
        output='screen',
    )

    # ── mux_controller — routes LQR or RL+LQR to servo ───────────────────────
    # When rl:=false → subscribe to /marble_servo/delta_twist_cmds (LQR only)
    # When rl:=true  → subscribe to /marble_servo_rl/delta_twist_cmds (LQR+RL)
    mux_node_lqr = Node(
        package='marble_balancer',
        executable='mux_controller',
        name='mux_controller',
        output='screen',
        parameters=[{'auto_topic': '/marble_servo/delta_twist_cmds'}],
        condition=UnlessCondition(LaunchConfiguration('rl')),
    )
    mux_node_rl = Node(
        package='marble_balancer',
        executable='mux_controller',
        name='mux_controller',
        output='screen',
        parameters=[{'auto_topic': '/marble_servo_rl/delta_twist_cmds'}],
        condition=IfCondition(LaunchConfiguration('rl')),
    )

    # ── TD3 residual node (only when rl:=true) ────────────────────────────────
    rl_residual_node = Node(
        package='marble_balancer',
        executable='rl_residual',
        name='rl_residual_node',
        output='screen',
        parameters=[{
            'rl_model': LaunchConfiguration('rl_model'),
            'rl_norm':  LaunchConfiguration('rl_norm'),
            'rl_stage': LaunchConfiguration('rl_stage'),
        }],
        condition=IfCondition(LaunchConfiguration('rl')),
    )

    # ── Optional nodes ────────────────────────────────────────────────────────
    plotter_node = Node(
        package='marble_balancer',
        executable='marble_plotter',
        name='marble_plotter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('plot')),
    )
    lissajous_node = Node(
        package='marble_balancer',
        executable='marble_lissajous',
        name='marble_lissajous',
        output='screen',
        condition=IfCondition(LaunchConfiguration('lissajous')),
    )
    tcp_lissajous_node = Node(
        package='marble_balancer',
        executable='tcp_lissajous',
        name='tcp_lissajous',
        output='screen',
        condition=IfCondition(LaunchConfiguration('tcp_lissajous')),
    )

    # ── Event-driven sequencing ───────────────────────────────────────────────
    #
    #  move_group starts
    #       │ OnProcessStart
    #       ▼
    #  go_to_pose  (homes arm, then exits)
    #       │ OnProcessExit + 1.0 s settle
    #       ▼
    #  marble_spawner_xy  (spawns at x,y, then exits)
    #       │ OnProcessExit + 0.5 s settle
    #       ▼
    #  controller + mux + optional nodes

    go_to_pose_on_moveit = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[go_to_pose_node],
        )
    )

    spawner_on_homed = RegisterEventHandler(
        OnProcessExit(
            target_action=go_to_pose_node,
            on_exit=[TimerAction(period=1.0, actions=[spawner_xy_node])],
        )
    )

    controller_on_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawner_xy_node,
            on_exit=[
                TimerAction(period=0.5, actions=[
                    controller_node,
                    mux_node_lqr,
                    mux_node_rl,
                    rl_residual_node,
                    plotter_node,
                    lissajous_node,
                    tcp_lissajous_node,
                ]),
            ],
        )
    )

    return LaunchDescription([
        x_arg, y_arg, drop_height_arg,
        plot_arg, lissajous_arg, tcp_lissajous_arg,
        rl_arg, rl_model_arg, rl_norm_arg, rl_stage_arg,
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose_on_moveit,
        spawner_on_homed,
        controller_on_spawned,
    ])
