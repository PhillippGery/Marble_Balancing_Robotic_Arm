import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
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
    record_arg = DeclareLaunchArgument(
        'plot', default_value='false',
        description='Set to true to record marble data and plot after shutdown')
    lissajous_arg = DeclareLaunchArgument(
        'lissajous', default_value='false',
        description='Set to true to drive the marble along a Lissajous curve')
    tcp_lissajous_arg = DeclareLaunchArgument(
        'tcp_lissajous', default_value='false',
        description='Set to true to move the TCP along a Lissajous curve while balancing')
    rl_arg = DeclareLaunchArgument(
        'rl', default_value='false',
        description='Set to true to enable the SAC residual controller')
    rl_model_arg = DeclareLaunchArgument(
        'rl_model', default_value='',
        description='Path to trained SAC model .zip file')
    rl_norm_arg = DeclareLaunchArgument(
        'rl_norm', default_value='',
        description='Path to VecNormalize stats .pkl file')
    rl_stage_arg = DeclareLaunchArgument(
        'rl_stage', default_value='3',
        description='Curriculum stage (0-3) controlling residual clip budget')

    pkg_marble = get_package_share_directory('marble_balancer')
    pkg_moveit = get_package_share_directory('ur_moveit_config')

    # ── 1. Robot description via xacro ────────────────────────────────────────
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
        ]),
        value_type=str,
    )

    # ── 2. Gazebo + UR5e + controllers ────────────────────────────────────────
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gazebo'),
            '/launch/ur_sim_control.launch.py',
        ]),
        launch_arguments={
            'ur_type':                  'ur5e',
            'launch_rviz':              'false',
            'description_package':      'marble_balancer',
            'description_file':         'ur5e_marble_balancer.urdf.xacro',
            'runtime_config_package':   'marble_balancer',
            'controllers_file':         'ur_controllers.yaml',
            'initial_joint_controller': 'ur7e_arm_controller',
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

    # ── 5. go_to_pose: homing move ────────────────────────────────────────────
    # Waits internally for /compute_ik + /joint_states, moves robot, then exits.
    go_to_pose_node = Node(
        package='marble_balancer',
        executable='go_to_pose',
        name='go_to_pose',
        output='screen',
    )

    # ── 6. Marble spawner ─────────────────────────────────────────────────────
    # Reads world → tool0 TF, adds drop offset, calls /spawn_entity service.
    # No hardcoded coordinates — always drops above the actual TCP position.
    marble_spawn = Node(
        package='marble_balancer',
        executable='marble_spawner',
        name='marble_spawner',
        output='screen',
    )

    # ── 7. LQR marble balancing controller ───────────────────────────────────
    # Output remapped so mux_controller sits between it and servo_node.
    pilot_node = Node(
        package='marble_balancer',
        executable='marble_servo_controller',
        name='marble_servo_controller',
        remappings=[('/servo_node/delta_twist_cmds', '/marble_servo/delta_twist_cmds')],
        output='screen',
    )

    # ── 7b. Mux: manual commands take priority over LQR ──────────────────────
    # Listens on /manual/delta_twist_cmds; falls back to LQR after manual_timeout.
    mux_node = Node(
        package='marble_balancer',
        executable='mux_controller',
        name='mux_controller',
        parameters=[{'manual_timeout': 0.5, 'publish_rate': 30.0}],
        output='screen',
    )

    # ── 8. Data recorder/plotter (optional, starts with marble spawn) ─────────
    plotter_node = Node(
        package='marble_balancer',
        executable='marble_plotter',
        name='marble_plotter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('plot')),
    )

    # ── 9. Lissajous setpoint node (optional) ────────────────────────────────
    # Publishes desired marble position on /marble/desired_pos.
    # Controller always subscribes; when this node is not running, desired = (0,0).
    lissajous_node = Node(
        package='marble_balancer',
        executable='marble_lissajous',
        name='marble_lissajous',
        output='screen',
        condition=IfCondition(LaunchConfiguration('lissajous')),
    )

    # ── 10. TCP Lissajous node (optional) ─────────────────────────────────────
    # Moves the robot TCP along a Lissajous curve in XY while marble stays balanced.
    # Publishes TCP linear velocity + feedforward tilt; activates after marble lands.
    tcp_lissajous_node = Node(
        package='marble_balancer',
        executable='tcp_lissajous',
        name='tcp_lissajous_node',
        parameters=[{
            'amplitude_x':  0.30, #was 0.4 both
            'amplitude_y':  0.30,
            'period':       12.0,
            'fa':           1,
            'fb':           2,
            'delta':        1.5707963,   # pi/2
            'ff_gain':      0.0,   # start at 0 (disabled); try -1.0 if oscillation worsens with 1.0
            'publish_rate': 30.0,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('tcp_lissajous')),
    )

    # ── Event-driven sequencing ───────────────────────────────────────────────
    #
    #  ur_sim ──► move_group + servo_node
    #                  │
    #             OnProcessStart
    #                  ▼
    #             go_to_pose  (waits for /compute_ik + /joint_states, exits)
    #                  │
    #             OnProcessExit  (+6 s settle)
    #                  ▼
    #             marble_spawner  (reads TF → drops marble → exits)
    #                  │
    #             OnProcessExit
    #                  ▼
    #             pilot_node  (waits internally for marble to land, then LQR)

    go_to_pose_on_moveit_ready = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group_node,
            on_start=[go_to_pose_node],
        )
    )

    on_homed = RegisterEventHandler(
        OnProcessExit(
            target_action=go_to_pose_node,
            on_exit=[TimerAction(period=0.5, actions=[marble_spawn])],
        )
    )

    # ── 11. SAC residual controller (optional) ────────────────────────────────
    # Subscribes to /marble/lqr_state, adds RL residual, publishes to
    # /marble_servo_rl/delta_twist_cmds. mux_controller auto_topic is remapped
    # to /marble_servo_rl/delta_twist_cmds when rl:=true.
    rl_residual_node = Node(
        package='marble_balancer',
        executable='rl_residual',
        name='rl_residual_node',
        parameters=[{
            'rl_model': LaunchConfiguration('rl_model'),
            'rl_norm':  LaunchConfiguration('rl_norm'),
            'rl_stage': LaunchConfiguration('rl_stage'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rl')),
    )

    # When RL is active, mux listens to RL output instead of raw LQR
    mux_node_rl = Node(
        package='marble_balancer',
        executable='mux_controller',
        name='mux_controller',
        parameters=[{
            'manual_timeout': 0.5,
            'publish_rate':   30.0,
            'auto_topic':     '/marble_servo_rl/delta_twist_cmds',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rl')),
    )

    # ── 12. Real-time 2D marble position visualizer ───────────────────────────
    visualizer_node = Node(
        package='marble_balancer',
        executable='marble_visualizer',
        name='marble_visualizer',
        parameters=[{'trail_length': 200, 'update_rate': 20.0}],
        output='screen',
    )

    on_marble_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=marble_spawn,
            on_exit=[pilot_node, mux_node, mux_node_rl,
                     rl_residual_node, visualizer_node,
                     plotter_node, lissajous_node, tcp_lissajous_node],
        )
    )

    return LaunchDescription([
        record_arg,
        lissajous_arg,
        tcp_lissajous_arg,
        rl_arg,
        rl_model_arg,
        rl_norm_arg,
        rl_stage_arg,
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose_on_moveit_ready,
        on_homed,
        on_marble_spawned,
    ])
