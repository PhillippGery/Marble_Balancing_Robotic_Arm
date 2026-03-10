import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
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
    pilot_node = Node(
        package='marble_balancer',
        executable='marble_servo_controller',
        name='marble_servo_controller',
        output='screen',
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
            on_exit=[TimerAction(period=2.0, actions=[marble_spawn])],
        )
    )

    on_marble_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=marble_spawn,
            on_exit=[pilot_node],
        )
    )

    return LaunchDescription([
        ur_sim,
        move_group_node,
        servo_node,
        go_to_pose_on_moveit_ready,
        on_homed,
        on_marble_spawned,
    ])
