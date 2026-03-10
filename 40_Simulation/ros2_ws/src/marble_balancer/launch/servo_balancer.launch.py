import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_marble = get_package_share_directory('marble_balancer')
    pkg_moveit = get_package_share_directory('ur_moveit_config')

    # ── 1. Gazebo + UR5e + controllers (reuse existing launch) ────────────────
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gazebo'),
            '/launch/ur_sim_control.launch.py',
        ]),
        launch_arguments={
            'ur_type':                   'ur5e',
            'launch_rviz':               'false',
            'controllers_file':          os.path.join(pkg_marble, 'config', 'ur_controllers.yaml'),
            'initial_joint_controller':  'ur7e_arm_controller',
        }.items(),
    )

    # ── 2. Build MoveIt Servo parameters ─────────────────────────────────────
    # Load flat YAML and wrap it — this is exactly what the official servo
    # example does and what moveit_servo C++ code (Humble 2.5.x) expects.
    servo_yaml = load_yaml(os.path.join(pkg_marble, 'config', 'servo_params.yaml'))
    servo_params = {'moveit_servo': servo_yaml}

    # Robot model — servo node needs all three to build the planning scene
    with open('/home/Gery/fixed_ur5e.urdf', 'r') as f:
        robot_description_str = f.read()

    with open(os.path.join(pkg_moveit, 'config', 'ur.srdf'), 'r') as f:
        robot_description_semantic_str = f.read()

    kinematics_yaml = load_yaml(os.path.join(pkg_moveit, 'config', 'kinematics.yaml'))

    # ── 3. robot_state_publisher (servo needs TF tree) ────────────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_str}],
        output='screen',
    )

    moveit_params = [
        {'robot_description':            robot_description_str},
        {'robot_description_semantic':   robot_description_semantic_str},
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

    # ── 4a. move_group (provides /compute_ik and planning services) ───────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        parameters=moveit_params,
        output='screen',
    )

    # ── 4b. MoveIt Servo node ─────────────────────────────────────────────────
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            {'robot_description':          robot_description_str},
            {'robot_description_semantic': robot_description_semantic_str},
            {'robot_description_kinematics': kinematics_yaml},
        ],
        output='screen',
    )

    # ── 5. Static Pilot (zero-twist to hold position) ─────────────────────────
    pilot_node = Node(
        package='marble_balancer',
        executable='marble_servo_controller',
        name='marble_servo_controller',
        output='screen',
    )

    # ── Sequencing: Gazebo needs ~15 s, servo needs ~3 s after that ───────────
    servo_delayed = TimerAction(period=15.0, actions=[rsp_node, move_group_node, servo_node])
    pilot_delayed = TimerAction(period=20.0, actions=[pilot_node])

    return LaunchDescription([
        ur_sim,
        servo_delayed,
        pilot_delayed,
    ])
