"""
display.launch.py
=================
Launch file for the 3-DOF Planar Arm (Task 1).

Starts:
  • robot_state_publisher   — publishes TF from URDF
  • joint_state_publisher_gui — interactive joint sliders
  • fk_solver               — FK computation node  (optional, default ON)
  • ik_solver               — IK service node      (optional, default ON)
  • rviz2                   — 3-D visualiser        (optional, default ON)

Usage:
  ros2 launch planar_arm_description display.launch.py
  ros2 launch planar_arm_description display.launch.py launch_rviz:=false
  ros2 launch planar_arm_description display.launch.py launch_ik:=false launch_fk:=false
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _resolve_xacro(context, *args, **kwargs):
    """Expand the xacro file to a URDF string at launch time."""
    pkg = get_package_share_directory('planar_arm_description')
    xacro_file = os.path.join(pkg, 'urdf', 'planar_arm.urdf.xacro')
    robot_description = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')
    return robot_description


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------

def generate_launch_description():

    # ── Package paths ───────────────────────────────────────────────────────
    pkg_desc   = get_package_share_directory('planar_arm_description')
    rviz_cfg   = os.path.join(pkg_desc, 'rviz', 'planar_arm.rviz')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'planar_arm.urdf.xacro')

    # ── Resolve xacro → URDF once at launch time ────────────────────────────
    robot_description_content = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')

    robot_description = {'robot_description': robot_description_content}

    # ── Launch arguments ────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock topic (Gazebo / sim)')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Start RViz2 visualiser')

    launch_fk_arg = DeclareLaunchArgument(
        'launch_fk',
        default_value='true',
        description='Start FK solver node')

    launch_ik_arg = DeclareLaunchArgument(
        'launch_ik',
        default_value='true',
        description='Start IK solver service')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    launch_rviz    = LaunchConfiguration('launch_rviz')
    launch_fk      = LaunchConfiguration('launch_fk')
    launch_ik      = LaunchConfiguration('launch_ik')

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name       = 'robot_state_publisher',
        output     = 'screen',
        parameters = [
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── joint_state_publisher_gui ───────────────────────────────────────────
    joint_state_publisher_gui = Node(
        package    = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name       = 'joint_state_publisher_gui',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── fk_solver ───────────────────────────────────────────────────────────
    fk_solver = Node(
        package    = 'planar_arm_kinematics',
        executable = 'fk_solver',
        name       = 'fk_solver',
        output     = 'screen',
        parameters = [
            {
                'l1': 0.4,
                'l2': 0.3,
                'l3': 0.2,
                'base_frame': 'base_link',
                'ee_frame'  : 'computed_end_effector',
                'publish_hz': 10.0,
                'verbose'   : True,
                'use_sim_time': use_sim_time,
            }
        ],
        condition = IfCondition(launch_fk),
    )

    # ── ik_solver ───────────────────────────────────────────────────────────
    ik_solver = Node(
        package    = 'planar_arm_kinematics',
        executable = 'ik_solver',
        name       = 'ik_solver',
        output     = 'screen',
        parameters = [
            {
                'l1': 0.4,
                'l2': 0.3,
                'l3': 0.2,
                'use_sim_time': use_sim_time,
            }
        ],
        condition = IfCondition(launch_ik),
    )

    # ── rviz2 ───────────────────────────────────────────────────────────────
    rviz2 = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rviz_cfg],
        parameters = [{'use_sim_time': use_sim_time}],
        condition  = IfCondition(launch_rviz),
    )

    # ── Assembly ────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments first
        use_sim_time_arg,
        launch_rviz_arg,
        launch_fk_arg,
        launch_ik_arg,

        # Banner
        LogInfo(msg='╔══════════════════════════════════════╗'),
        LogInfo(msg='║  Planar Arm 3-DOF  —  launching...   ║'),
        LogInfo(msg='╚══════════════════════════════════════╝'),

        # Core nodes
        robot_state_publisher,
        joint_state_publisher_gui,

        # Optional nodes
        fk_solver,
        ik_solver,
        rviz2,
    ])