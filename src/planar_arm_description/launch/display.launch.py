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

# Import required Python modules
import os                # For joining paths (os.path.join)
import subprocess        # To run external commands like 'xacro'

# ROS 2 specific imports for finding package paths
from ament_index_python.packages import get_package_share_directory

# Launch system imports
from launch import LaunchDescription          # The main description returned by the function
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition     # Conditionally include nodes (e.g., only if launch_rviz is true)
from launch.substitutions import LaunchConfiguration  # To access launch arguments inside nodes
from launch_ros.actions import Node           # To declare a ROS 2 node


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _resolve_xacro(context, *args, **kwargs):
    """
    Helper function (currently not used in this launch file, but could be used
    with OpaqueFunction). It expands the xacro file to a URDF string.
    """
    # Get the absolute path to the 'planar_arm_description' package
    pkg = get_package_share_directory('planar_arm_description')
    # Build the full path to the .xacro file
    xacro_file = os.path.join(pkg, 'urdf', 'planar_arm.urdf.xacro')
    # Run the 'xacro' command and capture its output (the URDF as a string)
    robot_description = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')
    return robot_description


# ---------------------------------------------------------------------------
# generate_launch_description
# ---------------------------------------------------------------------------
# This function is the entry point for every ROS 2 launch file.
# It must return a LaunchDescription object.

def generate_launch_description():

    # ── Package paths ───────────────────────────────────────────────────────
    # Get the absolute path to the 'planar_arm_description' package
    pkg_desc   = get_package_share_directory('planar_arm_description')
    # Build the path to the RViz configuration file
    rviz_cfg   = os.path.join(pkg_desc, 'rviz', 'planar_arm.rviz')
    # Build the path to the xacro URDF file
    xacro_file = os.path.join(pkg_desc, 'urdf', 'planar_arm.urdf.xacro')

    # ── Resolve xacro → URDF once at launch time ────────────────────────────
    # Run the xacro command to convert the .xacro file into a plain URDF string.
    # This string is then stored in 'robot_description_content'.
    robot_description_content = subprocess.check_output(
        ['xacro', xacro_file]
    ).decode('utf-8')

    # Wrap the URDF string into a dictionary that will be passed as a ROS parameter.
    robot_description = {'robot_description': robot_description_content}

    # ── Launch arguments (allow user to override defaults) ────────────────────
    # These arguments can be set from the command line, e.g.:
    #   ros2 launch ... use_sim_time:=true launch_rviz:=false

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

    # These LaunchConfiguration objects are used inside nodes to get the actual
    # value of the arguments (e.g., true/false for launch_rviz).
    use_sim_time   = LaunchConfiguration('use_sim_time')
    launch_rviz    = LaunchConfiguration('launch_rviz')
    launch_fk      = LaunchConfiguration('launch_fk')
    launch_ik      = LaunchConfiguration('launch_ik')

    # ── robot_state_publisher ───────────────────────────────────────────────
    # This node reads the URDF and the joint states (from /joint_states) and
    # publishes all transforms (TF) of the robot.
    robot_state_publisher = Node(
        package    = 'robot_state_publisher',   # ROS 2 package name
        executable = 'robot_state_publisher',   # Executable name
        name       = 'robot_state_publisher',   # Node name (optional)
        output     = 'screen',                  # Print logs to terminal
        parameters = [
            robot_description,                  # The URDF string
            {'use_sim_time': use_sim_time},     # Sim time flag
        ],
    )

    # ── joint_state_publisher_gui ───────────────────────────────────────────
    # This node provides a simple GUI window with sliders for each joint.
    # When you move a slider, it publishes the joint angles on /joint_states.
    joint_state_publisher_gui = Node(
        package    = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name       = 'joint_state_publisher_gui',
        output     = 'screen',
        parameters = [{'use_sim_time': use_sim_time}],
    )

    # ── fk_solver ───────────────────────────────────────────────────────────
    # This is our custom forward kinematics node (from planar_arm_kinematics).
    # It subscribes to /joint_states, computes the end-effector position,
    # publishes /end_effector_position, and broadcasts a TF frame.
    fk_solver = Node(
        package    = 'planar_arm_kinematics',
        executable = 'fk_solver',          # Matches entry point in setup.py
        name       = 'fk_solver',
        output     = 'screen',
        parameters = [
            {
                'l1': 0.4,                 # Length of link 1 (m)
                'l2': 0.3,                 # Length of link 2 (m)
                'l3': 0.2,                 # Length of link 3 (m)
                'base_frame': 'base_link', # TF frame of the robot base
                'ee_frame'  : 'computed_end_effector', # TF frame to broadcast
                'publish_hz': 10.0,        # How often to compute and publish
                'verbose'   : True,        # Print FK info to terminal
                'use_sim_time': use_sim_time,
            }
        ],
        condition = IfCondition(launch_fk),  # Start only if launch_fk is true
    )

    # ── ik_solver ───────────────────────────────────────────────────────────
    # This is our custom inverse kinematics service node.
    # It provides the service /compute_ik. When called, it returns joint angles
    # for a given (x, y) target.
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
    # This starts the RViz2 visualisation tool with our pre‑saved configuration.
    # It will display the robot model, TF frames, and any markers.
    rviz2 = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rviz_cfg],   # Load this specific .rviz file
        parameters = [{'use_sim_time': use_sim_time}],
        condition  = IfCondition(launch_rviz),
    )

    # ── Assembly ────────────────────────────────────────────────────────────
    # The LaunchDescription object contains all the actions that will be executed
    # when the launch file is run. Order matters: arguments first, then nodes.
    return LaunchDescription([
        # Arguments first (so they can be used by the nodes)
        use_sim_time_arg,
        launch_rviz_arg,
        launch_fk_arg,
        launch_ik_arg,

        # Banner messages printed to the terminal
        LogInfo(msg='╔══════════════════════════════════════╗'),
        LogInfo(msg='║  Planar Arm 3-DOF  —  launching...   ║'),
        LogInfo(msg='╚══════════════════════════════════════╝'),

        # Core nodes (always started)
        robot_state_publisher,
        joint_state_publisher_gui,

        # Optional nodes (started only if their launch argument is true)
        fk_solver,
        ik_solver,
        rviz2,
    ])