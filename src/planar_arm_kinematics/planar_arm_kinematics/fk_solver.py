#!/usr/bin/env python3
"""
fk_solver.py
============
ROS 2 node — Forward Kinematics for a 3-DOF Planar Robotic Arm
===============================================================

What this node does:
  - Subscribes to the '/joint_states' topic (which contains the current joint angles).
  - Uses the forward kinematics (FK) equations to calculate where the end‑effector (tool tip) is.
  - Publishes the end‑effector position on the '/end_effector_position' topic.
  - Broadcasts a TF transform from 'base_link' to 'computed_end_effector' so that RViz can show it.

FK equations (for a planar arm moving in the XY plane):
  x = L1*cos(θ1) + L2*cos(θ1+θ2) + L3*cos(θ1+θ2+θ3)
  y = L1*sin(θ1) + L2*sin(θ1+θ2) + L3*sin(θ1+θ2+θ3)

Expected result when all joint angles are 0°:
  x = L1 + L2 + L3 = 0.4 + 0.3 + 0.2 = 0.9 m,  y = 0 m
"""

# Import required Python and ROS 2 modules
import math                     # For cos, sin, degrees, etc.
from typing import Dict         # For type hints (dictionary)

import rclpy                    # Main ROS 2 Python client library
from rclpy.node import Node     # Base class for all ROS 2 nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # For quality of service settings

# ROS 2 message types
from sensor_msgs.msg import JointState      # Contains joint names and positions
from geometry_msgs.msg import Point, TransformStamped   # For position and TF messages

import tf2_ros                  # For broadcasting transforms (TF)


# ---------------------------------------------------------------------------
# Main node class
# ---------------------------------------------------------------------------

class FKSolverNode(Node):
    """
    Forward Kinematics node.
    It subscribes to joint angles, computes end‑effector pose, and publishes it.
    """

    # These are the names of the joints we care about (must match the URDF)
    JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')

    def __init__(self) -> None:
        """Constructor: sets up the node, parameters, subscribers, publishers, and timer."""
        super().__init__('fk_solver')   # Name of the node

        # ── Declare and read ROS parameters (can be set from launch file or command line) ──
        self.declare_parameter('l1',          0.4)      # Length of link 1 (metres)
        self.declare_parameter('l2',          0.3)      # Length of link 2 (metres)
        self.declare_parameter('l3',          0.2)      # Length of link 3 (metres)
        self.declare_parameter('base_frame',  'base_link')   # Name of the fixed frame (robot base)
        self.declare_parameter('ee_frame',    'computed_end_effector')  # Name of the frame we broadcast
        self.declare_parameter('publish_hz',  10.0)     # How many times per second to compute and publish
        self.declare_parameter('verbose',     True)     # If True, print FK results to the terminal

        # Store parameter values in instance variables (with self.)
        self._L1         = self.get_parameter('l1').value
        self._L2         = self.get_parameter('l2').value
        self._L3         = self.get_parameter('l3').value
        self._base_frame = self.get_parameter('base_frame').value
        self._ee_frame   = self.get_parameter('ee_frame').value
        self._verbose    = self.get_parameter('verbose').value

        # Latest joint angles: start with all zeros (so the arm is straight right)
        self._angles: Dict[str, float] = {j: 0.0 for j in self.JOINT_NAMES}
        self._data_received = False   # Not used further, but could be used to wait for first message

        # ── Quality of Service (QoS) for the joint_states subscriber ──
        # We use BEST_EFFORT because joint_state_publisher_gui publishes fast and we don't need perfect reliability.
        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
        )

        # ── Subscriber: listens to '/joint_states' ──
        # When a new JointState message arrives, _joint_state_cb is called.
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            sensor_qos,
        )

        # ── Publisher: sends the end‑effector position as a Point message ──
        # Topic name: '/end_effector_position'
        self._ee_pub = self.create_publisher(
            Point,
            '/end_effector_position',
            10,   # queue size
        )

        # ── TF broadcaster: allows us to send transforms to the TF tree ──
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Timer: calls _publish_fk() at a fixed rate (e.g., 10 Hz) ──
        # This ensures we publish even if joint_states messages are slow.
        hz = self.get_parameter('publish_hz').value
        self._timer = self.create_timer(1.0 / hz, self._publish_fk)

        # Print a startup message so the user knows the node is ready
        self.get_logger().info(
            f'FK Solver ready  L1={self._L1} L2={self._L2} L3={self._L3}  '
            f'base={self._base_frame}  ee={self._ee_frame}  @{hz:.0f} Hz'
        )

    # ── Callback for joint_states subscription ──
    def _joint_state_cb(self, msg: JointState) -> None:
        """
        Called every time a JointState message arrives.
        It extracts the positions of joint_1, joint_2, joint_3 and stores them.
        """
        # msg.name is a list of joint names (e.g., ['joint_1', 'joint_2', ...])
        # msg.position is a list of corresponding angles (in radians)
        for name, pos in zip(msg.name, msg.position):
            if name in self._angles:          # Only care about our three joints
                self._angles[name] = float(pos)   # Store the angle
        self._data_received = True   # (optional flag, not used further)

    # ── Timer callback: does the actual FK calculation and publishing ──
    def _publish_fk(self) -> None:
        """
        Called at regular intervals (e.g., 10 times per second).
        Computes the end‑effector position using FK equations,
        publishes it, and broadcasts a TF frame.
        """
        # Get the latest joint angles (in radians)
        th1 = self._angles['joint_1']
        th2 = self._angles['joint_2']
        th3 = self._angles['joint_3']

        # ── Forward Kinematics equations ──
        # a1 = angle of first link (θ1)
        # a2 = angle of second link (θ1+θ2)
        # a3 = angle of third link (θ1+θ2+θ3)
        a1 = th1
        a2 = th1 + th2
        a3 = th1 + th2 + th3

        # Compute x and y coordinates of the end‑effector
        x = self._L1 * math.cos(a1) + self._L2 * math.cos(a2) + self._L3 * math.cos(a3)
        y = self._L1 * math.sin(a1) + self._L2 * math.sin(a2) + self._L3 * math.sin(a3)
        z = 0.0   # The arm moves only in the XY plane, so Z is always 0

        # ── Publish the end‑effector position as a Point message ──
        pt = Point(x=x, y=y, z=z)
        self._ee_pub.publish(pt)

        # ── Broadcast a transform (TF) from base_link to computed_end_effector ──
        tf_msg = TransformStamped()
        tf_msg.header.stamp          = self.get_clock().now().to_msg()   # Current time
        tf_msg.header.frame_id       = self._base_frame                  # Parent frame: base_link
        tf_msg.child_frame_id        = self._ee_frame                    # Child frame: computed_end_effector
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        # Orientation: the end‑effector frame should be rotated by the total angle (a3) around Z.
        # We convert the angle to a quaternion (z and w components, because rotation is only around Z).
        yaw = a3
        tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
        # (x and y rotation components stay 0)

        self._tf_broadcaster.sendTransform(tf_msg)

        # ── If verbose is True, print the result to the terminal ──
        if self._verbose:
            self.get_logger().info(
                f'[FK]  θ1={math.degrees(th1):+7.2f}°  '
                f'θ2={math.degrees(th2):+7.2f}°  '
                f'θ3={math.degrees(th3):+7.2f}°  '
                f'→  x={x:+.4f} m   y={y:+.4f} m'
            )


# ---------------------------------------------------------------------------
# Main function: entry point when the script is run directly
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Initializes the ROS 2 system, creates the node, and keeps it spinning
    (i.e., running and responding to callbacks) until Ctrl+C is pressed.
    """
    rclpy.init(args=args)                # Start ROS 2 communication
    node = FKSolverNode()                # Create an instance of our node
    try:
        rclpy.spin(node)                 # Keep the node alive and processing callbacks
    except KeyboardInterrupt:            # When user presses Ctrl+C
        pass
    finally:
        node.destroy_node()              # Clean up
        rclpy.shutdown()                 # Shut down ROS 2


# This condition ensures that main() is called when the script is executed directly,
# but not when it is imported as a module.
if __name__ == '__main__':
    main()