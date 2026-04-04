#!/usr/bin/env python3
"""
fk_solver.py
============
ROS 2 node — Forward Kinematics for a 3-DOF Planar Robotic Arm
===============================================================

Subscribes:  /joint_states          (sensor_msgs/JointState)
Publishes:   /end_effector_position  (geometry_msgs/Point)
Broadcasts:  TF frame  computed_end_effector  ← base_link

FK equations (standard DH for planar arm in the XY plane):
  x = L1*cos(θ1) + L2*cos(θ1+θ2) + L3*cos(θ1+θ2+θ3)
  y = L1*sin(θ1) + L2*sin(θ1+θ2) + L3*sin(θ1+θ2+θ3)

Expected at all joints = 0: x = L1+L2+L3 = 0.9 m, y = 0 m
"""

import math
from typing import Dict

import rclpy
from rclpy.node   import Node
from rclpy.qos    import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg      import JointState
from geometry_msgs.msg    import Point, TransformStamped

import tf2_ros


# ---------------------------------------------------------------------------

class FKSolverNode(Node):
    """Forward kinematics node — computes & publishes EE pose from joint angles."""

    # Ordered joint names expected in /joint_states
    JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3')

    def __init__(self) -> None:
        super().__init__('fk_solver')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('l1',          0.4)
        self.declare_parameter('l2',          0.3)
        self.declare_parameter('l3',          0.2)
        self.declare_parameter('base_frame',  'base_link')
        self.declare_parameter('ee_frame',    'computed_end_effector')
        self.declare_parameter('publish_hz',  10.0)
        self.declare_parameter('verbose',     True)

        self._L1         = self.get_parameter('l1').value
        self._L2         = self.get_parameter('l2').value
        self._L3         = self.get_parameter('l3').value
        self._base_frame = self.get_parameter('base_frame').value
        self._ee_frame   = self.get_parameter('ee_frame').value
        self._verbose    = self.get_parameter('verbose').value

        # Latest joint angles — default to 0 rad each
        self._angles: Dict[str, float] = {j: 0.0 for j in self.JOINT_NAMES}
        self._data_received = False

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 5,
        )

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            sensor_qos,
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self._ee_pub = self.create_publisher(
            Point,
            '/end_effector_position',
            10,
        )

        # ── TF broadcaster ───────────────────────────────────────────────────
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Timer (publish at fixed rate even if joint_states lags) ──────────
        hz = self.get_parameter('publish_hz').value
        self._timer = self.create_timer(1.0 / hz, self._publish_fk)

        self.get_logger().info(
            f'FK Solver ready  L1={self._L1} L2={self._L2} L3={self._L3}  '
            f'base={self._base_frame}  ee={self._ee_frame}  @{hz:.0f} Hz'
        )

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState) -> None:
        """Parse /joint_states and cache the three arm joint angles."""
        for name, pos in zip(msg.name, msg.position):
            if name in self._angles:
                self._angles[name] = float(pos)
        self._data_received = True

    def _publish_fk(self) -> None:
        """Compute FK, publish Point, broadcast TF."""
        th1, th2, th3 = (
            self._angles['joint_1'],
            self._angles['joint_2'],
            self._angles['joint_3'],
        )

        # ── FK equations ─────────────────────────────────────────────────────
        a1 = th1
        a2 = th1 + th2
        a3 = th1 + th2 + th3

        x = self._L1 * math.cos(a1) + self._L2 * math.cos(a2) + self._L3 * math.cos(a3)
        y = self._L1 * math.sin(a1) + self._L2 * math.sin(a2) + self._L3 * math.sin(a3)
        z = 0.0  # planar arm — always in XY plane

        # ── Publish Point ─────────────────────────────────────────────────────
        pt = Point(x=x, y=y, z=z)
        self._ee_pub.publish(pt)

        # ── Broadcast TF ─────────────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp          = self.get_clock().now().to_msg()
        tf_msg.header.frame_id       = self._base_frame
        tf_msg.child_frame_id        = self._ee_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        # End-effector orientation: rotate by (θ1+θ2+θ3) about Z
        yaw = a3
        tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(tf_msg)

        # ── Console output ────────────────────────────────────────────────────
        if self._verbose:
            self.get_logger().info(
                f'[FK]  θ1={math.degrees(th1):+7.2f}°  '
                f'θ2={math.degrees(th2):+7.2f}°  '
                f'θ3={math.degrees(th3):+7.2f}°  '
                f'→  x={x:+.4f} m   y={y:+.4f} m'
            )


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()