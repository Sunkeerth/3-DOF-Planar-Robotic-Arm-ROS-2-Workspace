#!/usr/bin/env python3
"""
ik_solver.py
============
ROS 2 service server — Inverse Kinematics for a 3-DOF Planar Arm
=================================================================

What this node does:
  - Provides a service called '/compute_ik' (using the custom interface 'ComputeIK').
  - When someone calls the service with a target (x, y) position,
    it calculates the joint angles (θ1, θ2, θ3) needed to reach that point.
  - The IK solution assumes θ3 = 0 (the third link does not bend), so we only solve for θ1 and θ2.
  - Returns success/failure, the joint angles, and a human‑readable message.

Geometric IK method (θ3 = 0):
  1. Find the direction (angle φ) from base to target.
  2. Subtract the fixed L3 length along that direction to get the wrist position.
  3. Solve the 2‑link arm (L1, L2) to reach that wrist position using the law of cosines.
  4. θ2 = acos( (r² - L1² - L2²) / (2·L1·L2) )   (elbow‑up solution)
  5. θ1 = atan2(wy, wx) - atan2(L2·sinθ2, L1 + L2·cosθ2)
  6. θ3 = 0

Reachability condition: |L1 - L2| ≤ distance to wrist ≤ L1 + L2
"""

import math                     # For trigonometric functions and sqrt

import rclpy                    # ROS 2 Python client library
from rclpy.node import Node     # Base class for ROS 2 nodes

# Import the custom service definition (generated from ComputeIK.srv)
from planar_arm_interfaces.srv import ComputeIK


# ── Constants for numerical stability ────────────────────────────────────────────────
_EPS   = 1e-9          # Anything smaller than this is treated as zero
_CLAMP = 1.0 - 1e-7    # Clamp acos argument to stay inside [-1, 1] (avoids math domain error)


# ---------------------------------------------------------------------------
# Main node class
# ---------------------------------------------------------------------------

class IKSolverNode(Node):
    """
    Inverse Kinematics service node.
    It listens for service calls on '/compute_ik' and responds with joint angles.
    """

    def __init__(self) -> None:
        """Constructor: declares parameters and creates the service."""
        super().__init__('ik_solver')   # Node name

        # ── Declare ROS parameters (can be overridden from launch file) ──
        self.declare_parameter('l1', 0.4)   # Length of link 1 (metres)
        self.declare_parameter('l2', 0.3)   # Length of link 2 (metres)
        self.declare_parameter('l3', 0.2)   # Length of link 3 (metres)

        # Store parameter values in instance variables
        self._L1 = self.get_parameter('l1').value
        self._L2 = self.get_parameter('l2').value
        self._L3 = self.get_parameter('l3').value

        # ── Create the service ──
        # Service type: ComputeIK
        # Service name: '/compute_ik'
        # Callback function: self._ik_callback (called whenever a request arrives)
        self._srv = self.create_service(
            ComputeIK,
            '/compute_ik',
            self._ik_callback,
        )

        # Print a startup message so the user knows the node is ready
        self.get_logger().info(
            f'IK Solver ready on /compute_ik  '
            f'L1={self._L1}  L2={self._L2}  L3={self._L3}'
        )

    # ── Service callback: this function runs every time a request is received ──
    def _ik_callback(self, request: ComputeIK.Request,
                     response: ComputeIK.Response) -> ComputeIK.Response:
        """
        Solve inverse kinematics for the given target (x, y).
        Fills the response with success flag, message, and joint angles.
        """

        # Get target coordinates from the request
        tx = request.target_x
        ty = request.target_y
        self.get_logger().info(f'[IK]  Request → target_x={tx:.4f}  target_y={ty:.4f}')

        # ── Special case: target is at the origin (0, 0) ──
        # At the origin, the arm can be in many configurations (singular).
        # We return the simplest: all joints at 0 (arm fully extended to the right).
        if abs(tx) < _EPS and abs(ty) < _EPS:
            response.success = True
            response.message = (
                'WARNING: Target is at origin — degenerate configuration. '
                'Returning zero-angle solution (all joints = 0).'
            )
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.theta3 = 0.0
            self.get_logger().warn(response.message)   # Print a warning
            return response

        # ── Because θ3 is fixed to 0, the end‑effector is offset from the wrist ──
        # The wrist is the point where link 2 ends and link 3 begins.
        # We first find the direction from base to target.
        phi = math.atan2(ty, tx)   # angle of the target from base (in radians)

        # Subtract the fixed L3 length along that direction to get the wrist position.
        wx = tx - self._L3 * math.cos(phi)   # wrist x coordinate
        wy = ty - self._L3 * math.sin(phi)   # wrist y coordinate

        # Distance from base to wrist (the centre of the wrist circle)
        r2 = wx * wx + wy * wy
        r  = math.sqrt(r2)

        # ── Check if the wrist is within the reachable range of the 2‑link arm (L1, L2) ──
        r_max = self._L1 + self._L2          # maximum reach (arm fully extended)
        r_min = abs(self._L1 - self._L2)     # minimum reach (arm folded onto itself)

        if r > r_max + _EPS:
            # Target is too far away
            response.success = False
            response.message = (
                f'Target ({tx:.3f}, {ty:.3f}) is UNREACHABLE — '
                f'distance to wrist-centre {r:.4f} m exceeds max reach {r_max:.4f} m.'
            )
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.theta3 = 0.0
            self.get_logger().warn(response.message)
            return response

        if r < r_min - _EPS:
            # Target is too close (the arm cannot fold enough)
            response.success = False
            response.message = (
                f'Target ({tx:.3f}, {ty:.3f}) is UNREACHABLE — '
                f'distance to wrist-centre {r:.4f} m is below min reach {r_min:.4f} m '
                f'(arm cannot fold that tightly).'
            )
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.theta3 = 0.0
            self.get_logger().warn(response.message)
            return response

        # ── Solve for θ2 using the law of cosines ──
        # cosθ2 = (r² - L1² - L2²) / (2·L1·L2)
        cos_th2 = (r2 - self._L1**2 - self._L2**2) / (2.0 * self._L1 * self._L2)

        # Clamp the value to [-1, 1] to avoid floating‑point errors near ±1
        cos_th2 = max(-_CLAMP, min(_CLAMP, cos_th2))

        # Choose the "elbow up" solution (θ2 positive). acos returns an angle in [0, π].
        th2 = math.acos(cos_th2)

        # Check for near‑singularity (arm almost fully extended or folded)
        singularity_warning = ''
        if abs(abs(cos_th2) - 1.0) < 1e-4:
            singularity_warning = (
                ' [NEAR SINGULARITY — arm nearly fully extended/folded. '
                'Solution may be ill-conditioned.]'
            )
            self.get_logger().warn(f'[IK] Near singularity at target ({tx},{ty})')

        # ── Solve for θ1 ──
        # Using the formula: θ1 = atan2(wy, wx) - atan2(L2·sinθ2, L1 + L2·cosθ2)
        k1 = self._L1 + self._L2 * math.cos(th2)   # denominator for the second angle
        k2 = self._L2 * math.sin(th2)              # numerator
        th1 = math.atan2(wy, wx) - math.atan2(k2, k1)

        # Normalise θ1 to the range (-π, π] (optional, but keeps angles tidy)
        th1 = math.atan2(math.sin(th1), math.cos(th1))

        # θ3 is fixed to 0 (as per the problem statement)
        th3 = 0.0

        # ── Validation: use forward kinematics to check the solution ──
        # Compute the end‑effector position from the angles we just found.
        x_check = (self._L1 * math.cos(th1)
                   + self._L2 * math.cos(th1 + th2)
                   + self._L3 * math.cos(th1 + th2 + th3))
        y_check = (self._L1 * math.sin(th1)
                   + self._L2 * math.sin(th1 + th2)
                   + self._L3 * math.sin(th1 + th2 + th3))
        residual = math.hypot(x_check - tx, y_check - ty)   # Euclidean distance error

        # If the error is larger than 1 mm, reject the solution (safety check)
        if residual > 1e-3:
            response.success = False
            response.message = (
                f'IK solution did not verify (residual={residual:.6f} m). '
                'This target may be on the workspace boundary.'
            )
            response.theta1 = th1
            response.theta2 = th2
            response.theta3 = th3
            return response

        # ── Success: fill the response with the computed angles ──
        response.success = True
        response.message = (
            f'Solution found  θ1={math.degrees(th1):+.2f}°  '
            f'θ2={math.degrees(th2):+.2f}°  θ3=0.00°  '
            f'(FK residual={residual:.6f} m){singularity_warning}'
        )
        response.theta1 = th1
        response.theta2 = th2
        response.theta3 = th3

        self.get_logger().info(f'[IK]  {response.message}')
        return response


# ---------------------------------------------------------------------------
# Main function: entry point when the script is run directly
# ---------------------------------------------------------------------------

def main(args=None):
    """
    Initialises the ROS 2 system, creates the node, and keeps it spinning
    (i.e., waiting for service calls) until Ctrl+C is pressed.
    """
    rclpy.init(args=args)                # Start ROS 2 communication
    node = IKSolverNode()                # Create an instance of our node
    try:
        rclpy.spin(node)                 # Keep the node alive and processing callbacks
    except KeyboardInterrupt:            # When user presses Ctrl+C
        pass
    finally:
        node.destroy_node()              # Clean up
        rclpy.shutdown()                 # Shut down ROS 2


if __name__ == '__main__':
    main()