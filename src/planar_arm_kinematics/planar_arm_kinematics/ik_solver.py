#!/usr/bin/env python3
"""
ik_solver.py
============
ROS 2 service server — Inverse Kinematics for a 3-DOF Planar Arm
=================================================================

Service:  /compute_ik  (planar_arm_interfaces/srv/ComputeIK)

Geometric IK (θ3 = 0 fixed, solve for θ1 and θ2):
─────────────────────────────────────────────────
  effective reach = L1 + L2  (θ3=0 means L3 is folded into "effective target")

  Adjusted target  (subtract L3 contribution):
    x' = target_x − L3·cos(φ)        where φ is the approach angle
    y' = target_y − L3·sin(φ)

  Because θ3=0 the wrist angle equals the cumulative arm angle.
  We solve the 2-link sub-problem (L1, L2) to the adjusted target:

  cos(θ2) = (x'² + y'² − L1² − L2²) / (2·L1·L2)   ← law of cosines
  θ2       = ±arccos(…)  — we return the elbow-up solution

  k1 = L1 + L2·cos(θ2)
  k2 = L2·sin(θ2)
  θ1 = arctan2(y', x') − arctan2(k2, k1)

  θ3 = 0.0

Singularity handling:
  • Fully extended / fully folded (|cos θ2| ≈ 1) → θ1 degenerates, we clamp
  • Origin (0,0) → warn and return θ1=0, θ2=0, θ3=0 (arm fully extended)
  • Outside workspace → return success=False

Reachability condition (elbow workspace):
  |L1 − L2| ≤ √(x'² + y'²) ≤ L1 + L2
"""

import math

import rclpy
from rclpy.node import Node

from planar_arm_interfaces.srv import ComputeIK


# ── Constants ────────────────────────────────────────────────────────────────
_EPS   = 1e-9   # numerical zero
_CLAMP = 1.0 - 1e-7   # safe clamp for acos argument


# ---------------------------------------------------------------------------

class IKSolverNode(Node):

    def __init__(self) -> None:
        super().__init__('ik_solver')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('l1', 0.4)
        self.declare_parameter('l2', 0.3)
        self.declare_parameter('l3', 0.2)

        self._L1 = self.get_parameter('l1').value
        self._L2 = self.get_parameter('l2').value
        self._L3 = self.get_parameter('l3').value

        # ── Service ──────────────────────────────────────────────────────────
        self._srv = self.create_service(
            ComputeIK,
            '/compute_ik',
            self._ik_callback,
        )

        self.get_logger().info(
            f'IK Solver ready on /compute_ik  '
            f'L1={self._L1}  L2={self._L2}  L3={self._L3}'
        )

    # ── Service callback ─────────────────────────────────────────────────────

    def _ik_callback(self, request: ComputeIK.Request,
                     response: ComputeIK.Response) -> ComputeIK.Response:

        tx, ty = request.target_x, request.target_y
        self.get_logger().info(f'[IK]  Request → target_x={tx:.4f}  target_y={ty:.4f}')

        # ── Degenerate case: origin ───────────────────────────────────────────
        if abs(tx) < _EPS and abs(ty) < _EPS:
            # Arm fully folded — multiple solutions; return zero-config
            response.success = True
            response.message = (
                'WARNING: Target is at origin — degenerate configuration. '
                'Returning zero-angle solution (all joints = 0).'
            )
            response.theta1 = 0.0
            response.theta2 = 0.0
            response.theta3 = 0.0
            self.get_logger().warn(response.message)
            return response

        # ── θ3 = 0 — treat the 3-link arm as a 2-link arm to a shifted target
        #    With θ3=0, the EE is offset by L3 along the cumulative wrist angle.
        #    Strategy: solve as if the 3rd link does not bend,
        #    then θ1+θ2 = arctan2(y,x) and we reach backward L3.
        # ─────────────────────────────────────────────────────────────────────
        # Approach angle: direction to the target
        phi = math.atan2(ty, tx)

        # Adjusted wrist target (subtract L3 from the tool-point)
        wx = tx - self._L3 * math.cos(phi)
        wy = ty - self._L3 * math.sin(phi)

        r2 = wx * wx + wy * wy
        r  = math.sqrt(r2)

        # ── Reachability (2-link sub-arm) ─────────────────────────────────────
        r_max = self._L1 + self._L2
        r_min = abs(self._L1 - self._L2)

        if r > r_max + _EPS:
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

        # ── Solve θ2 (law of cosines) ─────────────────────────────────────────
        cos_th2 = (r2 - self._L1**2 - self._L2**2) / (2.0 * self._L1 * self._L2)

        # Numerical clamp for floating-point noise near ±1
        cos_th2 = max(-_CLAMP, min(_CLAMP, cos_th2))

        # Elbow-UP solution: θ2 > 0
        th2 = math.acos(cos_th2)

        # Detect near-singularity (arm fully stretched or folded)
        singularity_warning = ''
        if abs(abs(cos_th2) - 1.0) < 1e-4:
            singularity_warning = (
                ' [NEAR SINGULARITY — arm nearly fully extended/folded. '
                'Solution may be ill-conditioned.]'
            )
            self.get_logger().warn(f'[IK] Near singularity at target ({tx},{ty})')

        # ── Solve θ1 ──────────────────────────────────────────────────────────
        k1 = self._L1 + self._L2 * math.cos(th2)
        k2 = self._L2 * math.sin(th2)
        th1 = math.atan2(wy, wx) - math.atan2(k2, k1)

        # Normalise to (−π, π)
        th1 = math.atan2(math.sin(th1), math.cos(th1))

        th3 = 0.0  # per specification

        # ── Validation — forward-check ────────────────────────────────────────
        x_check = (self._L1 * math.cos(th1)
                   + self._L2 * math.cos(th1 + th2)
                   + self._L3 * math.cos(th1 + th2 + th3))
        y_check = (self._L1 * math.sin(th1)
                   + self._L2 * math.sin(th1 + th2)
                   + self._L3 * math.sin(th1 + th2 + th3))
        residual = math.hypot(x_check - tx, y_check - ty)

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

        # ── Success ───────────────────────────────────────────────────────────
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

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()