#!/usr/bin/env python3
"""
test_ik_client.py
=================
ROS 2 service client test script for the /compute_ik service.

Tests:
  #1  (0.5, 0.4)   — reachable target
  #2  (0.0, 0.0)   — degenerate edge-case (origin)
  #3  (1.5, 1.5)   — outside workspace (unreachable)

Usage:
  # In a sourced workspace, with ik_solver already running:
  ros2 run planar_arm_kinematics test_ik_client

  # Or call directly:
  python3 test_ik_client.py
"""

import math
import sys

import rclpy
from rclpy.node import Node

from planar_arm_interfaces.srv import ComputeIK


# ---------------------------------------------------------------------------

TEST_CASES = [
    {'id': 1, 'x': 0.5,  'y': 0.4,  'expected': 'Reachable'},
    {'id': 2, 'x': 0.0,  'y': 0.0,  'expected': 'Edge case (origin)'},
    {'id': 3, 'x': 1.5,  'y': 1.5,  'expected': 'Unreachable'},
]


class IKTestClient(Node):

    def __init__(self) -> None:
        super().__init__('ik_test_client')
        self._client = self.create_client(ComputeIK, '/compute_ik')

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info('Waiting for /compute_ik service...')
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Service /compute_ik not available!')
            return False
        self.get_logger().info('Service ready.')
        return True

    def call(self, target_x: float, target_y: float) -> ComputeIK.Response:
        req = ComputeIK.Request()
        req.target_x = target_x
        req.target_y = target_y
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


# ---------------------------------------------------------------------------

def _separator(char: str = '─', width: int = 70) -> str:
    return char * width


def _print_result(test: dict, resp: ComputeIK.Response) -> None:
    status = '✓ SUCCESS' if resp.success else '✗ FAILURE'
    print(_separator())
    print(f'  Test #{test["id"]}  |  Expected: {test["expected"]}  |  {status}')
    print(_separator('─', 70))
    print(f'  Target   : x = {test["x"]:.4f} m   y = {test["y"]:.4f} m')
    print(f'  Success  : {resp.success}')
    print(f'  Message  : {resp.message}')
    if resp.success:
        print(f'  θ1       : {resp.theta1:+.6f} rad  ({math.degrees(resp.theta1):+.2f}°)')
        print(f'  θ2       : {resp.theta2:+.6f} rad  ({math.degrees(resp.theta2):+.2f}°)')
        print(f'  θ3       : {resp.theta3:+.6f} rad  ({math.degrees(resp.theta3):+.2f}°)')

        # Forward-check
        L1, L2, L3 = 0.4, 0.3, 0.2
        xc = (L1 * math.cos(resp.theta1)
              + L2 * math.cos(resp.theta1 + resp.theta2)
              + L3 * math.cos(resp.theta1 + resp.theta2 + resp.theta3))
        yc = (L1 * math.sin(resp.theta1)
              + L2 * math.sin(resp.theta1 + resp.theta2)
              + L3 * math.sin(resp.theta1 + resp.theta2 + resp.theta3))
        err = math.hypot(xc - test['x'], yc - test['y'])
        print(f'  FK check : ({xc:+.4f}, {yc:+.4f})  residual = {err:.6f} m')


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    client = IKTestClient()

    if not client.wait_for_server(timeout_sec=10.0):
        client.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print('\n' + _separator('═'))
    print('  IK Solver Test Client — 3-DOF Planar Arm')
    print(_separator('═'))

    for test in TEST_CASES:
        resp = client.call(test['x'], test['y'])
        _print_result(test, resp)

    print(_separator('═'))
    print('  All test cases completed.')
    print(_separator('═') + '\n')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()