#!/usr/bin/env python3
"""
test_ik_client.py
=================
ROS 2 service client test script for the /compute_ik service.

What this node does:
  - Creates a ROS 2 node that acts as a client for the '/compute_ik' service.
  - Sends three predefined target positions to the IK solver.
  - Prints the results (success, joint angles, forward‑kinematics check) in a nice table.
  - This is used to verify that the IK solver works correctly.

Tests:
  #1  (0.5, 0.4)   — reachable target (inside workspace)
  #2  (0.0, 0.0)   — degenerate edge‑case (the origin)
  #3  (1.5, 1.5)   — outside workspace (unreachable)

Usage:
  # Make sure the ik_solver node is already running (e.g., from the launch file).
  # Then run this script in another terminal:
  ros2 run planar_arm_kinematics test_ik_client
"""

import math      # For converting radians to degrees and forward‑kinematics check
import sys       # For exiting with an error code if the service is not available

import rclpy
from rclpy.node import Node

# Import the custom service definition (the same one used by ik_solver)
from planar_arm_interfaces.srv import ComputeIK


# ---------------------------------------------------------------------------
# Test cases: each is a dictionary with id, x, y, and expected outcome (for display)
# ---------------------------------------------------------------------------
TEST_CASES = [
    {'id': 1, 'x': 0.5,  'y': 0.4,  'expected': 'Reachable'},
    {'id': 2, 'x': 0.0,  'y': 0.0,  'expected': 'Edge case (origin)'},
    {'id': 3, 'x': 1.5,  'y': 1.5,  'expected': 'Unreachable'},
]


# ---------------------------------------------------------------------------
# Client node class
# ---------------------------------------------------------------------------
class IKTestClient(Node):
    """A simple ROS 2 node that calls the IK service with test coordinates."""

    def __init__(self) -> None:
        """Constructor: creates a client for the '/compute_ik' service."""
        super().__init__('ik_test_client')   # Node name

        # Create a client that can call the ComputeIK service on the topic '/compute_ik'
        self._client = self.create_client(ComputeIK, '/compute_ik')

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for the IK service to become available.
        Returns True if the service is ready, False if timeout occurs.
        """
        self.get_logger().info('Waiting for /compute_ik service...')
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Service /compute_ik not available!')
            return False
        self.get_logger().info('Service ready.')
        return True

    def call(self, target_x: float, target_y: float) -> ComputeIK.Response:
        """
        Call the IK service with the given target (x, y).
        Returns the service response (which contains success flag, angles, and message).
        """
        req = ComputeIK.Request()
        req.target_x = target_x
        req.target_y = target_y

        # Send the request asynchronously and wait for the response
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)   # Wait until the service replies
        return future.result()                            # Get the response object


# ---------------------------------------------------------------------------
# Helper functions for pretty printing
# ---------------------------------------------------------------------------

def _separator(char: str = '─', width: int = 70) -> str:
    """Return a string of repeated characters (e.g., a line)."""
    return char * width


def _print_result(test: dict, resp: ComputeIK.Response) -> None:
    """
    Print the result of one test case in a human‑readable format.
    Includes the target, success flag, message, joint angles (in degrees and radians),
    and a forward‑kinematics check to verify the solution.
    """
    # Decide the status symbol: ✓ for success, ✗ for failure
    status = '✓ SUCCESS' if resp.success else '✗ FAILURE'
    print(_separator())
    print(f'  Test #{test["id"]}  |  Expected: {test["expected"]}  |  {status}')
    print(_separator('─', 70))
    print(f'  Target   : x = {test["x"]:.4f} m   y = {test["y"]:.4f} m')
    print(f'  Success  : {resp.success}')
    print(f'  Message  : {resp.message}')

    # If the IK succeeded, also print the joint angles (both radians and degrees)
    if resp.success:
        print(f'  θ1       : {resp.theta1:+.6f} rad  ({math.degrees(resp.theta1):+.2f}°)')
        print(f'  θ2       : {resp.theta2:+.6f} rad  ({math.degrees(resp.theta2):+.2f}°)')
        print(f'  θ3       : {resp.theta3:+.6f} rad  ({math.degrees(resp.theta3):+.2f}°)')

        # Optional: verify the solution by running forward kinematics ourselves
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
# Main function
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    """
    Entry point: initialise ROS, create the client, wait for the service,
    run the three test cases, print the results, and shut down.
    """
    rclpy.init(args=args)                # Start ROS 2 communication
    client = IKTestClient()              # Create the client node

    # Wait for the IK service to be available (max 10 seconds)
    if not client.wait_for_server(timeout_sec=10.0):
        client.destroy_node()
        rclpy.shutdown()
        sys.exit(1)                      # Exit with error code

    # Print a header
    print('\n' + _separator('═'))
    print('  IK Solver Test Client — 3-DOF Planar Arm')
    print(_separator('═'))

    # Loop through all test cases
    for test in TEST_CASES:
        resp = client.call(test['x'], test['y'])   # Call the service
        _print_result(test, resp)                  # Print the result

    # Print footer
    print(_separator('═'))
    print('  All test cases completed.')
    print(_separator('═') + '\n')

    # Clean up
    client.destroy_node()
    rclpy.shutdown()


# This condition ensures that main() is called when the script is executed directly.
if __name__ == '__main__':
    main()