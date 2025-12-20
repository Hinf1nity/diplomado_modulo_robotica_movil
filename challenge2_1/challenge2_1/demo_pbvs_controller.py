#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import numpy as np
from marker_tf_listener import MarkerTFListener


class DemoPBVSController(MarkerTFListener):
    """
    PROFESSOR DEMO: Position-Based Visual Servoing

    This controller demonstrates:
    - Reading marker pose from TF
    - Computing 3D position error
    - Computing orientation error (yaw)
    - Proportional control for both linear and angular velocities
    """

    def __init__(self):
        super().__init__('demo_pbvs_controller')

        # Control gains
        self.declare_parameter('kp_linear', 0.4)
        self.declare_parameter('kp_angular', 0.8)

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value

        # Desired pose relative to marker (camera frame)
        self.declare_parameter('desired_distance', 0.5)  # 50cm from marker
        self.declare_parameter('desired_lateral_offset', 0.0)  # centered

        self.desired_z = self.get_parameter('desired_distance').value
        self.desired_x = self.get_parameter('desired_lateral_offset').value
        self.desired_y = 0.0

        # Goal tolerances
        self.position_tolerance = 0.05  # 5cm
        self.yaw_tolerance = 0.1  # ~5.7 degrees

        # Velocity limits
        self.max_linear_vel = 0.3  # m/s
        self.max_angular_vel = 1.0  # rad/s

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=== PBVS DEMO Controller Ready ===')
        self.get_logger().info(
            f'Target: z={self.desired_z}m, x={self.desired_x}m')
        self.get_logger().info(
            f'Gains: kp_linear={self.kp_linear}, kp_angular={self.kp_angular}')

    def control_loop(self):
        """Main control loop"""

        # Get marker transform
        transform = self.get_marker_transform()

        if transform is None:
            # Marker not visible - stop robot
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('Marker not visible!', throttle_duration_sec=2.0)
            return

        # Extract position and orientation
        position, quat = self.transform_to_pose(transform)

        # Current position in camera frame
        x_current = position[0]
        y_current = position[1]
        z_current = position[2]

        # Compute position errors
        error_x = self.desired_x - x_current
        error_y = self.desired_y - y_current
        error_z = self.desired_z - z_current

        # Total position error
        position_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)

        # Compute desired yaw (angle to face marker)
        # For mobile robot: we want to rotate to align with marker
        desired_yaw = -np.arctan2(y_current, z_current)

        # Current yaw from quaternion
        euler = self.quaternion_to_euler(quat)
        current_yaw = euler[2]

        # Yaw error (simplified for demo)
        yaw_error = desired_yaw

        # Normalize to [-pi, pi]
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        # Create control command
        cmd = Twist()

        # Check if goal reached
        if position_error < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.get_logger().info('✓ GOAL REACHED!', throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(cmd)
            return

        # Proportional control
        # Linear velocity: move forward/backward based on depth error
        cmd.linear.x = self.kp_linear * error_z

        # Angular velocity: rotate to face marker
        cmd.angular.z = self.kp_angular * yaw_error

        # Apply velocity limits
        cmd.linear.x = np.clip(
            cmd.linear.x, -self.max_linear_vel, self.max_linear_vel)
        cmd.angular.z = np.clip(
            cmd.angular.z, -self.max_angular_vel, self.max_angular_vel)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Log status
        self.get_logger().info(
            f'PBVS | Pos: [{x_current:.2f}, {y_current:.2f}, {z_current:.2f}]m | '
            f'Err: pos={position_error:.3f}m, yaw={np.degrees(yaw_error):.1f}° | '
            f'Cmd: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    controller = DemoPBVSController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        controller.cmd_vel_pub.publish(Twist())
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
