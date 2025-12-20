#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
import numpy as np
from marker_tf_listener import MarkerTFListener


class DemoIBVSController(MarkerTFListener):
    """
    PROFESSOR DEMO: Image-Based Visual Servoing

    This controller demonstrates:
    - Computing image features (pixel coordinates) from 3D pose
    - Defining desired features in image space
    - Proportional control in image space
    - Handling depth estimation
    """

    def __init__(self):
        super().__init__('demo_ibvs_controller')

        # Control gain
        self.declare_parameter('lambda_gain', 0.5)
        self.lambda_gain = self.get_parameter('lambda_gain').value

        # Desired image features (pixels)
        self.declare_parameter('desired_u', 320.0)  # center of 640px image
        self.declare_parameter('desired_v', 240.0)  # center of 480px image
        self.declare_parameter('desired_depth', 0.5)  # 50cm depth

        self.u_desired = self.get_parameter('desired_u').value
        self.v_desired = self.get_parameter('desired_v').value
        self.z_desired = self.get_parameter('desired_depth').value

        # Camera intrinsics
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Current depth estimate
        self.Z = self.z_desired  # Initial estimate

        # Tolerance
        self.pixel_tolerance = 20  # pixels
        self.depth_tolerance = 0.05  # meters

        # Velocity limits
        self.max_linear_vel = 0.3
        self.max_angular_vel = 1.0

        # Subscribers and publishers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop at 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('=== IBVS DEMO Controller Ready ===')
        self.get_logger().info(
            f'Target: u={self.u_desired}px, v={self.v_desired}px, z={self.z_desired}m')

    def camera_info_callback(self, msg):
        """Get camera calibration parameters"""
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.fx = K[0, 0]
            self.fy = K[1, 1]
            self.cx = K[0, 2]
            self.cy = K[1, 2]
            self.get_logger().info(
                f'Camera calibrated: fx={self.fx:.1f}, fy={self.fy:.1f}')

    def control_loop(self):
        """Main control loop"""

        # Wait for camera calibration
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2.0)
            return

        # Get marker transform
        transform = self.get_marker_transform()

        if transform is None:
            # Marker not visible
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().warn('Marker not visible!', throttle_duration_sec=2.0)
            return

        # Extract 3D position
        position, _ = self.transform_to_pose(transform)
        x, y, z = position

        # Update depth estimate (adaptive)
        self.Z = z

        # Project 3D point to image plane
        u_current = self.fx * (x / z) + self.cx
        v_current = self.fy * (y / z) + self.cy

        # Compute image space errors
        error_u = u_current - self.u_desired
        error_v = v_current - self.v_desired
        error_z = z - self.z_desired

        # Image space error magnitude
        image_error = np.sqrt(error_u**2 + error_v**2)

        # Check if goal reached
        if image_error < self.pixel_tolerance and abs(error_z) < self.depth_tolerance:
            self.get_logger().info('✓ GOAL REACHED!', throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(Twist())
            return

        # Compute normalized image coordinates
        x_norm = (u_current - self.cx) / self.fx

        # Control law (simplified for differential drive)
        cmd = Twist()

        # Angular velocity: center marker horizontally in image
        # Negative because positive image x corresponds to negative robot rotation
        cmd.angular.z = -self.lambda_gain * x_norm * 2.5

        # Linear velocity: approach to desired depth
        cmd.linear.x = -self.lambda_gain * error_z

        # Apply velocity limits
        cmd.linear.x = np.clip(
            cmd.linear.x, -self.max_linear_vel, self.max_linear_vel)
        cmd.angular.z = np.clip(
            cmd.angular.z, -self.max_angular_vel, self.max_angular_vel)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Log status
        self.get_logger().info(
            f'IBVS | Image: [{u_current:.1f}, {v_current:.1f}]px, Z={z:.2f}m | '
            f'Err: img={image_error:.1f}px, depth={error_z:.3f}m | '
            f'Cmd: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    controller = DemoIBVSController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.cmd_vel_pub.publish(Twist())
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
