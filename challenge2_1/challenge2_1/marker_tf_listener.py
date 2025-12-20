#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation


class MarkerTFListener(Node):
    """Base class for listening to ArUco marker transforms"""

    def __init__(self, node_name):
        super().__init__(node_name)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_frame', 'base_link')

        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.target_frame = f'aruco_marker_{self.target_marker_id}'

        self.get_logger().info(
            f'Initialized - Tracking marker {self.target_marker_id}')

    def get_marker_transform(self):
        """Get current transform from camera to marker"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            return transform
        except TransformException as ex:
            return None

    def transform_to_pose(self, transform):
        """Extract position and orientation from transform"""
        if transform is None:
            return None, None

        # Position
        position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Orientation as quaternion
        quat = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])

        return position, quat

    def quaternion_to_euler(self, quat):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        rotation = Rotation.from_quat(quat)
        return rotation.as_euler('xyz')
