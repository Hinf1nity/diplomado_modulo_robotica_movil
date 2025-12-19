import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameter('num_sectors', 18)
        self.declare_parameter('obstacle_threshold', 1.0)  # meters
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('goal_angle', 0.0)  # relative angle to goal
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sector_markers', 10)
        
        self.num_sectors = self.get_parameter('num_sectors').value
        
    def divide_scan_into_sectors(self, scan_msg):
        """
        Divide LiDAR scan into angular sectors
        Returns: list of (angle, min_distance, density) for each sector
        """
        num_ranges = len(scan_msg.ranges)
        sector_size = num_ranges // self.num_sectors
        sectors = []
        
        for i in range(self.num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            
            # Get ranges for this sector
            sector_ranges = scan_msg.ranges[start_idx:end_idx]
            
            # Filter valid ranges
            valid_ranges = [r for r in sector_ranges 
                           if not math.isinf(r) and not math.isnan(r) 
                           and scan_msg.range_min < r < scan_msg.range_max]
            
            if valid_ranges:
                min_dist = min(valid_ranges)
                avg_dist = np.mean(valid_ranges)
                density = len(valid_ranges) / len(sector_ranges)
            else:
                min_dist = float('inf')
                avg_dist = float('inf')
                density = 0.0
            
            # Calculate sector center angle
            center_idx = (start_idx + end_idx) // 2
            angle = scan_msg.angle_min + center_idx * scan_msg.angle_increment
            
            sectors.append({
                'angle': angle,
                'min_distance': min_dist,
                'avg_distance': avg_dist,
                'density': density
            })
        
        return sectors
    
    def calculate_sector_cost(self, sector, goal_angle):
        """
        Calculate cost for navigating in this sector's direction
        Lower cost = better direction
        """
        # Distance cost: prefer farther obstacles
        if sector['min_distance'] == float('inf'):
            distance_cost = 0.0
        else:
            distance_cost = 1.0 / (sector['min_distance'] + 0.1)
        
        # Heading cost: prefer directions toward goal
        angle_diff = abs(self.normalize_angle(sector['angle'] - goal_angle))
        heading_cost = angle_diff / math.pi
        
        # Density cost: prefer less cluttered directions
        density_cost = sector['density']
        
        # Weighted combination
        total_cost = (2.0 * distance_cost + 
                     1.0 * heading_cost + 
                     0.5 * density_cost)
        
        return total_cost
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def find_best_direction(self, sectors, goal_angle):
        """
        Find the best direction to move
        Returns: (best_angle, is_safe)
        """
        threshold = self.get_parameter('obstacle_threshold').value
        
        # Filter safe sectors (far enough from obstacles)
        safe_sectors = [s for s in sectors if s['min_distance'] > threshold]
        
        if not safe_sectors:
            # Emergency: no safe direction, return safest available
            safest = max(sectors, key=lambda s: s['min_distance'])
            return safest['angle'], False
        
        # Find sector with minimum cost
        best_sector = min(safe_sectors, 
                         key=lambda s: self.calculate_sector_cost(s, goal_angle))
        
        return best_sector['angle'], True
    
    def compute_velocity_command(self, target_angle, is_safe):
        """
        Generate Twist message based on target direction
        """
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        if not is_safe:
            # Emergency: stop or slow down significantly
            cmd.twist.linear.x = 0.05
            cmd.twist.angular.z = np.sign(target_angle) * self.get_parameter('max_angular_vel').value
            return cmd
        
        # Normal operation
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        
        # Angular velocity proportional to angle error
        angular_vel = 1.0* target_angle  # Proportional control
        angular_vel = np.clip(angular_vel, -max_angular, max_angular)
        
        # Linear velocity reduced when turning
        alignment_factor = 1.0 - abs(target_angle) / math.pi
        linear_vel = max_linear * alignment_factor
        
        cmd.twist.linear.x = linear_vel
        cmd.twist.angular.z = angular_vel
        
        return cmd
    
    def visualize_sectors(self, sectors):
        """Create markers to visualize sector analysis"""
        # TODO: Create MarkerArray showing sector densities
        # Use different colors for safe/unsafe sectors
        marker_array = MarkerArray()
        for i, sector in enumerate(sectors):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'sectors'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.1  # shaft diameter
            marker.scale.y = 0.2  # head diameter
            marker.scale.z = 0.2  # head length
            
            # Position and orientation
            angle = sector['angle']
            marker.pose.position.x = 0.5 * math.cos(angle)
            marker.pose.position.y = 0.5 * math.sin(angle)
            marker.pose.position.z = 0.0
            quat_z = math.sin(angle / 2.0)
            quat_w = math.cos(angle / 2.0)
            marker.pose.orientation.z = quat_z
            marker.pose.orientation.w = quat_w
            
            # Color based on safety
            threshold = self.get_parameter('obstacle_threshold').value
            if sector['min_distance'] > threshold:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
    
    def scan_callback(self, msg):
        """Main control loop"""
        # Divide scan into sectors
        sectors = self.divide_scan_into_sectors(msg)
        self.visualize_sectors(sectors)
        
        # Get goal angle (could come from higher-level planner)
        goal_angle = self.get_parameter('goal_angle').value
        
        # Find best direction
        best_angle, is_safe = self.find_best_direction(sectors, goal_angle)
        
        # Generate velocity command
        cmd = self.compute_velocity_command(best_angle, is_safe)
        
        # Publish
        # self.cmd_pub.publish(cmd)
        
        # Visualize (optional)
        # self.visualize_sectors(sectors)
        
        # Debug
        status = "SAFE" if is_safe else "DANGER"
        self.get_logger().info(
            f'[{status}] Target angle: {math.degrees(best_angle):.1f}°, '
            f'Cmd: v={cmd.twist.linear.x:.2f}, ω={cmd.twist.angular.z:.2f}')
        
def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()