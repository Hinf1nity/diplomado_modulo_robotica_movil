# Laboratory: 2D LiDAR Perception and Robot Motion Control in ROS2 Gazebo

## Objectives

By the end of this lab, students will be able to:

- Understand 2D LiDAR sensor principles and data structures in ROS2
- Control different robot kinematics: Ackermann, omnidirectional (mecanum), and differential drive
- Process laser scan data for obstacle detection
- Implement perception-based motion control strategies
- Compare motion capabilities across different robot platforms

## Prerequisites

- ROS2 installed and configured
- Gazebo simulation environment with Ackermann, mecanum, and differential drive robots
- Basic understanding of mobile robot kinematics
- Python or C++ programming experience
- Understanding of `geometry_msgs/Twist` message structure

## Lab Duration

Approximately 3 hours

---

## Part 1: Understanding Robot Kinematics and LiDAR (30 minutes)

### 1.1 Launch Different Robot Simulations

You have access to three robot types:

**Differential Drive Robot:**

bash

```bash
ros2 launch <package_name> diff_drive_gazebo.launch.py
```

**Omnidirectional (Mecanum) Robot:**

bash

```bash
ros2 launch <package_name> mecanum_gazebo.launch.py
```

**Ackermann Steering Robot:**

bash

```bash
ros2 launch <package_name> ackermann_gazebo.launch.py
```

### 1.2 Explore Control Interfaces

For each robot type, examine the control topic:

bash

````bash
ros2 topic info /cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

**Twist Message Structure:**
```
Vector3 linear   # linear.x, linear.y, linear.z
Vector3 angular  # angular.x, angular.y, angular.z
````

### 1.3 Manual Testing with Teleoperation

Test each robot manually:

bash

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

**Exploration Tasks:**

|Robot Type|Test Commands|Expected Behavior|
|---|---|---|
|Differential|linear.x > 0|Forward motion|
|Differential|angular.z > 0|Rotate counter-clockwise|
|Differential|linear.y ≠ 0|**No effect** (constrained)|
|Mecanum|linear.x > 0|Forward motion|
|Mecanum|linear.y > 0|**Lateral motion (strafe right)**|
|Mecanum|Both x and y|**Diagonal motion**|
|Ackermann|linear.x > 0|Forward motion|
|Ackermann|angular.z > 0|**Gradual turn** (steering constraint)|

### 1.4 LiDAR Data Review

Visualize sensor data in RViz2:

bash

```bash
ros2 run rviz2 rviz2
```

Add LaserScan display for `/scan` topic and observe data while moving robots.

**Discussion Questions:**

- How does the `Twist` message control each robot differently?
- What motion constraints exist for each robot type?
- What advantages does omnidirectional motion provide?
- Why can't Ackermann robots rotate in place?
- How does LiDAR data update frequency compare to control frequency?

---

## Challenge 1: Control Ackermann Steering Robot (40 minutes)

### Objective

Implement a node to control an Ackermann steering robot through a predefined path, respecting its kinematic constraints.

### Background: Ackermann Steering Constraints

Unlike differential drive robots, Ackermann robots:

- **Cannot rotate in place** (minimum turning radius)
- Have **steering angle limits**
- Require **coordinated speed and steering** for turning
- Need **larger space for maneuvers**

### Task

Create a node that navigates the Ackermann robot through a course:

1. **Start**: Move forward 3 meters
2. **Turn right**: Execute a smooth right turn (90 degrees)
3. **Straight**: Move forward 2 meters
4. **Turn left**: Execute a smooth left turn (90 degrees)
5. **Return**: Move forward 3 meters
6. **Stop**: Come to complete stop

### Requirements

- Account for minimum turning radius
- Implement smooth steering transitions
- Use appropriate velocity for turns vs straight sections
- Monitor execution with odometry or time-based estimation

### Implementation Structure

python

````python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Parameters
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('turn_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 0.5)
        
        # State variables
        self.current_pose = None
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        # TODO: Store current position and orientation
        self.current_pose = msg.pose.pose
        
    def move_straight(self, distance, velocity):
        """
        Move straight for given distance
        Args:
            distance: meters to travel
            velocity: forward speed (m/s)
        """
        # TODO: Implement straight motion
        # Remember: Only linear.x, angular.z = 0
        pass
    
    def execute_turn(self, angle_degrees, direction='right'):
        """
        Execute Ackermann turn
        Args:
            angle_degrees: target turn angle
            direction: 'right' or 'left'
        """
        # TODO: Implement turning with appropriate radius
        # Use both linear.x and angular.z
        # Angular velocity determines turning radius: R = v / ω
        pass
    
    def control_loop(self):
        # TODO: Implement state machine for the course
        # States: STRAIGHT_1, TURN_RIGHT, STRAIGHT_2, TURN_LEFT, STRAIGHT_3, STOP
        pass
    
    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
```

### Key Calculations

**Turning radius relationship:**
```
R = linear_velocity / angular_velocity
````

For a 90-degree turn with radius R:

- Arc length: `s = R * π/2`
- Time needed: `t = s / linear_velocity`

### Testing Procedure

1. Launch Ackermann robot in Gazebo
2. Create an obstacle course with turns
3. Run your controller node
4. Verify the robot completes the path
5. Measure and log actual path vs intended path

### Expected Output

- Robot follows the path smoothly
- No jerky steering changes
- Appropriate turning radius maintained
- Stops precisely at end position

### Bonus Challenges

- Add dynamic obstacle detection using LiDAR
- Implement reverse parking maneuver
- Calculate and display actual turning radius
- Add parameter tuning for different surface friction
- Implement parallel parking sequence

---

## Challenge 2: Control Omnidirectional (Mecanum) Robot (40 minutes)

### Objective

Exploit the unique capabilities of mecanum wheels to perform maneuvers impossible for other robot types.

### Background: Omnidirectional Motion

Mecanum robots can:

- **Move in any direction** without rotating
- **Strafe laterally** (sideways motion)
- **Combine translation and rotation** simultaneously
- **Execute complex trajectories** efficiently

### Task

Implement a node that demonstrates omnidirectional capabilities:

1. **Forward**: Move forward 2 meters
2. **Strafe right**: Move sideways right 1.5 meters (no rotation!)
3. **Diagonal**: Move diagonally (forward-left) 2 meters
4. **Rotate in place**: 360-degree rotation while stationary
5. **Crab walk**: Move forward while simultaneously strafing right
6. **Return to start**: Any creative path back

### Requirements

- Use all three degrees of freedom: linear.x, linear.y, angular.z
- Maintain smooth velocity transitions
- Demonstrate simultaneous translation and rotation
- Visualize the path in RViz

### Implementation Structure

python

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math

class OmnidirectionalController(Node):
    def __init__(self):
        super().__init__('omnidirectional_controller')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        
        # Parameters
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.current_pose = None
        self.start_pose = None
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.start_pose is None:
            self.start_pose = self.current_pose
    
    def move_forward(self, distance, velocity):
        """Standard forward motion"""
        # TODO: linear.x = velocity, linear.y = 0, angular.z = 0
        pass
    
    def strafe_sideways(self, distance, velocity, direction='right'):
        """Pure lateral motion - unique to omnidirectional!"""
        # TODO: linear.x = 0, linear.y = ±velocity, angular.z = 0
        # Positive linear.y = left, Negative linear.y = right
        pass
    
    def move_diagonal(self, distance, angle_degrees, velocity):
        """
        Move at an angle relative to robot frame
        Args:
            angle_degrees: 0=forward, 90=left, -90=right, 45=forward-left
        """
        # TODO: Decompose velocity into x and y components
        # linear.x = velocity * cos(angle)
        # linear.y = velocity * sin(angle)
        pass
    
    def rotate_in_place(self, angle_degrees, angular_vel):
        """Rotate without translating"""
        # TODO: linear.x = 0, linear.y = 0, angular.z = angular_vel
        pass
    
    def crab_walk(self, distance_forward, distance_right, velocity):
        """
        Simultaneous forward and lateral motion
        This is the signature move of omnidirectional robots!
        """
        # TODO: Combine linear.x and linear.y simultaneously
        # Calculate velocities to reach both distances proportionally
        pass
    
    def holonomic_motion(self, vel_x, vel_y, vel_theta):
        """
        Fully holonomic motion: move and rotate simultaneously
        """
        msg = Twist()
        msg.linear.x = vel_x
        msg.linear.y = vel_y
        msg.angular.z = vel_theta
        self.cmd_pub.publish(msg)
    
    def control_loop(self):
        # TODO: Implement state machine for maneuver sequence
        pass
```

### Advanced Maneuvers

**Circle strafing** (move in circle while facing center):

python

````python
def circle_strafe(self, radius, angular_velocity):
    """Robot moves in circle while rotating to always face inward"""
    linear_vel = radius * abs(angular_velocity)
    
    msg = Twist()
    msg.linear.x = linear_vel  # Move forward/backward
    msg.angular.z = angular_velocity  # Rotate
    # The magic: adjust linear.y to create circular path
    msg.linear.y = -linear_vel if angular_velocity > 0 else linear_vel
```

### Testing Procedure

1. Launch mecanum robot in open Gazebo world
2. Place markers to visualize target positions
3. Run controller and observe unique maneuvers
4. Compare efficiency with differential drive for same tasks

### Expected Output

- Robot performs all maneuvers smoothly
- True lateral motion without rotation
- Diagonal movement at specified angles
- Simultaneous translation and rotation

### Bonus Challenges

- Implement square path using only strafing (no forward motion)
- Create figure-8 pattern with continuous motion
- Add LiDAR-based strafing to avoid obstacles
- Implement "moonwalk" (move backward while facing forward)
- Create omnidirectional go-to-goal behavior

---

## Challenge 3: LiDAR-Based Wall Following (Differential Drive) (45 minutes)

### Objective
Implement a classic wall-following algorithm for a differential drive robot using 2D LiDAR data.

### Background

Wall following is a fundamental navigation behavior that:
- Keeps robot at constant distance from wall
- Enables corridor navigation
- Serves as basis for maze-solving algorithms
- Demonstrates sensor-based feedback control

### Task

Create a node that makes a differential drive robot follow a wall on its right side:

1. Maintain constant distance from wall (setpoint: 0.5m)
2. Adjust heading to stay parallel to wall
3. Handle corners (both inside and outside)
4. Transition smoothly between wall segments

### Algorithm Overview

**Sensor Processing:**
- Extract right-side LiDAR readings (e.g., -45° to -135°)
- Calculate perpendicular distance to wall
- Estimate wall angle relative to robot

**Control Strategy:**
```
Error = current_distance - desired_distance
Angular_velocity = K_p * Error + K_d * (Error - Previous_Error)
Linear_velocity = base_velocity * (1 - abs(Angular_velocity) / max_angular)
````

### Implementation Structure

python

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        
        # Parameters
        self.declare_parameter('desired_distance', 0.5)  # meters from wall
        self.declare_parameter('k_p', 1.0)  # Proportional gain
        self.declare_parameter('k_d', 0.5)  # Derivative gain
        self.declare_parameter('base_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.0)
        
        # Get parameters
        self.desired_distance = self.get_parameter('desired_distance').value
        self.k_p = self.get_parameter('k_p').value
        self.k_d = self.get_parameter('k_d').value
        
        # Subscribers and Publishers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.previous_error = 0.0
        self.wall_distance = 0.0
        
    def get_right_side_distance(self, scan_msg):
        """
        Extract distance to right wall from LiDAR scan
        Returns: average perpendicular distance to wall
        """
        # TODO: Get ranges from right side of robot
        # Typical right side: angles from -45° to -135° (270° to 315° in 0-360)
        # Calculate indices based on scan_msg.angle_min, angle_max, angle_increment
        
        # TODO: Filter out invalid readings (inf, nan, out of range)
        
        # TODO: Return minimum or average distance
        pass
    
    def estimate_wall_angle(self, scan_msg):
        """
        Estimate angle of wall relative to robot
        Uses readings at different angles to compute wall orientation
        """
        # TODO: Get two distance measurements at different angles
        # TODO: Use trigonometry to calculate wall angle
        # Advanced: Use least squares fitting for better estimate
        pass
    
    def compute_control(self, current_distance):
        """
        PD controller for wall following
        """
        # Calculate error
        error = current_distance - self.desired_distance
        
        # Derivative term
        error_derivative = error - self.previous_error
        
        # PD control law
        angular_velocity = -(self.k_p * error + self.k_d * error_derivative)
        
        # Update previous error
        self.previous_error = error
        
        # Clip angular velocity
        angular_velocity = np.clip(angular_velocity, 
                                   -self.get_parameter('max_angular_vel').value,
                                   self.get_parameter('max_angular_vel').value)
        
        # Reduce linear velocity when turning sharply
        linear_velocity = self.get_parameter('base_linear_vel').value
        linear_velocity *= (1.0 - abs(angular_velocity) / 
                           self.get_parameter('max_angular_vel').value * 0.5)
        
        return linear_velocity, angular_velocity
    
    def scan_callback(self, msg):
        """
        Main control loop triggered by LiDAR data
        """
        # Get wall distance
        wall_distance = self.get_right_side_distance(msg)
        
        if wall_distance is None or wall_distance == float('inf'):
            # No wall detected - search behavior
            self.publish_search_behavior()
            return
        
        # Compute velocities
        linear_vel, angular_vel = self.compute_control(wall_distance)
        
        # Safety check: stop if too close
        if wall_distance < 0.2:
            linear_vel = 0.0
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)
        
        # Debug output
        self.get_logger().info(
            f'Distance: {wall_distance:.2f}m, Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}')
    
    def publish_search_behavior(self):
        """Rotate to find wall"""
        cmd = Twist()
        cmd.angular.z = 0.3
        self.cmd_pub.publish(cmd)
```

### Corner Handling

**Inside Corner (concave):**

- Wall distance suddenly increases
- Increase angular velocity to follow turn
- May need to reduce linear velocity

**Outside Corner (convex):**

- Wall distance suddenly decreases
- Reduce angular velocity
- Safety stop if too close

### Testing Procedure

1. Create Gazebo world with walls/corridors
2. Launch differential drive robot
3. Run wall follower node
4. Test in:
    - Straight corridor
    - 90-degree corners
    - Curved walls
    - Open space (wall loss)

### Expected Behavior

- Robot maintains steady distance from wall
- Smooth motion along straight walls
- Successful corner navigation
- Recovery when wall is lost

### Bonus Challenges

- Implement left-wall following
- Add wall-switch capability (follow left or right)
- Implement maze-solving (left/right-hand rule)
- Add forward obstacle detection for dead-ends
- Tune PID controller (add integral term)
- Log performance metrics (distance error over time)

---

## Challenge 4: LiDAR-Based Obstacle Avoidance (Differential Drive) (45 minutes)

### Objective

Implement reactive obstacle avoidance using LiDAR to navigate a differential drive robot through a cluttered environment.

### Background

Reactive navigation:

- Uses current sensor data only (no map)
- Fast response to dynamic obstacles
- Computationally efficient
- Foundation for more complex behaviors

### Task

Create a node that:

1. Divides LiDAR scan into sectors
2. Identifies free and blocked sectors
3. Finds best direction to move
4. Generates velocity commands to navigate toward goal while avoiding obstacles

### Algorithm: Vector Field Histogram (VFH) Simplified

**Step 1:** Divide 360° scan into sectors (e.g., 18 sectors of 20° each)

**Step 2:** Build polar obstacle density map:

- For each sector, calculate obstacle density
- Density = f(distance, number of points)

**Step 3:** Find candidate directions:

- Identify valleys (low density sectors)
- Calculate cost for each valley: distance to goal + obstacle proximity

**Step 4:** Select best direction and generate motion commands

### Implementation Structure

python

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
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
        cmd = Twist()
        
        if not is_safe:
            # Emergency: stop or slow down significantly
            cmd.linear.x = 0.05
            cmd.angular.z = np.sign(target_angle) * self.get_parameter('max_angular_vel').value
            return cmd
        
        # Normal operation
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        
        # Angular velocity proportional to angle error
        angular_vel = 2.0 * target_angle  # Proportional control
        angular_vel = np.clip(angular_vel, -max_angular, max_angular)
        
        # Linear velocity reduced when turning
        alignment_factor = 1.0 - abs(target_angle) / math.pi
        linear_vel = max_linear * alignment_factor
        
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        
        return cmd
    
    def visualize_sectors(self, sectors):
        """Create markers to visualize sector analysis"""
        # TODO: Create MarkerArray showing sector densities
        # Use different colors for safe/unsafe sectors
        pass
    
    def scan_callback(self, msg):
        """Main control loop"""
        # Divide scan into sectors
        sectors = self.divide_scan_into_sectors(msg)
        
        # Get goal angle (could come from higher-level planner)
        goal_angle = self.get_parameter('goal_angle').value
        
        # Find best direction
        best_angle, is_safe = self.find_best_direction(sectors, goal_angle)
        
        # Generate velocity command
        cmd = self.compute_velocity_command(best_angle, is_safe)
        
        # Publish
        self.cmd_pub.publish(cmd)
        
        # Visualize (optional)
        # self.visualize_sectors(sectors)
        
        # Debug
        status = "SAFE" if is_safe else "DANGER"
        self.get_logger().info(
            f'[{status}] Target angle: {math.degrees(best_angle):.1f}°, '
            f'Cmd: v={cmd.linear.x:.2f}, ω={cmd.angular.z:.2f}')
```

### Testing Scenarios

Create Gazebo worlds with:

1. **Scattered obstacles**: Random placement
2. **Narrow passages**: Test gap detection
3. **Moving obstacles**: Dynamic environment
4. **Cluttered space**: Dense obstacle field
5. **Maze**: Multiple decision points

### Expected Behavior

- Robot navigates around obstacles smoothly
- Chooses efficient paths through open space
- Recovers from trapped situations
- Balances between goal-seeking and obstacle avoidance

### Bonus Challenges

- Add hysteresis to prevent oscillation
- Implement dynamic goal updating (interactive goal setting)
- Add speed adaptation based on obstacle density
- Combine with wall-following for corridor navigation
- Implement deliberative layer (remember visited areas)
- Add multi-objective optimization (speed, safety, efficiency)

---

## Integration and Comparison (20 minutes)

### Cross-Platform Analysis

Create a comparison table of your implementations:

|Feature|Differential|Mecanum|Ackermann|
|---|---|---|---|
|Degrees of Freedom|2 (x, θ)|3 (x, y, θ)|2 (x, θ)|
|Can rotate in place?|✓|✓|✗|
|Lateral motion?|✗|✓|✗|
|Minimum turning radius|~0|~0|>0 (constrained)|
|Best for wall following|✓|○|○|
|Best for obstacle avoidance|✓|✓|○|
|Parking capability|Good|Excellent|Limited|

### Reflection Questions

1. Which robot type handled tight spaces best?
2. How did kinematic constraints affect your algorithms?
3. Which control strategy was most robust?
4. What modifications would improve each implementation?