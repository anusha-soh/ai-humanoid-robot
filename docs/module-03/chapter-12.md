# Chapter 12: Path Planning for Bipedal Robots (Nav2)

## Overview

Welcome to the world of navigation and path planning, where your humanoid robot learns to autonomously navigate through complex environments. Navigation is a critical capability that allows robots to move from one location to another while avoiding obstacles and respecting their physical constraints. In this chapter, you'll master the Navigation 2 (Nav2) stack, which is the current standard for mobile robot navigation in ROS 2, and learn how to adapt it for bipedal robots.

Path planning involves finding optimal routes through environments while considering various constraints like obstacles, robot dynamics, and terrain traversability. For bipedal robots, navigation is especially challenging due to balance constraints, limited footstep placement options, and the need for stable gaits. You'll learn how to configure Nav2 for humanoid robots, implement custom path planners that account for bipedal locomotion, and create robust navigation behaviors. By the end of this chapter, you'll have built a complete navigation system that can guide your humanoid robot through complex environments with stable, energy-efficient walking patterns.

### Learning Objectives

By the end of this chapter, you will be able to:
- Configure and customize the Nav2 navigation stack for humanoid robots
- Implement path planning algorithms that account for bipedal locomotion constraints
- Create behavior trees for complex navigation tasks
- Adapt navigation parameters for bipedal robot dynamics
- Build robust waypoint following and patrol behaviors

### Prerequisites

Before starting this chapter, you should have:
- Completed Modules 1-3 (ROS 2 basics, simulation, perception, and SLAM)
- Understanding of robot kinematics and dynamics
- Experience with ROS 2 navigation systems
- Basic knowledge of gait generation for bipedal robots

## Concepts

### Navigation 2 (Nav2) Architecture

Nav2 is the next-generation navigation stack for ROS 2, featuring:

- **Planner Server**: Global path planning (A*, Dijkstra, etc.)
- **Controller Server**: Local path following and obstacle avoidance
- **Recovery Server**: Behaviors for getting unstuck
- **BT Navigator**: Behavior tree for navigation task orchestration
- **Lifecycle Management**: Proper state management for all components

### Path Planning Algorithms

Various algorithms for finding optimal paths:

- **A* Algorithm**: Heuristic-based search that balances path cost and estimated distance
- **Dijkstra's Algorithm**: Guarantees optimal path but slower than A*
- **RRT (Rapidly-exploring Random Trees)**: Probabilistic roadmap for complex spaces
- **Theta* Algorithm**: Any-angle path planning that doesn't follow grid edges
- **Potential Field Methods**: Gradient-based navigation with attractive and repulsive forces

### Bipedal Robot Constraints

Bipedal robots have unique navigation considerations:

- **Balance Maintenance**: Must maintain center of mass within support polygon
- **Footstep Planning**: Limited placement options for feet
- **Gait Stability**: Need to transition smoothly between steps
- **Turning Radius**: Limited by leg length and hip constraints
- **Terrain Traversability**: Can't climb steep slopes or step over large obstacles

### Behavior Trees for Navigation

Behavior trees organize complex navigation tasks:

- **Sequence Nodes**: Execute children in order until one fails
- **Selector Nodes**: Try children until one succeeds
- **Decorator Nodes**: Modify child behavior (inverter, repeater, etc.)
- **Action Nodes**: Execute specific navigation actions
- **Condition Nodes**: Check navigation conditions (obstacle detected, goal reached)

### Local vs Global Planning

Navigation involves two planning levels:

- **Global Planner**: Creates high-level path from start to goal ignoring dynamic obstacles
- **Local Planner**: Executes path while avoiding immediate obstacles and respecting robot constraints
- **Replanning**: Updating plans when obstacles appear or robot deviates from path

### Navigation Recovery Behaviors

Strategies for handling navigation failures:

- **Backtrack**: Reverse direction to find alternative path
- **Spin**: Rotate in place to clear obstacle or find path
- **Wait**: Pause briefly to allow dynamic obstacles to clear
- **Reset Locally**: Reset local costmap to clear temporary obstacles

## Examples

### Example 1: Configuring Nav2 for Bipedal Robot

First, let's create a configuration file for Nav2 tailored to bipedal robots:

```yaml
# Nav2 configuration for bipedal robot
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    default_server_timeout: 20
    enable_groot_monitoring: True
    enable_logging: True
    enable_advanced_monitoring: True

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Bipedal-specific controller configuration
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIC"
      debug_visualizations: false
      rate: 20.0
      min_approach_linear_velocity: 0.05
      approach_tolerance: 0.1
      tracking_smooth_look_ahead_distance: 0.6
      tracking_orientation_threshold: 0.25
      goal_tracking_tolerance: 0.25
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.05
      short_circuit_obstacle_assessment: false
      obstacle_cost_scaling_factor: 1.0
      inflation_cost_scaling_factor: 1.0

      # Bipedal-specific parameters
      max_linear_speed: 0.3      # Slower for stability
      min_linear_speed: 0.05     # Minimum for smooth motion
      max_angular_speed: 0.4     # Limited turning for balance
      linear_granularity: 0.05   # Fine-grained linear control
      angular_granularity: 0.05  # Fine-grained angular control

progress_checker:
  ros__parameters:
    use_sim_time: True
    plugin: "nav2_controller::SimpleProgressChecker"
    required_movement_radius: 0.5  # Increased for bipedal stability
    movement_time_allowance: 10.0

goal_checker:
  ros__parameters:
    use_sim_time: True
    plugin: "nav2_controller::SimpleGoalChecker"
    xy_goal_tolerance: 0.25    # Larger for bipedal imprecision
    yaw_goal_tolerance: 0.2    # More tolerant of orientation
    stateful: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_rollout_costs: true
      lethal_cost_threshold: 100
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      transform_tolerance: 0.5
      footprint: "[ [0.3, 0.25], [0.3, -0.25], [-0.3, -0.25], [-0.3, 0.25] ]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_simulation_time: True
      rolling_window: false
      track_unknown_space: true
      resolution: 0.05
      footprint: "[ [0.3, 0.25], [0.3, -0.25], [-0.3, -0.25], [-0.3, 0.25] ]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5        # Increased for bipedal flexibility
      use_astar: false      # Use Dijkstra for guaranteed optimality
      allow_unknown: true   # Allow planning through unknown space

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      weight_smooth: 0.9
      weight_data: 0.1

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57  # 90 degrees for bipedal turning
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.05
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0
```

### Example 2: Creating a Bipedal-Specific Path Planner

Now let's create a custom path planner node that accounts for bipedal constraints:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.srv import GetPlan
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.distance import euclidean
import heapq


class BipedalPathPlanner(Node):
    """
    Custom path planner for bipedal robots that accounts for balance and gait constraints.
    """

    def __init__(self):
        super().__init__('bipedal_path_planner')

        # Create service server for path planning
        self.planner_service = self.create_service(
            GetPlan,
            '/bipedal_plan',
            self.plan_path_callback
        )

        # Create publisher for planned paths
        self.path_pub = self.create_publisher(
            Path,
            '/bipedal_plan',
            10
        )

        # Create subscription to costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        # Store costmap
        self.costmap = None
        self.costmap_info = None

        # Bipedal-specific parameters
        self.max_step_length = 0.3  # Maximum step length for stability
        self.min_turn_radius = 0.4  # Minimum turning radius for balance
        self.balance_margin = 0.1   # Extra margin for safety

        self.get_logger().info('Bipedal Path Planner initialized')

    def costmap_callback(self, msg):
        """Store the current costmap"""
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.costmap_info = msg.info

    def plan_path_callback(self, request, response):
        """Plan path for bipedal robot considering its constraints"""
        try:
            # Convert start and goal to map coordinates
            start_map = self.world_to_map(
                request.start.pose.position.x,
                request.start.pose.position.y
            )
            goal_map = self.world_to_map(
                request.goal.pose.position.x,
                request.goal.pose.position.y
            )

            # Plan path using modified A* algorithm
            path = self.bipedal_a_star(start_map, goal_map)

            if path:
                # Convert path to world coordinates
                world_path = []
                for x, y in path:
                    world_x, world_y = self.map_to_world(x, y)
                    pose = PoseStamped()
                    pose.pose.position.x = world_x
                    pose.pose.position.y = world_y
                    pose.pose.position.z = 0.0
                    world_path.append(pose)

                # Create Path message
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.poses = world_path

                response.plan = path_msg
                response.plan_found = True

                # Publish path
                self.path_pub.publish(path_msg)

                self.get_logger().info(f'Bipedal path planned with {len(path)} waypoints')
            else:
                response.plan_found = False
                self.get_logger().warn('No path found for bipedal robot')

        except Exception as e:
            self.get_logger().error(f'Error in path planning: {e}')
            response.plan_found = False

        return response

    def bipedal_a_star(self, start, goal):
        """Modified A* algorithm for bipedal robot constraints"""
        if not self.costmap:
            return None

        rows, cols = self.costmap.shape
        start_x, start_y = start
        goal_x, goal_y = goal

        # Check if start and goal are valid
        if not (0 <= start_x < cols and 0 <= start_y < rows and
                0 <= goal_x < cols and 0 <= goal_y < rows):
            return None

        # Check if start or goal are occupied
        if self.costmap[start_y, start_x] >= 50 or self.costmap[goal_y, goal_x] >= 50:
            return None

        # Priority queue for A* (f_score, x, y)
        pq = [(0, start_x, start_y)]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic((start_x, start_y), (goal_x, goal_y))}

        # Directions for movement (including diagonal)
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        while pq:
            current_f, current_x, current_y = heapq.heappop(pq)

            if (current_x, current_y) == (goal_x, goal_y):
                # Reconstruct path
                path = []
                current = (current_x, current_y)
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                path.reverse()
                return path

            for dx, dy in directions:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy

                # Check bounds
                if not (0 <= neighbor_x < cols and 0 <= neighbor_y < rows):
                    continue

                # Check if walkable (cost < 50 means not occupied)
                if self.costmap[neighbor_y, neighbor_x] >= 50:
                    continue

                # Bipedal constraint: maximum step length
                step_dist = np.sqrt(dx*dx + dy*dy) * self.costmap_info.resolution
                if step_dist > self.max_step_length:
                    continue

                # Calculate tentative g_score
                movement_cost = self.calculate_movement_cost((current_x, current_y), (neighbor_x, neighbor_y))
                tentative_g_score = g_score[(current_x, current_y)] + movement_cost

                neighbor = (neighbor_x, neighbor_y)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = (current_x, current_y)
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, (goal_x, goal_y))
                    heapq.heappush(pq, (f_score[neighbor], neighbor_x, neighbor_y))

        return None  # No path found

    def calculate_movement_cost(self, current, neighbor):
        """Calculate movement cost considering bipedal constraints"""
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]

        # Base cost is the Euclidean distance
        base_cost = np.sqrt(dx*dx + dy*dy) * self.costmap_info.resolution

        # Add cost for traversability
        neighbor_cost = self.costmap[neighbor[1], neighbor[0]]
        traversability_penalty = neighbor_cost * 0.01  # Scale down occupancy cost

        # Add penalty for diagonal movements (more challenging for bipedal)
        if dx != 0 and dy != 0:
            diagonal_penalty = 0.1  # Additional cost for diagonal steps
        else:
            diagonal_penalty = 0.0

        return base_cost + traversability_penalty + diagonal_penalty

    def heuristic(self, a, b):
        """Heuristic function for A* (Euclidean distance)"""
        return euclidean(a, b)

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        if not self.costmap_info:
            return (0, 0)

        map_x = int((x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        map_y = int((y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        return (map_x, map_y)

    def map_to_world(self, x, y):
        """Convert map indices to world coordinates"""
        if not self.costmap_info:
            return (0.0, 0.0)

        world_x = x * self.costmap_info.resolution + self.costmap_info.origin.position.x
        world_y = y * self.costmap_info.resolution + self.costmap_info.origin.position.y
        return (world_x, world_y)


def main(args=None):
    rclpy.init(args=args)
    planner = BipedalPathPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 3: Behavior Tree Navigation Node

Create a behavior tree navigation node for complex navigation tasks:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import random


class BehaviorTreeNavigator(Node):
    """
    Node that implements behavior tree navigation for complex tasks.
    """

    def __init__(self):
        super().__init__('behavior_tree_navigator')

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create publisher for status updates
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)

        # Create timer for behavior execution
        self.behavior_timer = self.create_timer(1.0, self.execute_behavior)

        # Navigation state
        self.navigation_state = 'idle'  # idle, navigating, patrolling, waiting
        self.waypoints = [
            [1.0, 1.0, 0.0, 1.0],   # x, y, z, w (quaternion)
            [2.0, 0.0, 0.0, 1.0],
            [1.0, -1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
        self.current_waypoint_index = 0
        self.patrol_active = False

        # Wait for navigation server
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigation server...')

        self.get_logger().info('Behavior Tree Navigator initialized')

    def execute_behavior(self):
        """Execute navigation behaviors based on current state"""
        if self.navigation_state == 'idle':
            self.idle_behavior()
        elif self.navigation_state == 'navigating':
            self.navigation_behavior()
        elif self.navigation_state == 'patrolling':
            self.patrol_behavior()
        elif self.navigation_state == 'waiting':
            self.wait_behavior()

    def idle_behavior(self):
        """Idle behavior - wait for navigation commands"""
        # Check for external commands or triggers
        # For now, randomly start patrolling
        if random.random() < 0.01:  # 1% chance per second
            self.start_patrol()

    def navigation_behavior(self):
        """Handle ongoing navigation"""
        # This would normally check if navigation is complete
        # For simplicity, we'll just log the state
        self.get_logger().info('Navigation in progress...')

    def patrol_behavior(self):
        """Patrol behavior - navigate between waypoints"""
        if not self.waypoints:
            self.navigation_state = 'idle'
            return

        # Navigate to current waypoint
        target = self.waypoints[self.current_waypoint_index]
        self.navigate_to_pose(target)

        # Move to next waypoint after delay
        time.sleep(2)  # Wait 2 seconds after reaching waypoint
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)

    def wait_behavior(self):
        """Wait behavior - pause navigation"""
        self.get_logger().info('Navigation paused')
        # Wait behavior implementation

    def navigate_to_pose(self, pose):
        """Send navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = pose[2]
        goal_msg.pose.pose.orientation.w = pose[3]

        self.navigation_state = 'navigating'
        self.get_logger().info(f'Navigating to ({pose[0]}, {pose[1]})')

        # Send goal asynchronously
        self.nav_client.send_goal_async(goal_msg)

    def start_patrol(self):
        """Start patrol behavior"""
        if not self.patrol_active and len(self.waypoints) > 0:
            self.patrol_active = True
            self.navigation_state = 'patrolling'
            self.current_waypoint_index = 0
            self.get_logger().info('Patrol started')
            self.status_pub.publish(String(data='Patrol started'))

    def stop_patrol(self):
        """Stop patrol behavior"""
        self.patrol_active = False
        self.navigation_state = 'idle'
        self.get_logger().info('Patrol stopped')
        self.status_pub.publish(String(data='Patrol stopped'))


def main(args=None):
    rclpy.init(args=args)
    navigator = BehaviorTreeNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.stop_patrol()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 4: Waypoint Navigation Launch File

Create a launch file to bring up the complete navigation system:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default='turtlebot3_world.yaml')
    params_file = LaunchConfiguration('params_file', default='nav2_params.yaml')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_respawn = LaunchConfiguration('use_respawn', default='False')

    # Package names
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_robot_nav = get_package_share_directory('robot_navigation')

    # Navigation launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                FindPackageShare('robot_navigation'),
                'config',
                'bipedal_nav2_params.yaml'
            ]),
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn
        }.items()
    )

    # Bipedal-specific path planner
    bipedal_planner = Node(
        package='robot_navigation',
        executable='bipedal_path_planner',
        name='bipedal_path_planner',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_step_length': 0.3},
            {'min_turn_radius': 0.4},
            {'balance_margin': 0.1}
        ],
        output='screen'
    )

    # Behavior tree navigator
    behavior_navigator = Node(
        package='robot_navigation',
        executable='behavior_tree_navigator',
        name='behavior_tree_navigator',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Waypoint follower
    waypoint_follower = Node(
        package='robot_navigation',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'waypoint_radius': 0.5},  # Radius to consider waypoint reached
            {'linear_speed': 0.2},
            {'angular_speed': 0.4}
        ],
        output='screen'
    )

    # Navigation safety supervisor
    safety_supervisor = Node(
        package='robot_navigation',
        executable='safety_supervisor',
        name='safety_supervisor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'collision_threshold': 0.3},  # Minimum distance to obstacles
            {'stability_threshold': 5.0},  # Maximum time without stability
            {'emergency_stop_distance': 0.15}  # Distance to emergency stop
        ],
        output='screen'
    )

    # RViz2 for navigation visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_navigation'),
        'rviz',
        'nav2_default_view.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': PathJoinSubstitution([
                FindPackageShare('robot_navigation'),
                'maps',
                'humanoid_lab.yaml'
            ])},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower',
                'safety_supervisor'
            ]}
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='turtlebot3_world.yaml',
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='nav2_params.yaml',
            description='Full path to param file to load'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Whether to use composed bringup'
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes'
        ),
        nav2_bringup_launch,
        bipedal_planner,
        behavior_navigator,
        waypoint_follower,
        safety_supervisor,
        map_server,
        lifecycle_manager,
        rviz
    ])
```

## Exercise

### Required Exercise: Complete Navigation System for Bipedal Robot

Build a complete navigation system that can guide a bipedal robot through complex environments:

**Task**: Create a system with:
1. A Nav2 configuration tailored for bipedal robot constraints
2. A custom path planner that accounts for balance and gait limitations
3. A behavior tree navigator for complex navigation tasks
4. A safety supervisor to ensure stable locomotion

**Acceptance Criteria**:
- Navigation system operates safely with bipedal-specific constraints
- Path planner generates stable, energy-efficient paths for bipedal robots
- Robot successfully navigates between waypoints while avoiding obstacles
- System includes recovery behaviors for handling navigation failures

**Implementation Steps**:
1. Configure Nav2 with bipedal-specific parameters for your robot
2. Implement the custom path planner with balance and gait constraints
3. Create the behavior tree navigator with patrol and waypoint following
4. Build the safety supervisor with stability monitoring
5. Test the complete system in simulation with various navigation challenges

**Gazebo/Isaac Sim Fallback**: The system should work with both simulators:
- Use appropriate coordinate frames and transforms for each simulator
- Ensure proper integration with the SLAM system from previous chapters
- Validate that navigation parameters work across both simulation platforms

**Extension Challenges**:
- **Beginner**: Add visualization markers to show planned paths and navigation states
- **Intermediate**: Implement dynamic obstacle avoidance for moving obstacles
- **Advanced**: Create a learning system that adapts navigation parameters based on terrain feedback

## References

- [Navigation 2 (Nav2) Documentation](https://navigation.ros.org/)
- [ROS 2 Navigation Tutorials](https://navigation.ros.org/tutorials/)
- [A* Pathfinding Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Behavior Trees in Robotics](https://arxiv.org/abs/1709.00084)
- [Bipedal Robot Navigation](https://ieeexplore.ieee.org/document/9123456)
- [Path Planning for Humanoid Robots](https://link.springer.com/chapter/10.1007/978-3-030-13563-3_4)
