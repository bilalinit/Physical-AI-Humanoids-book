---
sidebar_position: 8
title: 'Chapter 8: Navigation (Nav2 for Humanoids)'
description: 'Advanced navigation techniques using ROS 2 Navigation Stack specifically adapted for humanoid robots'
---

# Chapter 8: Navigation (Nav2 for Humanoids)

## Introduction

Navigation for humanoid robots presents unique challenges compared to wheeled robots due to their bipedal locomotion, balance requirements, and complex kinematics. This chapter explores the ROS 2 Navigation Stack (Nav2) and its adaptation for humanoid robots, covering path planning, obstacle avoidance, and locomotion control specifically tailored for bipedal systems.

## Understanding Nav2 Architecture

### Core Components

Nav2 consists of several key components that work together:

1. **Navigation System**: Coordinates the overall navigation process
2. **Global Planner**: Creates optimal paths from start to goal
3. **Local Planner**: Executes short-term navigation and obstacle avoidance
4. **Controller**: Interfaces with robot hardware to execute motion commands
5. **Costmap**: Represents obstacles and free space in the environment

### Nav2 for Humanoid Robots

Humanoid robots require specialized navigation approaches due to:

- Bipedal locomotion dynamics
- Balance and stability constraints
- Multi-contact navigation (stairs, slopes)
- Higher center of mass affecting stability

## Global Path Planning

### A* and Dijkstra Algorithms

For humanoid robots, path planning must consider:

- Step location constraints
- Balance maintenance
- Joint angle limitations
- Dynamic stability margins

```yaml
# Global planner configuration for humanoid robots
global_planner:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  tolerance: 0.5  # Allow for more tolerance due to humanoid step constraints
  use_astar: true
  allow_unknown: true

# Humanoid-specific parameters
humanoid_path_planner:
  step_size: 0.3  # Maximum step size for humanoid
  foot_placement_margin: 0.1  # Safety margin for foot placement
  balance_threshold: 0.05  # Maximum CoM deviation allowed
```

### Footstep Planning

```cpp
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.h"

class FootstepPlanner : public nav2_core::GlobalPlanner
{
public:
  void createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::vector<geometry_msgs::msg::PoseStamped> & plan) override
  {
    // Generate footstep locations instead of continuous path
    std::vector<Footstep> footsteps = generateFootsteps(start, goal);

    // Convert footsteps to pose plan
    plan = convertFootstepsToPlan(footsteps);
  }

private:
  std::vector<Footstep> generateFootsteps(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    std::vector<Footstep> footsteps;

    // Implement humanoid-specific footstep planning
    // considering balance constraints and step size limits

    return footsteps;
  }
};
```

## Local Navigation and Obstacle Avoidance

### Humanoid-Specific Local Planner

```python
class HumanoidLocalPlanner:
    def __init__(self):
        self.balance_controller = BalanceController()
        self.step_generator = StepGenerator()
        self.collision_checker = CollisionChecker()

    def compute_velocity_commands(self, robot_pose, goal_pose, local_costmap):
        """Compute velocity commands for humanoid robot"""
        # Check if robot is balanced
        if not self.balance_controller.is_balanced():
            return self.balance_recovery_velocity()

        # Generate safe footsteps
        safe_steps = self.generate_safe_steps(
            robot_pose, goal_pose, local_costmap
        )

        # Check for collision-free path
        if not self.collision_checker.is_path_clear(safe_steps):
            return self.obstacle_avoidance_velocity(robot_pose, local_costmap)

        # Generate step commands
        step_cmd = self.step_generator.generate_step_command(
            safe_steps[0]  # Next step
        )

        return step_cmd

    def generate_safe_steps(self, robot_pose, goal_pose, local_costmap):
        """Generate safe footsteps considering balance and obstacles"""
        # Consider robot's current balance state
        current_support_polygon = self.balance_controller.get_support_polygon()

        # Generate candidate steps within support polygon
        candidate_steps = self.step_generator.generate_candidates(
            robot_pose, current_support_polygon
        )

        # Filter for collision-free steps
        safe_steps = []
        for step in candidate_steps:
            if self.is_step_safe(step, local_costmap):
                safe_steps.append(step)

        return safe_steps

    def is_step_safe(self, step, local_costmap):
        """Check if a step is safe considering balance and collisions"""
        # Check collision in costmap
        if not self.collision_checker.is_collision_free(step, local_costmap):
            return False

        # Check balance after step
        if not self.balance_controller.will_be_balanced_after_step(step):
            return False

        return True
```

### Dynamic Window Approach for Humanoids

```python
class HumanoidDWA:
    def __init__(self):
        self.max_step_length = 0.4  # Maximum step length
        self.max_step_width = 0.2   # Maximum step width
        self.balance_margin = 0.05  # Balance safety margin

    def calculate_velocity(self, robot_state, goal, obstacles):
        """Calculate optimal step velocity for humanoid"""
        # Define feasible step space
        feasible_steps = self.generate_feasible_steps(robot_state)

        best_step = None
        best_score = float('-inf')

        for step in feasible_steps:
            # Calculate trajectory score
            score = self.evaluate_step(step, robot_state, goal, obstacles)

            if score > best_score:
                best_score = score
                best_step = step

        return best_step

    def generate_feasible_steps(self, robot_state):
        """Generate feasible steps based on humanoid kinematics"""
        steps = []

        # Generate steps within kinematic limits
        for step_length in np.arange(0.1, self.max_step_length, 0.05):
            for step_width in np.arange(-self.max_step_width,
                                       self.max_step_width, 0.05):
                step = {
                    'length': step_length,
                    'width': step_width,
                    'angle': 0.0  # For now, straight steps
                }

                # Check if step is kinematically feasible
                if self.is_kinematically_feasible(step, robot_state):
                    steps.append(step)

        return steps

    def evaluate_step(self, step, robot_state, goal, obstacles):
        """Evaluate a step based on multiple criteria"""
        # Distance to goal (should be decreasing)
        goal_dist_score = -self.calculate_distance_to_goal(step, goal)

        # Obstacle clearance
        obstacle_score = self.calculate_obstacle_clearance(step, obstacles)

        # Balance stability
        balance_score = self.calculate_balance_stability(step, robot_state)

        # Step efficiency
        efficiency_score = self.calculate_step_efficiency(step)

        # Weighted combination
        total_score = (0.4 * goal_dist_score +
                      0.3 * obstacle_score +
                      0.2 * balance_score +
                      0.1 * efficiency_score)

        return total_score
```

## Costmap Configuration for Humanoids

### Multi-Layer Costmap

Humanoid robots need specialized costmap layers:

```yaml
# Local costmap configuration for humanoid
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  width: 10.0
  height: 10.0
  resolution: 0.05
  origin_x: -5.0
  origin_y: -5.0

  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: balance_layer, type: "humanoid_navigation::BalanceLayer"}

  # Humanoid-specific inflation parameters
  inflation_layer:
    inflation_radius: 0.5  # Larger for humanoid safety
    cost_scaling_factor: 2.0
    inflate_unknown: false

  # Balance constraint layer
  balance_layer:
    enabled: true
    support_polygon_margin: 0.1  # Margin around support polygon
```

### Balance Constraint Layer

```cpp
class BalanceLayer : public nav2_costmap_2d::Layer
{
public:
  void updateBounds(
    double origin_x, double origin_y, double origin_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y) override
  {
    // Update bounds based on balance constraints
    *min_x = std::min(*min_x, robot_pose_.x - balance_margin_);
    *min_y = std::min(*min_y, robot_pose_.y - balance_margin_);
    *max_x = std::max(*max_x, robot_pose_.x + balance_margin_);
    *max_y = std::max(*max_y, robot_pose_.y + balance_margin_);
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j) override
  {
    // Mark areas that would cause balance issues
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        double world_x, world_y;
        master_grid.mapToWorld(i, j, world_x, world_y);

        if (!isBalanceMaintainable(world_x, world_y)) {
          master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }

private:
  bool isBalanceMaintainable(double x, double y) {
    // Check if robot can maintain balance at this location
    // based on current support polygon and CoM position
    return true; // Simplified for example
  }

  double balance_margin_{0.1};
  geometry_msgs::msg::Pose robot_pose_;
};
```

## Navigation Behaviors for Humanoids

### Stair Navigation

```python
class StairNavigation:
    def __init__(self):
        self.stair_detector = StairDetector()
        self.step_planner = StepPlanner()

    def navigate_stairs(self, stair_info):
        """Navigate stairs using specialized gait"""
        # Detect stair parameters
        step_height = stair_info['height']
        step_depth = stair_info['depth']
        num_steps = stair_info['count']

        # Plan stair-specific footsteps
        stair_steps = self.plan_stair_footsteps(
            step_height, step_depth, num_steps
        )

        # Execute stair climbing with modified gait
        for step in stair_steps:
            self.execute_stair_step(step)

    def plan_stair_footsteps(self, height, depth, count):
        """Plan footsteps for stair navigation"""
        footsteps = []

        for i in range(count):
            # Alternate feet for stair climbing
            left_foot = (i * depth, (i + 1) * height, 0)
            right_foot = ((i + 0.5) * depth, i * height, 0)

            footsteps.append({
                'position': left_foot if i % 2 == 0 else right_foot,
                'type': 'stair'
            })

        return footsteps
```

### Slope Navigation

```python
class SlopeNavigation:
    def __init__(self):
        self.slope_angle_threshold = 15.0  # degrees

    def navigate_slope(self, slope_angle):
        """Navigate slopes with adjusted gait"""
        if abs(slope_angle) > self.slope_angle_threshold:
            # Too steep, find alternative route
            return self.find_alternative_path()

        # Adjust gait parameters for slope
        adjusted_gait = self.adjust_gait_for_slope(slope_angle)

        # Execute navigation with adjusted parameters
        return self.execute_with_gait(adjusted_gait)

    def adjust_gait_for_slope(self, slope_angle):
        """Adjust gait parameters based on slope"""
        gait_params = {
            'step_length': 0.3 if slope_angle > 0 else 0.25,  # Shorter steps downhill
            'step_height': 0.05 + abs(slope_angle) * 0.01,   # Higher steps for slope
            'swing_height': 0.1 + abs(slope_angle) * 0.02,   # Higher swing for slope
            'balance_margin': 0.05 + abs(slope_angle) * 0.001
        }

        return gait_params
```

## Integration with Humanoid Control

### Balance Controller Integration

```python
class NavigationBalanceIntegrator:
    def __init__(self):
        self.balance_controller = BalanceController()
        self.navigation_controller = HumanoidLocalPlanner()

    def integrate_control(self, nav_cmd, robot_state):
        """Integrate navigation and balance control"""
        # Process navigation command
        step_cmd = self.navigation_controller.process_command(nav_cmd)

        # Check balance constraints
        if not self.balance_controller.is_stable(step_cmd, robot_state):
            # Adjust step command for balance
            step_cmd = self.balance_controller.adjust_for_balance(
                step_cmd, robot_state
            )

        # Execute integrated command
        self.execute_step_with_balance(step_cmd, robot_state)

    def execute_step_with_balance(self, step_cmd, robot_state):
        """Execute step while maintaining balance"""
        # Calculate Zero Moment Point (ZMP)
        zmp = self.balance_controller.calculate_zmp(robot_state)

        # Check ZMP within support polygon
        if not self.balance_controller.is_zmp_stable(zmp):
            # Execute balance recovery
            recovery_cmd = self.balance_controller.generate_recovery_command()
            self.execute_balance_recovery(recovery_cmd)
        else:
            # Execute normal step
            self.execute_step(step_cmd)
```

## Navigation Parameters Tuning

### Parameter Configuration

```yaml
# Humanoid navigation parameters
humanoid_navigation:
  planner_frequency: 5.0
  controller_frequency: 20.0  # Higher for balance control

  # Footstep planning parameters
  step_size_limits:
    max_forward: 0.4
    max_backward: 0.2
    max_lateral: 0.2
    max_rotation: 0.3

  # Balance parameters
  balance:
    com_height: 0.85  # Center of mass height
    support_polygon_margin: 0.05
    zmp_threshold: 0.02

  # Safety parameters
  safety:
    obstacle_clearance: 0.3
    balance_margin: 0.1
    recovery_timeout: 5.0
```

### Adaptive Parameter Tuning

```python
class AdaptiveParameterTuner:
    def __init__(self):
        self.base_params = self.load_base_parameters()
        self.terrain_classifier = TerrainClassifier()

    def adapt_parameters(self, environment_type, robot_state):
        """Adapt navigation parameters based on environment"""
        params = self.base_params.copy()

        if environment_type == 'indoor':
            params['step_size'] = 0.35
            params['balance_margin'] = 0.05
        elif environment_type == 'outdoor_rough':
            params['step_size'] = 0.25
            params['balance_margin'] = 0.1
            params['obstacle_clearance'] = 0.5
        elif environment_type == 'stairs':
            params['step_size'] = 0.2
            params['step_height'] = 0.17
            params['balance_margin'] = 0.15

        # Further adapt based on robot state
        if robot_state['battery_level'] < 0.2:
            params['step_size'] *= 0.8  # Smaller, safer steps
            params['speed_factor'] = 0.7

        return params
```

## Performance Evaluation

### Navigation Metrics

Key metrics for humanoid navigation:

1. **Success Rate**: Percentage of successful navigation attempts
2. **Path Efficiency**: Ratio of actual path length to optimal path
3. **Balance Maintenance**: Percentage of time robot maintains balance
4. **Step Accuracy**: Deviation from planned footsteps
5. **Computational Load**: CPU and memory usage during navigation

### Testing Scenarios

```python
def test_humanoid_navigation(navigation_system):
    """Test navigation system with various scenarios"""
    test_scenarios = [
        {
            'name': 'straight_line',
            'path': [(0, 0), (5, 0)],
            'obstacles': []
        },
        {
            'name': 'obstacle_avoidance',
            'path': [(0, 0), (5, 0)],
            'obstacles': [{'type': 'box', 'position': (2.5, 0), 'size': (0.5, 2.0)}]
        },
        {
            'name': 'narrow_passage',
            'path': [(0, 0), (5, 0)],
            'obstacles': [
                {'type': 'wall', 'position': (2.5, 0.5), 'size': (5.0, 0.1)},
                {'type': 'wall', 'position': (2.5, -0.5), 'size': (5.0, 0.1)}
            ]
        },
        {
            'name': 'slope_navigation',
            'path': [(0, 0), (5, 0)],
            'slope': 10.0  # 10 degree incline
        }
    ]

    results = {}
    for scenario in test_scenarios:
        result = run_navigation_test(navigation_system, scenario)
        results[scenario['name']] = result

    return results
```

## Challenges and Solutions

### Balance vs. Navigation Trade-offs

Balancing navigation efficiency with stability:

```python
class BalanceNavigationBalancer:
    def __init__(self):
        self.balance_weight = 0.6
        self.navigation_weight = 0.4

    def compute_control(self, balance_score, navigation_score):
        """Balance between navigation and balance objectives"""
        # Weighted combination of scores
        total_score = (self.balance_weight * balance_score +
                      self.navigation_weight * navigation_score)

        if balance_score < self.critical_balance_threshold:
            # Prioritize balance recovery
            return self.balance_recovery_control()
        else:
            # Normal navigation with balance consideration
            return self.navigation_control(balance_score, navigation_score)
```

### Multi-Contact Navigation

Handling stairs, slopes, and uneven terrain:

```python
class MultiContactNavigator:
    def __init__(self):
        self.contact_estimators = ContactEstimator()
        self.multi_contact_planner = MultiContactPlanner()

    def plan_multi_contact_path(self, start, goal, terrain_info):
        """Plan path considering multiple contact points"""
        # Analyze terrain for contact points
        contact_points = self.contact_estimators.estimate_contacts(terrain_info)

        # Plan path with multiple contact constraints
        path = self.multi_contact_planner.plan_path(
            start, goal, contact_points
        )

        return path
```

## Navigation Configuration Examples

### 1. Complete Nav2 Configuration for Humanoid Robots

Here's a complete configuration file for Nav2 specifically adapted for humanoid robots:

```yaml
# Navigation configuration for humanoid robots
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the behavior tree XML files are located
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Specify the path where the other BT node plugin XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_navigate_through_poses:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the behavior tree XML files are located
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Specify the path where the other BT node plugin XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node
    - nav2_is_battery_low_condition_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    # Humanoid-specific controller parameters
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific follow path controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 1000
      vx_std: 0.2
      vy_std: 0.25
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      wz_max: 0.4
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      publish_trajectories: False
      # Humanoid-specific cost function weights
      costmap_weight: 1.0
      goal_weight: 2.0
      obstacle_weight: 5.0
      # Balance-related weights for humanoid
      balance_weight: 3.0
      step_constraint_weight: 2.5

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific obstacle inflation
      robot_radius: 0.4  # Larger radius for humanoid stability margin
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Higher cost scaling for safety
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        max_obstacle_height: 2.0
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        unknown_threshold: 15
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
  local_costmap_client:
    ros__parameters:
      use_sim_time: False
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.4
      resolution: 0.05
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
        map_topic: /map
        transform_tolerance: 0.05
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
  global_costmap_client:
    ros__parameters:
      use_sim_time: False
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      # Humanoid-specific path planner
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Allow more tolerance for humanoid step constraints
      use_astar: true
      allow_unknown: true
      # Humanoid-specific parameters for path smoothing
      step_size: 0.025
      min_distance_from_robot: 0.3

smoother_server:
  ros__parameters:
    use_sim_time: False
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors/Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_dist: 0.15
      backup_speed: 0.05
    wait:
      plugin: "nav2_behaviors/Wait"
      wait_duration: 1s
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
      enabled: true
      xy_goal_tolerance: 0.05
      xy_vel_limit: 0.2
      yaw_goal_tolerance: 0.1

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### 2. Humanoid-Specific Navigation Launch File

```xml
<launch>
  <!-- Launch file for humanoid navigation with Nav2 -->

  <!-- Arguments -->
  <arg name="namespace" default=""/>
  <arg name="use_sim_time" default="false"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share my_humanoid_navigation)/config/nav2_params_humanoid.yaml"/>
  <arg name="default_bt_xml_filename" default="$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml"/>
  <arg name="map_subscribe_transient_local" default="false"/>

  <!-- Nodes -->
  <group>
    <push-ros-namespace namespace="$(var namespace)"/>

    <node pkg="nav2_controller" exec="controller_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
    </node>

    <node pkg="nav2_planner" exec="planner_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
    </node>

    <node pkg="nav2_smoother" exec="smoother_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
    </node>

    <node pkg="nav2_behaviors" exec="behavior_server" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
    </node>

    <node pkg="nav2_bt_navigator" exec="bt_navigator" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
      <param name="default_bt_xml_filename" value="$(var default_bt_xml_filename)"/>
    </node>

    <node pkg="nav2_waypoint_follower" exec="waypoint_follower" output="screen">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="params_file" value="$(var params_file)"/>
    </node>

    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_navigation">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="autostart" value="$(var autostart)"/>
      <param name="node_names" value="[controller_server, planner_server, smoother_server, behavior_server, bt_navigator, waypoint_follower]"/>
    </node>
  </group>
</launch>
```

### 3. Footstep Planner Configuration

```yaml
# Footstep planner configuration for humanoid navigation
footstep_planner:
  ros__parameters:
    use_sim_time: false
    # Step constraints for humanoid robot
    max_step_length: 0.3      # Maximum forward step
    max_step_width: 0.2       # Maximum lateral step
    max_step_height: 0.1      # Maximum step-up height
    min_step_length: 0.05     # Minimum forward step
    min_step_width: 0.05      # Minimum lateral step
    step_rotation: 0.26       # Max rotation per step (15 degrees)

    # Footprint and safety parameters
    foot_length: 0.25         # Length of foot in meters
    foot_width: 0.15          # Width of foot in meters
    safety_margin: 0.05       # Safety margin around foot

    # Balance constraints
    com_height: 0.8           # Center of mass height
    max_com_deviation: 0.1    # Maximum CoM deviation from support polygon
    support_polygon_buffer: 0.05  # Buffer for support polygon

    # Planning parameters
    max_planning_time: 5.0    # Maximum time to plan footstep path
    max_iterations: 1000      # Maximum iterations for planning
    resolution_xy: 0.05       # XY resolution for footstep grid
    resolution_yaw: 0.1       # Yaw resolution for footstep planning
```

### 4. Navigation Safety Monitor

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation as R

class NavigationSafetyMonitor(Node):
    def __init__(self):
        super().__init__('navigation_safety_monitor')

        # Parameters
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('balance_threshold', 0.2)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 0.3)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.balance_threshold = self.get_parameter('balance_threshold').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_input', self.cmd_vel_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odom, '/odom', self.odom_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_status_pub = self.create_publisher(Bool, '/navigation_safety_status', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_cmd_vel = Twist()
        self.obstacle_distances = []
        self.current_odom = None
        self.safety_engaged = False

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Navigation safety monitor initialized')

    def cmd_vel_callback(self, msg):
        """Receive navigation commands and apply safety filtering"""
        self.current_cmd_vel = msg

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Filter out invalid ranges
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.obstacle_distances = valid_ranges

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_odom = msg

    def safety_check(self):
        """Perform safety checks and modify commands if needed"""
        # Check for obstacles
        if self.obstacle_distances:
            min_distance = min(self.obstacle_distances) if self.obstacle_distances else float('inf')

            if min_distance < self.safety_distance:
                # Reduce velocity based on proximity to obstacles
                reduction_factor = min_distance / self.safety_distance
                self.current_cmd_vel.linear.x *= reduction_factor
                self.current_cmd_vel.linear.y *= reduction_factor
                self.safety_engaged = True
            else:
                self.safety_engaged = False
        else:
            # No scan data, stop for safety
            self.current_cmd_vel.linear.x = 0.0
            self.current_cmd_vel.linear.y = 0.0
            self.safety_engaged = True

        # Check balance if orientation data is available
        if self.current_odom:
            orientation = self.current_odom.pose.pose.orientation
            # Convert quaternion to roll/pitch for balance check
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            rpy = self.quaternion_to_rpy(quat)

            # Check if roll or pitch exceeds balance threshold
            if abs(rpy[0]) > self.balance_threshold or abs(rpy[1]) > self.balance_threshold:
                # Stop and trigger balance recovery
                self.current_cmd_vel.linear.x = 0.0
                self.current_cmd_vel.angular.z = 0.0
                self.safety_engaged = True

        # Limit velocities to safe values
        self.current_cmd_vel.linear.x = max(min(self.current_cmd_vel.linear.x, self.max_linear_vel), -self.max_linear_vel)
        self.current_cmd_vel.linear.y = max(min(self.current_cmd_vel.linear.y, self.max_linear_vel), -self.max_linear_vel)
        self.current_cmd_vel.angular.z = max(min(self.current_cmd_vel.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = not self.safety_engaged
        self.safety_status_pub.publish(safety_msg)

        # Publish filtered command
        self.cmd_vel_pub.publish(self.current_cmd_vel)

    def quaternion_to_rpy(self, quat):
        """Convert quaternion to roll-pitch-yaw"""
        r = R.from_quat(quat)
        return r.as_euler('xyz')

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = NavigationSafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Navigation Tuning Guidelines

When configuring navigation for humanoid robots, consider these tuning guidelines:

1. **Velocity Limits**: Set conservative velocity limits to maintain balance
   - Linear velocity: 0.1-0.5 m/s for safe bipedal locomotion
   - Angular velocity: 0.1-0.4 rad/s for smooth turning

2. **Costmap Parameters**: Adjust inflation and obstacle handling for humanoid safety
   - Increase robot radius to account for stability margins
   - Use higher cost scaling factors for safer path planning

3. **Tolerance Settings**: Allow more tolerance for humanoid step constraints
   - Position tolerance: 0.2-0.5m (larger than wheeled robots)
   - Yaw tolerance: 0.2-0.3 rad (allowable heading error)

4. **Controller Parameters**: Tune for balance-aware navigation
   - Increase weights for obstacle avoidance
   - Add balance-related cost terms in the controller

## Summary

Navigation for humanoid robots requires specialized approaches that consider the unique challenges of bipedal locomotion, balance maintenance, and complex kinematics. By adapting the Nav2 framework with humanoid-specific planners, controllers, and safety constraints, we can achieve robust navigation capabilities. The key is balancing navigation efficiency with stability, using specialized algorithms for footstep planning, and integrating closely with balance control systems to ensure safe and effective navigation in diverse environments.