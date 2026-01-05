---
sidebar_position: 17
title: "Nav2 (path planning concepts, navigation for humanoid/bipedal robots)"
---

# Nav2 (path planning concepts, navigation for humanoid/bipedal robots)

## Overview

Navigation2 (Nav2) is the next-generation navigation stack for ROS 2, designed specifically for mobile robots. It provides advanced path planning, obstacle avoidance, and navigation capabilities that are essential for autonomous robot operation. For humanoid robots, Nav2 requires special considerations due to their unique locomotion and balance characteristics.

## Nav2 Architecture

### Core Components
```
+-------------------+
|   Navigation      |
|   Actions        |
+-------------------+
|   Task Servers   |
+-------------------+
|   Controllers    |
+-------------------+
|   Planners       |
+-------------------+
|   Smoother       |
+-------------------+
|   Collision      |
|   Checker        |
+-------------------+
|   Costmap        |
|   Layers        |
+-------------------+
```

### Main Modules
- **Navigation Actions**: High-level navigation commands
- **Global Planner**: Long-term path planning
- **Local Planner**: Short-term obstacle avoidance
- **Controller**: Robot motion control
- **Costmap**: Environmental representation
- **Recovery**: Behavior when navigation fails

## Path Planning Concepts

### Global Planner
The global planner creates a long-term path from the robot's current position to the goal:

```
# Example global planner configuration
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### Local Planner
The local planner handles short-term navigation and obstacle avoidance:

```
# Example local planner configuration
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller/MPPICtrl"
      time_steps: 50
      control_frequency: 20.0
      prediction_horizon: 1.0
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.3
```

## Costmap Configuration

### Costmap Layers
```
# Costmap configuration for Nav2
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_rollout_costs: true
      lethal_cost_threshold: 100
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: LaserScan
```

### 2D and 3D Costmaps
- **2D Costmap**: Traditional navigation with planar obstacles
- **3D Costmap**: Volumetric navigation for complex environments
- **Layered Approach**: Multiple sensor inputs combined

## Humanoid/Bipedal Robot Navigation

### Special Considerations

#### Balance and Stability
Humanoid robots have unique challenges for navigation:
- **Center of Mass**: Must maintain balance during movement
- **Step Planning**: Requires planning each foot placement
- **Dynamic Stability**: Walking requires continuous balance control
- **Support Polygon**: Maintaining stable foot positions

#### Bipedal Locomotion
```
# Humanoid-specific navigation parameters
humanoid_controller:
  ros__parameters:
    # Walking pattern parameters
    step_size: 0.3
    step_height: 0.05
    walking_speed: 0.5
    balance_margin: 0.1

    # Stability constraints
    max_tilt_angle: 15.0
    support_polygon_margin: 0.05
    zmp_stability_threshold: 0.02
```

### Gait Planning for Humanoid Robots
- **Footstep Planning**: Planning where to place each foot
- **ZMP Control**: Zero Moment Point for balance maintenance
- **Walking Patterns**: Different gaits for different speeds
- **Terrain Adaptation**: Adjusting steps for uneven terrain

### Humanoid-Specific Navigation Challenges

#### Turning and Rotation
- **Step-by-step turning**: Cannot rotate in place like wheeled robots
- **Pivot walking**: Special patterns for changing direction
- **Wide turns**: Need more space for turning maneuvers

#### Obstacle Negotiation
- **Step over obstacles**: Ability to step over small obstacles
- **Stair navigation**: Special algorithms for stairs
- **Narrow passages**: Requires precise foot placement

## Nav2 Configuration for Humanoid Robots

### Custom Plugins for Bipedal Navigation
```
# Example configuration for humanoid navigation
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
      sim_frequency: 20.0
      angle_thresh: 0.0
      angle_offset: 1.57
      right_to_left: false
    backup:
      plugin: "nav2_recoveries/BackUp"
      sim_frequency: 20.0
      backup_dist: -0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_recoveries/Wait"
      sim_frequency: 20.0
      wait_duration: 1.0
```

### Footprint and Robot Geometry
```
# Humanoid robot footprint configuration
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.3  # Conservative estimate for humanoid
      footprint_padding: 0.1
      # For non-circular robots, use polygon footprint
      # footprint: [[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]
```

## Advanced Navigation Features

### Human-Aware Navigation
```
# Human-aware navigation configuration
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer", "social_layer"]
      social_layer:
        plugin: "nav2_social_layer/SocialLayer"
        enabled: true
        observation_sources: people
        people:
          topic: /people_tracking
          max_obstacle_height: 2.0
          clearing: false
          marking: true
```

### Multi-Floor Navigation
- **Elevator Navigation**: Special handling for elevators
- **Stair Climbing**: Integration with stair climbing algorithms
- **Map Switching**: Seamless transition between floors
- **Localization**: Maintaining position across floors

## Integration with Isaac ROS

### GPU-Accelerated Navigation
```
# Isaac ROS Nav2 integration
isaac_nav2_planner:
  ros__parameters:
    use_gpu_planning: true
    acceleration_device: cuda
    max_planning_time: 0.1
    gpu_planner_type: "dijkstra"
    optimization_level: 2
```

### Perception Integration
- **3D Mapping**: Using Isaac ROS 3D perception
- **Dynamic Obstacles**: Real-time obstacle detection
- **Semantic Mapping**: Object-aware navigation

## Launch and Execution

### Basic Nav2 Launch
```
# Launch file for Nav2 with humanoid-specific settings
<launch>
  <!-- Map server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server">
    <param name="yaml_config_file" value="$(find-pkg-share my_robot_bringup)/config/map.yaml"/>
  </node>

  <!-- Local costmap -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="local_costmap">
    <param name="use_sim_time" value="True"/>
    <param name="robot_base_frame" value="base_link"/>
  </node>

  <!-- Global planner -->
  <node pkg="nav2_planner" exec="planner_server" name="planner_server">
    <param name="use_sim_time" value="True"/>
  </node>

  <!-- Controller -->
  <node pkg="nav2_controller" exec="controller_server" name="controller_server">
    <param name="use_sim_time" value="True"/>
  </node>

  <!-- Behavior server -->
  <node pkg="nav2_behaviors" exec="behavior_server" name="behavior_server">
    <param name="use_sim_time" value="True"/>
  </node>

  <!-- Lifecycle manager -->
  <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager">
    <param name="use_sim_time" value="True"/>
    <param name="autostart" value="True"/>
    <param name="node_names" value="[map_server, local_costmap, planner_server, controller_server, behavior_server]"/>
  </node>
</launch>
```

## Navigation Safety and Validation

### Safety Considerations
- **Emergency Stop**: Integration with robot safety systems
- **Speed Limiting**: Restricting speeds for stability
- **Area Restrictions**: No-go zones for safety
- **Human Safety**: Maintaining safe distances from humans

### Validation and Testing
- **Simulation Testing**: Validate in simulation first
- **Controlled Environments**: Test in safe environments
- **Gradual Complexity**: Increase difficulty gradually
- **Performance Monitoring**: Track navigation performance

## Troubleshooting and Debugging

### Common Issues
- **Local Minima**: Robot getting stuck in local minima
- **Oscillation**: Robot oscillating between states
- **Path Quality**: Poor path planning quality
- **Timing Issues**: Synchronization problems

### Debugging Tools
- **RViz Visualization**: Visualize navigation state
- **Costmap Visualization**: Check costmap representation
- **Path Analysis**: Analyze planned paths
- **Performance Monitoring**: Track computational performance

## Best Practices

### Configuration Best Practices
- Start with default parameters and tune gradually
- Use simulation to validate before real robot testing
- Monitor robot stability during navigation
- Test in various environments

### Humanoid-Specific Best Practices
- Account for robot's balance constraints
- Plan paths that allow for stable foot placement
- Consider robot's field of view for sensor coverage
- Test navigation at different walking speeds

### Performance Optimization
- Optimize costmap resolution for your robot
- Tune controller parameters for robot dynamics
- Use appropriate planning frequencies
- Monitor computational requirements

## Summary

Nav2 provides the foundation for autonomous navigation in ROS 2, with special considerations needed for humanoid robots due to their unique locomotion and balance requirements. By understanding path planning concepts, configuring appropriate parameters, and accounting for bipedal locomotion constraints, you can enable safe and effective navigation for humanoid robots. The integration with Isaac ROS provides additional capabilities for perception-driven navigation with GPU acceleration.