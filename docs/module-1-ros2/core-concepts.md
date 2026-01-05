---
sidebar_position: 2
title: "ROS 2 Core Concepts with Humanoid Examples"
---

# ROS 2 Core Concepts with Humanoid Examples

## Overview

This lesson provides an in-depth look at the core concepts of ROS 2 with specific examples and applications to humanoid robots. We'll explore how these concepts enable different systems in a humanoid robot to work together effectively.

## Nodes: The Building Blocks

Nodes in ROS 2 are individual processes that perform specific functions. In a humanoid robot, each major system typically runs as its own node:

### Humanoid Robot Node Examples

#### Joint Controller Node
- Controls individual joints in the robot's body
- Receives commands for desired joint positions
- Sends feedback about current joint states

#### Balance Control Node
- Processes IMU and force/torque sensor data
- Calculates corrective movements to maintain balance
- Sends commands to adjust joint positions

#### Vision Processing Node
- Receives image data from cameras in the robot's head
- Processes images to detect objects, people, or obstacles
- Publishes results for other nodes to use

#### High-Level Planning Node
- Receives high-level commands (e.g., "walk to the kitchen")
- Breaks them down into sequences of lower-level commands
- Coordinates with other nodes to execute the plan

## Topics: Continuous Communication Channels

Topics enable continuous data streaming between nodes. In humanoid robots, topics are used to share sensor data, motor commands, and other continuously updated information.

### Common Topic Examples in Humanoid Robots

#### Joint State Topic (`/joint_states`)
```yaml
header:
  stamp:
    sec: 12345
    nanosec: 67890
name: [head_yaw, head_pitch, left_shoulder_roll, left_shoulder_pitch, ...]
position: [0.1, 0.2, 1.5, -0.3, ...]  # Radians
velocity: [0.0, 0.0, 0.1, -0.2, ...]   # Radians per second
effort: [0.5, 0.3, 2.1, 1.8, ...]     # Nm
```

#### Camera Image Topic (`/head_camera/rgb/image_raw`)
- Streams images from the robot's head-mounted camera
- Used by perception nodes for object detection
- Used by navigation nodes for obstacle detection

#### IMU Data Topic (`/imu/data`)
- Provides orientation and acceleration data
- Critical for balance control algorithms
- Used by localization systems

#### Robot Velocity Topic (`/cmd_vel`)
- Receives velocity commands from navigation system
- Controls base movement for wheeled robots
- In bipedal robots, this might be transformed to footstep planning

## Services: Request-Response Communication

Services provide one-time request-response communication. These are used when a node needs to request specific information or perform a specific action that doesn't require continuous communication.

### Common Service Examples in Humanoid Robots

#### Map Saving Service (`/map_saver/save_map`)
- Request: Save the current map to a file
- Response: Success or failure status

#### Robot Reset Service (`/reset_robot`)
- Request: Reset all joint positions to default
- Response: Confirmation when reset is complete

#### Calibration Service (`/calibrate_sensors`)
- Request: Perform sensor calibration
- Response: Calibration results and status

## Actions: Long-Running Tasks

Actions are used for long-running tasks that require feedback and can be canceled. This is particularly important for humanoid robots performing complex behaviors.

### Action Examples in Humanoid Robots

#### Navigation Action (`/navigate_to_pose`)
- Goal: Navigate to a specific location
- Feedback: Current progress toward the goal
- Result: Success or failure when navigation completes
- Cancel: Ability to stop navigation if needed

#### Manipulation Action (`/move_group`)
- Goal: Move the robot's arm to a specific position
- Feedback: Current progress of the movement
- Result: Success or failure of the manipulation task
- Cancel: Ability to stop the movement if an obstacle is detected

## Parameter Server: Configuration Management

The parameter server stores configuration values that nodes need. In humanoid robots, these might include calibration values, joint limits, or behavior parameters.

### Common Parameters in Humanoid Robots

#### Joint Limits
- Maximum and minimum joint angles
- Maximum velocity and acceleration limits
- Torque limits for each joint

#### Robot Dimensions
- Link lengths and masses
- Center of mass information
- Foot size and support polygon

#### Controller Gains
- PID controller parameters for joint control
- Balance control parameters
- Walking gait parameters

## TF (Transform) System: Coordinate Management

The TF (Transform) system manages coordinate frames throughout the robot. This is crucial for humanoid robots with multiple degrees of freedom and sensors in different locations.

### TF Examples in Humanoid Robots

#### Robot Body Frames
- `base_link`: The robot's main reference frame
- `head_link`: Frame for the robot's head and cameras
- `left_hand`: Frame for the left hand/gripper
- `right_foot`: Frame for the right foot

#### Sensor Frames
- `laser_frame`: Where the LiDAR is mounted
- `imu_frame`: Where the IMU is located
- `camera_frame`: Where the camera is positioned

## Humanoid-Specific Communication Patterns

### Balance and Locomotion
- Sensor fusion nodes combine data from IMU, joint encoders, and force sensors
- Balance control nodes receive this data and compute corrective actions
- Walking pattern generators create footstep plans based on navigation goals

### Perception and Action
- Vision nodes process camera data and detect objects
- Object recognition results are shared via topics
- Manipulation nodes use this information to plan and execute grasping actions

### Human-Robot Interaction
- Audio input nodes process speech commands
- Natural language processing nodes interpret the commands
- Behavior trees coordinate the execution of complex interaction patterns

## Best Practices for Humanoid ROS 2 Systems

### Design Principles
1. **Modularity**: Keep each node focused on a single responsibility
2. **Standardization**: Use standard ROS 2 message types where possible
3. **Scalability**: Design for the addition of new sensors and capabilities
4. **Robustness**: Handle failures gracefully and provide appropriate error handling

### Performance Considerations
1. **Real-time Requirements**: Critical control loops (like balance) need high-frequency updates
2. **Resource Management**: Efficiently manage CPU and memory usage
3. **Network Usage**: Optimize communication between nodes
4. **Synchronization**: Ensure proper timing between different systems

## Summary

Understanding these core ROS 2 concepts is essential for developing humanoid robots. The node-based architecture, with its various communication patterns, provides a flexible framework for building complex robotic systems. Each concept has specific applications in humanoid robotics, from the low-level joint control to high-level planning and human-robot interaction.