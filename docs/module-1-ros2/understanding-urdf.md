---
sidebar_position: 5
title: "Understanding URDF (Unified Robot Description Format) for humanoids"
---

# Understanding URDF (Unified Robot Description Format) for humanoids

## Overview

URDF (Unified Robot Description Format) is an XML format that describes robot properties:
- Physical structure and joint connections
- Kinematic properties
- Visual and collision models
- Inertial properties

For humanoid robots, URDF is particularly important as it describes the complex multi-limb structure that mimics human anatomy.

## URDF Structure for Humanoid Robots

A humanoid robot URDF typically includes:
- **Base link**: Usually the torso or pelvis
- **Limb chains**: Arms and legs with multiple joints
- **End effectors**: Hands and feet
- **Sensors**: Cameras, IMUs, force sensors

### Basic URDF Components

```
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="torso_head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.7" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Humanoid-Specific Considerations

### Multi-Limb Structure
Humanoid robots have complex kinematic chains:
- **Torso**: Central body with head, arms, and legs attached
- **Arms**: Shoulder, elbow, and wrist joints for manipulation
- **Legs**: Hip, knee, and ankle joints for locomotion
- **End effectors**: Hands for manipulation, feet for support

### Balance and Stability
- Center of mass considerations
- Inertial properties for stable locomotion
- Joint limits for safe operation

### Sensor Integration
- Mounting positions for cameras and IMUs
- Force/torque sensors in joints
- Tactile sensors in hands and feet

## URDF for Humanoid Locomotion

### Joint Types for Humanoid Movement
- **Revolute joints**: Rotational joints (knees, elbows)
- **Continuous joints**: Unlimited rotation (shoulders, hips)
- **Fixed joints**: Rigid connections between components

### Example: Simple Leg Structure
```
<!-- Left Leg -->
<joint name="left_hip_joint" type="revolute">
  <parent link="base_link"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<link name="left_thigh">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.05"/>
    </geometry>
  </visual>
</link>

<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="2.35" effort="100" velocity="1"/>
</joint>

<link name="left_shin">
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.05"/>
    </geometry>
  </visual>
</link>
```

## Tools for URDF Development

### rviz
- Visualize robot models in 3D
- Check joint limits and kinematics
- Verify collision geometry

### robot_state_publisher
- Publishes forward kinematics
- Updates robot pose in real-time

### xacro
- Macro language for URDF
- Allows parameterization and reuse
- Reduces redundancy in complex models

### Example using xacro:
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <xacro:property name="M_PI" value="3.14159"/>

  <xacro:macro name="simple_leg" params="prefix reflect">
    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_thigh"/>
      <origin xyz="0 ${reflect * -0.1} -0.1" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
    <!-- Additional leg components -->
  </xacro:macro>

  <xacro:simple_leg prefix="left" reflect="1"/>
  <xacro:simple_leg prefix="right" reflect="-1"/>
</robot>
```

## Best Practices for Humanoid URDF

- Use consistent naming conventions
- Include proper inertial properties for simulation
- Define appropriate collision geometry
- Consider kinematic constraints for safe operation
- Validate with forward kinematics tools
- Test in simulation before physical implementation

## Real-World Applications

URDF is used extensively in humanoid robotics:
- **Simulation**: Testing locomotion and manipulation in Gazebo
- **Control**: Forward and inverse kinematics calculations
- **Visualization**: Displaying robot state in rviz
- **Planning**: Motion planning algorithms

## Summary

URDF is the standard for describing robot geometry and kinematics in ROS. For humanoid robots, it's essential for representing the complex multi-limb structure needed for human-like movement. Understanding URDF is crucial for developing, simulating, and controlling humanoid robots.