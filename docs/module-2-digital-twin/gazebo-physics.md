---
sidebar_position: 9
title: "Simulating physics, gravity, and collisions in Gazebo"
---

# Simulating physics, gravity, and collisions in Gazebo

## Overview

Gazebo is a physics-based simulation environment that provides realistic modeling of robot dynamics, sensors, and environments. Understanding how Gazebo handles physics simulation is crucial for creating accurate and useful digital twins of humanoid robots.

## Physics Simulation Fundamentals

### Core Physics Concepts
- **Rigid Body Dynamics**: How objects move and interact
- **Collision Detection**: Identifying when objects make contact
- **Contact Response**: Calculating forces when objects collide
- **Gravity Simulation**: Realistic gravitational effects

### Physics Engines in Gazebo
- **ODE (Open Dynamics Engine)**: Default engine, good balance of speed and accuracy
- **Bullet**: Fast collision detection, good for complex scenarios
- **DART**: Advanced dynamics, good for articulated robots
- **Simbody**: High-fidelity simulation for complex systems

## Setting Up Physics in Gazebo

### World File Configuration
```
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Model Physics Properties
```
<model name="humanoid_robot">
  <link name="base_link">
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.1</iyy>
        <iyz>0.0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>

    <collision name="collision">
      <geometry>
        <box>
          <size>0.5 0.3 0.2</size>
        </box>
      </geometry>
    </collision>

    <visual name="visual">
      <geometry>
        <box>
          <size>0.5 0.3 0.2</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

## Gravity Simulation

### Configuring Gravity
- Standard Earth gravity: `<gravity>0 0 -9.8</gravity>`
- Different gravity values for other planets
- Custom gravity directions for testing

### Gravity Effects on Humanoid Robots
- Balance and stability challenges
- Walking pattern adjustments
- Center of mass considerations
- Joint load calculations

## Collision Detection and Response

### Collision Geometry Types
- **Box**: Simple rectangular shapes
- **Sphere**: Spherical objects
- **Cylinder**: Cylindrical shapes
- **Mesh**: Complex custom shapes
- **Plane**: Infinite flat surfaces

### Contact Materials
```
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
  </surface>
</collision>
```

## Humanoid-Specific Physics Considerations

### Balance and Stability
- Center of mass calculation
- Inertial tensor accuracy
- Joint damping and friction
- Foot contact modeling

### Walking Simulation
- Ground contact forces
- Friction parameters for traction
- Balance recovery strategies
- Dynamic stability margins

### Manipulation Physics
- Grasping force simulation
- Object interaction modeling
- Tool usage dynamics
- End-effector contact properties

## Advanced Physics Features

### Joint Dynamics
```
<joint name="hinge_joint" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.0</friction>
    </dynamics>
  </axis>
</joint>
```

### Sensor Integration
- IMU simulation with noise models
- Force/torque sensor simulation
- Joint position/velocity/effort feedback
- Contact sensor simulation

## Performance Optimization

### Simulation Accuracy vs. Speed
- Step size considerations
- Real-time factor adjustments
- Physics engine selection
- Model simplification strategies

### Computational Requirements
- CPU vs. GPU acceleration
- Parallel processing options
- Model complexity management
- Real-time constraints

## Validation and Tuning

### Comparing to Real Robots
- Motion capture validation
- Force sensor comparison
- Timing accuracy verification
- Behavior consistency checks

### Parameter Tuning Strategies
- System identification methods
- Iterative improvement process
- Sensitivity analysis
- Cross-validation techniques

## Best Practices

- Start with simple models and increase complexity
- Validate physics parameters against real robot data
- Use appropriate collision geometry for accuracy
- Implement proper damping for stable simulation
- Test with various scenarios and edge cases

## Summary

Physics simulation in Gazebo provides the foundation for realistic robot behavior in digital twins. For humanoid robots, accurate physics modeling is essential for testing balance, locomotion, and manipulation tasks safely and effectively. Proper configuration of physics parameters ensures that simulation results transfer well to real-world robot behavior.