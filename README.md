# Humanoid-Robot-Simulation

## Overview

This project demonstrates the initial setup of a humanoid robot simulation using ROS 2 and Gazebo. The focus of Day 1 is to establish a reliable ROS–Gazebo pipeline by successfully spawning a humanoid base model in simulation and keeping it stable.

The project is designed as a step-by-step humanoid development pipeline, starting from a minimal robot description and progressively adding articulation, control, and interaction.

## Project Goals

- Set up a ROS 2 workspace for humanoid robot simulation
- Define a humanoid robot using URDF
- Spawn the robot in Gazebo via ROS 2
- Ensure the robot remains stable in simulation
- Provide a clean foundation for future joint control and interaction

## Environment

- **Platform**: The Construct (ROS Development Studio)
- **ROS Version**: ROS 2
- **Simulator**: Gazebo (Classic)
- **Build System**: colcon

## Repository Structure

```
ros2_ws/
└── src/
    └── humanoid_description/
        ├── urdf/
        │   └── humanoid.urdf
        ├── launch/
        │   └── humanoid_gazebo.launch.py
        ├── meshes/
        ├── worlds/
        ├── config/
        ├── CMakeLists.txt
        └── package.xml
```

## Humanoid Model Description

The robot is defined using URDF:

- Current model is a simplified humanoid base
- Single rigid body representing the torso
- Spawned as a static model to ensure stability
- The robot is published to ROS via robot_state_publisher

Although minimal, the model is treated as a robot, not a generic Gazebo object:

- It is described using URDF
- It is published through `/robot_description`
- It participates in the ROS TF pipeline
- It is spawned through ROS–Gazebo integration

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the Simulation

```bash
ros2 launch humanoid_description humanoid_gazebo.launch.py
```

## Verification Checklist

After launching:

- ✓ Gazebo opens successfully
- ✓ A box-shaped humanoid base appears in the simulation
- ✓ The robot remains stable (no falling or instability)
- ✓ `/robot_description` topic is available
- ✓ `robot_state_publisher` node is running

## Current Status

**Day 1 – Completed**

- ✓ ROS 2 workspace initialized
- ✓ Humanoid base model successfully spawned in Gazebo
- ✓ ROS–Gazebo pipeline validated

This stage represents a humanoid robot base model (MVP) rather than a fully articulated humanoid.

## Roadmap

### Day 2 (Planned)

- Add articulated joints (e.g., legs or arms)
- Introduce multiple links to achieve humanoid morphology
- Enable joint actuation

### Day 3 (Planned)

- Integrate joint controllers
- Enable user interaction (keyboard or higher-level commands)

## Key Takeaway

This project follows an **engineering-first approach**:

> Stability and infrastructure before motion and behavior.

By validating the simulation pipeline early, the project provides a solid foundation for advanced humanoid control and interaction.