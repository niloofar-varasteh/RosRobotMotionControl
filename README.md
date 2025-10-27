# RosRobotMotionControl
UR10 robot motion control with Ros ,gazebo,ubunto
# Robot Motion Control in ROS (UR10 Manipulator)

This repository contains my implementation of the **Robot Motion Control in ROS** project  
The work focuses on **inverse kinematics**, **trajectory control**, and **ROS–MATLAB integration** for the **Universal Robot UR10** manipulator simulated in **Gazebo + RViz**.

---

##  Project Objectives

- Understand forward / inverse kinematics of industrial manipulators  
- Implement numerical **Inverse Kinematics (IK)** in MATLAB  
- Establish **ROS2 communication** between MATLAB and Gazebo  
- Control the **UR10 arm** through:
  - ROS *topic* interface (`/joint_trajectory_controller`)
  - ROS *action* interface (`/follow_joint_trajectory`)
- Visualize motion in **RViz** and **Gazebo**

---

##  System Requirements

| Component | Recommended Version |
|------------|--------------------|
| **Ubuntu** | 20.04 LTS |
| **ROS** | Noetic Ninjemys (or ROS 2 Humble via adaptation) |
| **MATLAB** | R2023a + Robotics System Toolbox + ROS Toolbox |
| **Simulator** | Gazebo 11 + RViz |
| **Robot Package** | `universal_robot` (UR10 model) |
| **Optional** | Virtual Machine (when running on Windows) |

---



