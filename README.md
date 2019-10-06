# UR5 Arm Pick and Place

Project on controlling a simulated robot to perform objects pick and place.
Inspired by Udacity's [Robotic arm - Pick & Place project](https://github.com/udacity/RoboND-Kinematics-Project)

## Overview

This is how the simulated environment is setup:

<p align="center">
     <img src="img/simulation.png" width="640">
</p>

The goal is to pick the object wich spawns on a random location on the table. A working Microsofot Kinect depth camera is also present.

## Usage

Here is a general overview on how the full simulation framework works:

<p align="center">
     <img src="img/ur5_gazebo.png" width="640">
</p>


**1- Start the simulation**: set ups the scene in gazebo and brings-up the robot arm.

```bash
roslaunch ur5_arm_gazebo ur5_grasping_world.launch 
```

**2- Start MOVEIT Planner**: starts the trajectory planner

```bash
roslaunch ur5_arm_moveit_config ur5_arm_planning_execution.launch 
```

**3- Start the pick and place commander**: starts the arm commander node plus the inverse kinematics solver and target spawner server nodes, follow the instructions in the terminal to start the grasping task.

```bash
roslaunch ur5_arm_moveit_config ur5_arm_planning_execution.launch 
```

## Contens

#### ROS Packages

* **ur5_arm**: ROS nodes used to perform control the simulated ur5 robot.

* **ur5_arm_gazebo**: Robot arm simulation world.

* **ur5_arm_description**: Custom UR5 robot URDF with gripper for the simulation environment.

* **gripper_description**: Custom gripper.

* **ur5_arm_moveit_config**: Package to control the UR5 robot using ROS MOVEIT!.

#### Jupyter Notebooks

* **InverseKinematicsUR5**: Forward and Inverse Kinematics theory and python implementation in sympy and numpy.

* **RobotSimulation**: Walkthrough on simulating and controlling a robot using MOVEIT! in ROS.

## Dependencies

* [ROS Robot Operating System](https://http://wiki.ros.org/kinetic)
* [Universal Robot ROS-INDUSTRIAL Package](https://github.com/ros-industrial/universal_robot)
