# Mobile Robot Simulation with ROS 2 & Gazebo

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FIron-22314E?style=flat&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange?style=flat&logo=gazebo&logoColor=white)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-blue)
![Status](https://img.shields.io/badge/Status-Educational-green)

**A ROS 2 package for simulating a differential drive mobile robot ("Articubot One") in Gazebo, featuring classic path planning implementations.**

This repository serves as a study bench for understanding ROS 2 package structure, URDF/Xacro robot modeling, Gazebo integration, and the implementation of standard path planning algorithms (A*, RRT) in both C++ and Python.

---

## Table of Contents
- [Overview](#-overview)
- [Key Features](#-key-features)
- [Project Structure](#-project-structure)
- [Prerequisites](#-prerequisites)
- [Installation & Build](#-installation--build)
- [Usage](#-usage)
- [Path Planning Algorithms](#-path-planning-algorithms)
- [License](#-license)

---

## Overview

This project implements a complete simulation pipeline for a mobile robot. It includes the robot description (URDF), sensor simulation (Lidar), and the necessary launch files to spawn the robot in a Gazebo world. Additionally, it contains raw implementations of path planning algorithms to study their logic within a ROS environment.

**Goals of this repository:**
* To study the syntax and structure of ROS 2 packages (`package.xml`, `CMakeLists.txt`, `launch` files).
* To understand `xacro` for modular robot descriptions.
* To practice implementing navigation algorithms (A*, RRT) from scratch.

---

## Key Features

* ** Robot Modeling:** Full URDF/Xacro description of a differential drive robot including inertial macros and collision geometry.
* ** Sensor Simulation:** Integrated Lidar sensor simulation (`lidar.xacro`).
* ** Gazebo Worlds:** Custom environments (`ob_world.world`) for testing obstacle avoidance.
* ** Path Planning:** Standalone implementations of A* and RRT algorithms in `src/`.
* ** One-Command Launch:** Streamlined `launch` files to start simulation, localization, and navigation stacks.

---

## Project Structure

```plaintext
articubot_one/
├── config/                 # Configuration files (Nav2 params, RViz config, etc.)
├── description/            # Robot Model (URDF/Xacro)
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot.urdf.xacro    # Main robot description file
│   └── robot_core.xacro
├── launch/                 # ROS 2 Launch files
│   ├── launch_sim.launch.py
│   ├── localization_launch.py
│   ├── navigation_launch.py
│   └── rsp.launch.py       # Robot State Publisher launch
├── src/                    # Path Planning Source Code
│   ├── A_star.cpp
│   ├── A_star_python.py
│   ├── RRT.cpp
│   └── dynamic_RRT.cpp
├── worlds/                 # Gazebo simulation environments
│   ├── empty.world
│   └── ob_world.world      # World with obstacles
├── CMakeLists.txt
├── package.xml
└── README.md
