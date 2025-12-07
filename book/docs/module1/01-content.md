---
sidebar_position: 1
---

# Module 1: Advanced ROS 2 Concepts

In this module, we will explore some of the more advanced concepts in ROS 2. These concepts are essential for building complex and robust robotic systems.

## 1. ROS 2 Actions

Actions are used for long-running, preemptible tasks, such as navigating to a goal. They are similar to services, but they provide feedback on the progress of the task and can be cancelled.

An action consists of three parts:
- **Goal**: The request sent by the client to the server.
- **Feedback**: Periodic updates from the server to the client on the progress of the task.
- **Result**: The final result of the task, sent by the server to the client upon completion.

## 2. ROS 2 Launch Files

Launch files are used to start and configure a set of ROS 2 nodes. They are written in Python and allow you to:

- **Launch multiple nodes with a single command.**
- **Set parameters for your nodes.**
- **Remap topic names.**
- **Set up more complex scenarios, such as starting different nodes based on command-line arguments.**

## 3. URDF and Robot State Publisher

The Unified Robot Description Format (URDF) is an XML format for representing a robot model. It describes the robot's physical properties, such as its links, joints, and visual appearance.

The Robot State Publisher is a ROS 2 node that reads the URDF and publishes the state of the robot's joints to the `/tf` topic. This allows other nodes, such as RViz2, to visualize the robot's model.

## 4. The TF2 Transformation Library

TF2 is a library that lets you keep track of multiple coordinate frames over time. It is a powerful tool for robotics, as it simplifies the process of transforming data between different parts of the robot and the environment.

With TF2, you can ask questions like: "What is the pose of the robot's end-effector relative to its base?" or "What is the location of the object in the robot's coordinate frame?"
