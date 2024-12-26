# Introduction to Robotics

This repository contains a series of robotics labs and a final project which focused on controlling and localizing a robot in various scenarios. These projects are built around the Turtlebot 3 Waffle Pi robot and demonstrate concepts such as basic movement, pose-to-pose control, path following, and advanced localization using Bayesian tracking and Kalman filters.

- Equipments: Turtlebot 3 Waffle Pi and the operating system ROS.
- Main components: Raspberry Pi with an OpenCR board, a Raspberry Pi camera, a 360-degree lidar unit, an IMU, a compass and a gyroscope.
- Langurage: Python using rospy library

---

## [1. Robot Basic Movement](https://github.com/angel-gao/Intro-to-Robotics/tree/main/1.%20Robot%20Basic%20Movement/nodes)

This section contains code to control the robot's basic movements, such as:

- Moving forward.
- Turning left or right.

### Key Features:
- Simple linear and angular velocity control.
- Foundation for more complex navigation tasks.

---

## [2. Pose-to-Pose Control](https://github.com/angel-gao/Intro-to-Robotics/tree/main/2.%20Pose%20to%20Pose%20Control/nodes)

The robot is given a target position and moves between poses using precise mathematical calculations.

### Key Features:
- The robot moves to the target position by travelling in a straight line, followed by a turning maneuver.
- Calculations include time required to go straight, expected turning rate and time required to turn.

---

## [3. Path Following](https://github.com/angel-gao/Intro-to-Robotics/tree/main/3.%20Path%20Following/nodes)

This section demonstrates the path following using a PID controller. The robot executes a closed-circuit race course using input from a camera.

### Key Features:
- Implementation of a PID controller to correct deviations from the path.
- Closed-loop control for precise path execution.
- Use of camera data for dynamic adjustments.


---

## [4. Kalman Filter for Localization](https://github.com/angel-gao/Intro-to-Robotics/tree/main/4.%20Kalman%20Filter%20for%20Localization/nodes)

This section implements an Extended Kalman Filter (EKF) for robot 1D localization.

### Key Features:
- Receives radar information as relative angles to the landmark.
- Localizes the robot relative to obstacles.
- Handles intermittent radar information (i.e., localization continues even when radar data is unavailable for portions of time when obstacles present between robot and the landmark).

---

## [5. Final Project: Bayesian Tracking for Localization](https://github.com/angel-gao/Intro-to-Robotics/tree/main/Final_project-Mail%20Delivery%20Bot%20Bayesian%20Tracking/nodes)

The final project simulates a mail delivery route in a lab environment. The Turtlebot 3 Waffle Pi robot loops along a track and delivers to 12 different offices, represented by four different colour patches (blue, orange, yellow, and green). For detailed project process, please refer to the [Project Report](https://github.com/angel-gao/Intro-to-Robotics/blob/main/Final_project-Mail%20Delivery%20Bot%20Bayesian%20Tracking/project%20report.pdf). 

### Key Features:
- **Color Recognition:** The robot uses a front-mounted camera to recognize colour patches.
- **Bayesian Tracking:** The robot maintains a belief about its location based on the given measurement model and the state update model.
- **Line Following:** After sensing a color patch, the robot follows a white line to the next office.
- **Localization:** Updates its belief until it is certain of the delivery office.


---




