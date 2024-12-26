# Introduction to Robotics

Welcome to the **Introduction to Robotics** repository! This repository contains a series of robotics projects and implementations focused on controlling and localizing a robot in various scenarios. These projects are built around the Turtlebot 3 Waffle Pi robot and demonstrate concepts such as basic movement, pose-to-pose control, path following, and advanced localization using Bayesian tracking and Kalman filters.

## Table of Contents

1. [Robot Basic Movement](#1-robot-basic-movement)
2. [Pose-to-Pose Control](#2-pose-to-pose-control)
3. [Path Following](#3-path-following)
4. [Kalman Filter for Localization](#4-kalman-filter-for-localization)
5. [Final Project: Bayesian Tracking for Localization](#5-final-project-bayesian-tracking-for-localization)

---

## 1. Robot Basic Movement

This section contains code to control the robot's basic movements, such as:

- Moving forward.
- Turning left or right.

### Key Features:
- Simple linear and angular velocity control.
- Foundation for more complex navigation tasks.

### Files:
- `basic_movement.py`

---

## 2. Pose-to-Pose Control

In this section, the robot is given a target position and moves between poses using precise mathematical calculations.

### Key Features:
- The robot moves to the target position by traveling a straight line, followed by a turning maneuver.
- Calculations include:
  - Time required to go straight.
  - Turning rate.
  - Time required to turn.

### Files:
- `pose_to_pose_control.py`

---

## 3. Path Following

This section demonstrates path following using a PID controller. The robot executes a closed-circuit race course using input from a camera.

### Key Features:
- Implementation of a PID controller to correct deviations from the path.
- Closed-loop control for precise path execution.
- Use of camera data for dynamic adjustments.

### Files:
- `path_following.py`

---

## 4. Kalman Filter for Localization

This section implements an Extended Kalman Filter (EKF) for 1D localization.

### Key Features:
- Receives radar information as relative angles.
- Localizes the robot relative to obstacles.
- Handles intermittent radar information (i.e., localization continues even when radar data is unavailable for portions of time).

### Files:
- `kalman_filter_localization.py`

---

## 5. Final Project: Bayesian Tracking for Localization

The final project simulates a mail delivery route in a lab environment. The Turtlebot 3 Waffle Pi robot loops along a track and delivers to 12 different offices, represented by four different color patches (blue, orange, yellow, and green).

### Key Features:
- **Color Recognition:** The robot uses a front-mounted camera to recognize color patches.
- **Bayesian Tracking:** The robot maintains a belief about its location based on:
  - Measurement model.
  - State update model.
- **Line Following:** After sensing a color patch, the robot follows a line to the next office.
- **Localization:** Updates its belief until it is certain of the delivery office.

### Files:
- `bayesian_tracking_localization.py`

---

## Setup and Installation

### Prerequisites
- [Turtlebot 3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- ROS (Robot Operating System)
- Python 3.x
- Required Python packages:
  ```bash
  pip install numpy matplotlib
  ```

### Running the Code
1. Clone the repository:
   ```bash
   git clone https://github.com/angel-gao/Intro-to-Robotics.git
   ```
2. Navigate to the project directory:
   ```bash
   cd Intro-to-Robotics
   ```
3. Run the desired script:
   ```bash
   python <script_name>.py
   ```

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- Thanks to the University of Toronto Robotics Team for providing resources and guidance.
- Special thanks to the open-source community for tools and inspiration.

---

Happy coding! 🚀

