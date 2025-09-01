# 🛠️ ROS Catkin Workspace - Navigation Launch

This repository contains a ROS Catkin workspace set up for launching a navigation stack using a pre-defined 2D map (pgm & yaml). This guide will walk you through launching the system and visualizing it in RViz.

---

## 📁 Prerequisites

- [ROS (Robot Operating System)](https://www.ros.org/) installed (e.g., ROS Noetic for Ubuntu 20.04)
- Workspace properly built using `catkin_make` or `catkin build`
- A valid map and launch file (`map.launch`) inside the `my_nav` package

---

## 🚀 How to Run

1. Open a terminal and source your ROS setup:
    ```bash
    source /opt/ros/noetic/setup.bash
    ```

2. Navigate to your Catkin workspace:
    ```bash
    cd ~/catkin_ws
    ```

3. Source your workspace:
    ```bash
    source devel/setup.bash
    ```

4. Launch the navigation stack:
    ```bash
    roslaunch my_nav map.launch
    ```

---

## 🧭 Visualization with RViz

In a **new terminal**, run the following:

1. Launch RViz:
    ```bash
    rviz
    ```

2. Load the appropriate RViz configuration or manually add the topics:
    - Map
    - RobotModel
    - TF
    - LaserScan (if applicable)

---

## ✅ Tips

- Make sure all required packages are built:  
  ```bash
  catkin_make
