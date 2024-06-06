# Visualizing Dropbear's URDF in RViz2

Visualizing Dropbear in RViz2 using ROS 2. The package is called `humanoid_description`.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble


## Installation and Setup

### 1. Create a ROS 2 Workspace
Create a ROS 2 workspace for this project:

```sh
mkdir -p ~/humanoid_ws/
cd ~/humanoid_ws
```

### 2. Build the Workspace
Build the project using `colcon`:

```sh
colcon build
```

### 3. Source the Setup File
Source the setup file to overlay the workspace on your environment:

```sh
source ~/humanoid_ws/install/setup.bash
```

## Launch RViz2

Launch RViz2 visualization by using the provided launch file:

```sh
ros2 launch humanoid_description humanoid_rviz.launch.py
```

This will start RViz2 and load the configuration to visualize Dropbear.


## Cole's attempt:

### My partner, Cole is working on a full-blown web-based URDF manipulator app at: https://github.com/robit-man/URDF-Threejs-Modifier. The repositry also contains a version of Dropbear's URDF that you may use to manipulate and simulate Dropbear assembly ❤️