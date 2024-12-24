# Dropbear Simulation - Gazebo

Here I'm working on Dropbear's simulation in Gazebo. I wrote a few ROS nodes to handle joint trajectory control, and after some issues, I finally got the closed-loop chain joints working properly in Gazebo. Everything’s smooth now!

### Folder Structure Overview:
- **[`dropbear_detailed_urdf`](./dropbear_detailed_urdf)**: Contains the most granular and detailed simulation of Dropbear. Every part is modeled individually as a mesh, and each part is linked through the URDF using XACRO files. This provides a detailed simulation, allowing for accurate control of individual parts of the robot. This is where I’ll focus moving forward as it gives the most fine-grained control.
  
- **[`dropbear_simplified_urdf`](./dropbear_simplified_urdf)**: Contains a simplified version of the URDF with fewer parts. Here I focus on getting the basic structure of the robot working - Head, Torso, Pelvis, Legs and Hands as their own combined meshes.

- **[`dropbear-sim`](./dropbear-sim)**: Contains my current simulation of Dropbear in Gazebo.

### What's Working:
- ROS nodes for joint trajectory control are set up and running fine.
- Closed-loop chain joints fixed and now working in Gazebo!
  
### What's Next:
- Planning to bring this into Mujoco and Isaac Labs. Some joints like prismatic ones are still having translation issues, but it’s on the todo list.
- I'll experiment with more plugins and extensions to enhance functionality.

### How to Get It Running:

1. Build the project:
   ```bash
   colcon build
   source install/setup.bash
   ```

2. Launch the simulation:
    ```bash
    ros2 launch dropbear dropbear_controllers.launch.py
    ```
    Launches the controller nodes for Dropbear. These nodes manage how the joints of the robot respond to trajectory commands and control signals.

    ```bash
    ros2 launch dropbear dropbear_detailed_gazebo.launch.py
    ```
    Launches the Gazebo simulation environment for Dropbear model.

    ```bash
    ros2 launch dropbear dropbear_detailed_display.launch.py
    ```
    Displays the detailed robot model in Gazebo, including meshes and other visual elements for each part, based on the detailed URDF

    ```bash
    ros2 launch dropbear dropbear_sdf.launch.py
    ```
    Launches the SDF (Simulation Description Format) version of the robot model, providing an alternative to URDF for simulation of closed loop kinematics chain in Gazebo. Has some gazebo tags for certain types of joints, useful especially when dealing with different platforms.

3. Run the trajectory control nodes:
    ```bash
    ros2 run dropbear hand_trajectory_publisher.py
    ```
    Publishes joint trajectory commands for the hands of the robot, controlling how it moves according to user input.

    ```bash
    ros2 run dropbear pelvic_girdle_joints_trajectory_publisher.py
    ```
    Publishes joint trajectory commands for the pelvic girdle joints of the robot, managing the movement of the robot’s lower torso, controlling how it moves - according to user input.

    ```bash
    ros2 run dropbear leg_trajectory_publisher.py
    ```
    Publishes joint trajectory commands for the legs of the robot, controlling how it moves according to user input.

## Video Demo:
    Check out the demo video showing the simulation in action!
    [JointFixes_WalkSimTry2.mp4](./JointFixes_WalkSimTry2.mp4)