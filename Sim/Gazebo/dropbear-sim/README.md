# Dropbear Simulation - Gazebo

Here I'm working on Dropbear's simulation in Gazebo. I wrote a few ROS nodes to handle joint trajectory control, and after some issues, I finally got the closed-loop chain joints working properly in Gazebo. Everything’s smooth now!

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
    ros2 launch dropbear dropbear_detailed_gazebo.launch.py
    ros2 launch dropbear dropbear_detailed_display.launch.py
    ros2 launch dropbear dropbear_sdf.launch.py
    ```

3. Run the trajectory control nodes:
    ```bash
    ros2 run dropbear hand_trajectory_publisher.py
    ros2 run dropbear pelvic_girdle_joints_trajectory_publisher.py
    ros2 run dropbear leg_trajectory_publisher.py
    ```

## Video Demo:
    Check out the demo video showing the simulation in action!
