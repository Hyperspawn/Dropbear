#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class LegJTCPublisher(Node):

    def __init__(self):
        super().__init__('leg_jtc_publisher_node')
        # Get user input for leg and joints
        self.leg:str
        self.joints:str = []

        # Publish topic name
        self.publish_topic:str

        self.get_logger().info("Enter 'right' or 'left' for the leg to control:")
        self.leg = input().lower()
        if self.leg == 'right':
            self.get_logger().info("Leg: Right Leg")
            self.get_logger().info("Controlled Joints: RL_hip_joint, RL_knee_actuator_joint, RL_Revolute67, RL_Revolute81, RL_Revolute88")
            self.joints = ["RL_hip_joint", "RL_knee_actuator_joint", "RL_Revolute67", "RL_Revolute81", "RL_Revolute88"]
            # Topic to publish to
            self.publish_topic = "/right_leg_controller/joint_trajectory"
        elif self.leg == 'left':
            self.get_logger().info("Leg: Left Leg")
            self.get_logger().info("Controlled Joints: LL_hip_joint, LL_knee_actuator_joint, LL_Revolute67, LL_Revolute81, LL_Revolute88")
            self.joints = ["LL_hip_joint", "LL_knee_actuator_joint", "LL_Revolute67", "LL_Revolute81", "LL_Revolute88"]
            # Topic to publish to
            self.publish_topic = "/left_leg_controller/joint_trajectory"
        else:
            self.get_logger().error("Invalid leg selection. Exiting...")
            sys.exit(1)

        # Creating the trajectory publisher
        self.trajectory_publisher = self.create_publisher(JointTrajectory, self.publish_topic, 10)
        timer_period = 1.0
        # Creating a timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

        # Prompting user for input for each joint
        self.goal_positions = []
        for joint in self.joints:
            try:
                position = float(input(f"Enter the target position for {joint}: "))
                self.goal_positions.append(position)
            except ValueError:
                self.get_logger().warn(f"Invalid input for {joint}. Defaulting to 0.0")
                self.goal_positions.append(0.0)
    
    # Timer callback function
    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info("Trajectory Sent!")

def main(args=None):
    rclpy.init(args=args)
    node = LegJTCPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
