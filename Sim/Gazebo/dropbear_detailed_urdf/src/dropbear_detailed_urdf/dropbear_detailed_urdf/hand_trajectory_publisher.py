#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HandJTCPublisher(Node):

    def __init__(self):
        super().__init__('hand_jtc_publisher_node')
        # Get user input for hand and joints
        self.leg:str
        self.joints:str = []

        # Publish topic name
        self.publish_topic:str

        self.get_logger().info("Enter 'right' or 'left' for the hand to control:")
        self.leg = input().lower()
        if self.leg == 'right':
            self.get_logger().info("Hand: Right Hand")
            self.get_logger().info("Controlled Joints: RH_yaw, RH_pitch, RH_roll, RH_elbow_joint, RH_wrist_roll")
            self.joints = ["RH_yaw", "RH_pitch", "RH_roll", "RH_elbow_joint", "RH_wrist_roll"]
            # Topic to publish to
            self.publish_topic = "/right_hand_controller/joint_trajectory"
        elif self.leg == 'left':
            self.get_logger().info("Hand: Left Hand")
            self.get_logger().info("Controlled Joints: LH_yaw, LH_pitch, LH_roll, LH_elbow_joint, LH_wrist_roll")
            self.joints = ["LH_yaw", "LH_pitch", "LH_roll", "LH_elbow_joint", "LH_wrist_roll"]
            # Topic to publish to
            self.publish_topic = "/left_hand_controller/joint_trajectory"
        else:
            self.get_logger().error("Invalid hand selection. Exiting...")
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
    node = HandJTCPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
