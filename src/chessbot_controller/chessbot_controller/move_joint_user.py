#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointMover(Node):
    def __init__(self):
        super().__init__('joint_mover')
        self.arm_publisher_ = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_publisher_ = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.arm_target_positions, self.gripper_target_position = self.get_joint_angles_from_user()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Joint Mover Node started and publishing joint positions.')

    def get_joint_angles_from_user(self):
        angles_degrees = input("Enter joint angles in degrees (7 angles, comma-separated): ")
        angles_degrees = angles_degrees.split(',')
        angles_degrees = [float(angle) for angle in angles_degrees]
        if len(angles_degrees) != 7:
            self.get_logger().error("Error: You must enter exactly 7 angles.")
            raise ValueError("Exactly 7 angles are required.")
        
        angles_radians = [math.radians(angle) for angle in angles_degrees]

        arm_angles_radians = angles_radians[:6] 
        gripper_angle_radian = angles_radians[6]
        
        return arm_angles_radians, gripper_angle_radian

    def timer_callback(self):
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        arm_point = JointTrajectoryPoint()
        arm_point.positions = self.arm_target_positions
        arm_point.time_from_start.sec = 5
        
        arm_msg.points.append(arm_point)
        self.arm_publisher_.publish(arm_msg)
        self.get_logger().info(f'Publishing Arm Joint Positions: {self.arm_target_positions}')
        
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ["joint7"]
        
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [self.gripper_target_position]
        gripper_point.time_from_start.sec = 2 
        
        gripper_msg.points.append(gripper_point)
        self.gripper_publisher_.publish(gripper_msg)
        self.get_logger().info(f'Publishing Gripper Joint Position: {self.gripper_target_position}')

def main(args=None):
    rclpy.init(args=args)
    joint_mover = JointMover()
    rclpy.spin(joint_mover)
    
    joint_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
