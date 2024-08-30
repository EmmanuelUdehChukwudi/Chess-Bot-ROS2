#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SliderControl(Node):

    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        arm_controller.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        gripper_controller.joint_names = ["joint7"]
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        arm_goal.positions = list(msg.position[:6])  
        gripper_goal.positions = [msg.position[6]]
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

        # Combine all joint positions into a single list
        all_joints = arm_goal.positions + gripper_goal.positions
        self.get_logger().info(f'Joint Positions: {all_joints}')


def main():
    rclpy.init()

    slider_control = SliderControl()
    rclpy.spin(slider_control)
    
    slider_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
