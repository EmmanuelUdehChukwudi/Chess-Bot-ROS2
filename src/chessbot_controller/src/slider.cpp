#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class SliderControl : public rclcpp::Node
{
public:
  SliderControl() : Node("slider_control")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", 10, std::bind(&SliderControl::sliderCallback, this, std::placeholders::_1));
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_controller/joint_trajectory", 10);
    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("gripper_controller/joint_trajectory", 10);
    RCLCPP_INFO(this->get_logger(), "Slider Control Node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;

  void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto arm_command = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    arm_command->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    auto arm_goal = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
    arm_goal->positions.assign(msg->position.begin(), msg->position.begin() + 6);

    arm_command->points.push_back(*arm_goal);
    arm_pub_->publish(*arm_command);

    if (msg->position.size() > 6) {
      auto gripper_command = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
      gripper_command->joint_names = {"joint7"};

      auto gripper_goal = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
      gripper_goal->positions.push_back(msg->position[6]); 

      gripper_command->points.push_back(*gripper_goal);
      gripper_pub_->publish(*gripper_command);
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
