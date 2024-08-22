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
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("chessbot_controller/joint_trajectory", 10);
    RCLCPP_INFO(this->get_logger(), "Slider Control Node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;

  void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto arm_command = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
    arm_command->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6","joint7"};

    auto arm_goal = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
    arm_goal->positions = msg->position;

    arm_command->points.push_back(*arm_goal);

    arm_pub_->publish(*arm_command);
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