#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_group_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::sleep_for(std::chrono::seconds(2));
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/move_group/display_planned_path", 10);
  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  RCLCPP_INFO(node->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  std::vector<geometry_msgs::msg::Pose> target_poses;

  geometry_msgs::msg::Pose pose1;
  pose1.orientation.x = -0.632;
  pose1.orientation.y = -0.183;
  pose1.orientation.z = -0.336;
  pose1.orientation.w = 0.674;
  pose1.position.x = 0.075;
  pose1.position.y = 0.216;
  pose1.position.z = 0.294;

  geometry_msgs::msg::Pose pose2;
  pose2.orientation.x = -0.499;
  pose2.orientation.y = 0.502;
  pose2.orientation.z = 0.499;
  pose2.orientation.w = 0.501;
  pose2.position.x = 0.147;
  pose2.position.y = 0.11;
  pose2.position.z = 0.198;

  target_poses.push_back(pose1);
  target_poses.push_back(pose2);

  for (const auto &pose : target_poses)
  {
    move_group.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto success = move_group.plan(my_plan);

    if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
      move_group.move();
    } else {
      RCLCPP_ERROR(node->get_logger(), "Plan failed for the current pose");
    }

    move_group.clearPoseTargets();
  }

  rclcpp::shutdown();
  return 0;
}
