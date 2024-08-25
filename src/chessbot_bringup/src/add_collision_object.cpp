#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_collision_object");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);

    auto spinner = std::thread([executor]() { executor->spin(); });

    moveit::planning_interface::PlanningSceneInterface current_scene;
    rclcpp::sleep_for(std::chrono::seconds(5));

    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.id = "test_cylinder";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2); 
    primitive.dimensions[0] = 0.1;  // height
    primitive.dimensions[1] = 0.005;  // radius

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = 0.4;
    pose.position.z = 0.3;

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;
    cylinder.header.frame_id = "base_link";

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    current_scene.addCollisionObjects(collision_objects);
    rclcpp::sleep_for(std::chrono::seconds(2));

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
