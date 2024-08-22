set(_AMENT_PACKAGE_NAME "moveit")
set(moveit_VERSION "0.3.0")
set(moveit_MAINTAINER "Udeh Emmanuel <emmanuelprime2@gmail.com>")
set(moveit_BUILD_DEPENDS )
set(moveit_BUILDTOOL_DEPENDS "ament_cmake")
set(moveit_BUILD_EXPORT_DEPENDS )
set(moveit_BUILDTOOL_EXPORT_DEPENDS )
set(moveit_EXEC_DEPENDS "moveit_ros_move_group" "moveit_kinematics" "moveit_planners" "moveit_simple_controller_manager" "joint_state_publisher" "joint_state_publisher_gui" "tf2_ros" "xacro" "chessbot_description" "controller_manager" "moveit_configs_utils" "moveit_ros_move_group" "moveit_ros_visualization" "moveit_ros_warehouse" "moveit_setup_assistant" "robot_state_publisher" "rviz2" "rviz_common" "rviz_default_plugins" "tf2_ros" "warehouse_ros_mongo" "xacro")
set(moveit_TEST_DEPENDS )
set(moveit_GROUP_DEPENDS )
set(moveit_MEMBER_OF_GROUPS )
set(moveit_DEPRECATED "")
set(moveit_EXPORT_TAGS)
list(APPEND moveit_EXPORT_TAGS "<build_type>ament_cmake</build_type>")