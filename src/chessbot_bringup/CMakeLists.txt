cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0072 NEW)  # Prefer GLVND when available
project(chessbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(goto_goal src/goto_goal.cpp)
add_executable(add_collision src/add_collision_object.cpp)
add_executable(pick_and_place src/pick_and_place.cpp)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(goto_goal
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  interactive_markers
  moveit_core
  moveit_ros_perception
)

ament_target_dependencies(add_collision
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  interactive_markers
  moveit_core
  moveit_ros_perception
)

ament_target_dependencies(pick_and_place
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  interactive_markers
  moveit_core
  moveit_ros_perception
)

# Linking OpenGL and GLUT libraries separately for each executable
target_link_libraries(goto_goal
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

target_link_libraries(add_collision
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

target_link_libraries(pick_and_place
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

install(TARGETS goto_goal add_collision pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
