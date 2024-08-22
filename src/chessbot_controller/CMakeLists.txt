cmake_minimum_required(VERSION 3.8)
project(chessbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

add_executable(slider_control src/slider.cpp)
ament_target_dependencies(slider_control rclcpp trajectory_msgs sensor_msgs)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS
  slider_control
  DESTINATION lib/${PROJECT_NAME}


)
add_library(
  ${PROJECT_NAME}
  SHARED
  src/arm_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(PROGRAMS
  ${PROJECT_NAME}/slider.py
  DESTINATION lib/${PROJECT_NAME}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface arm_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${SERIAL_LDFLAGS}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()