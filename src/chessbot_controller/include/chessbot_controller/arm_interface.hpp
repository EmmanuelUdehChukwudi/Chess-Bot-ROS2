#ifndef ARM_INTERFACE_HPP_
#define ARM_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "chessbot_controller/visibility_control.h"

#include <vector>
#include <string>


namespace arm_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotArmInterface : public hardware_interface::SystemInterface
{
public:
RCLCPP_SHARED_PTR_DEFINITIONS( RobotArmInterface)
  RobotArmInterface();
  virtual ~RobotArmInterface();


  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;
};
}


#endif  