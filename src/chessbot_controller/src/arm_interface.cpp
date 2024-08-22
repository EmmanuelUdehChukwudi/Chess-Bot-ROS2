#include "chessbot_controller/arm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace arm_controller
{
RobotArmInterface::RobotArmInterface()
{
  
}


RobotArmInterface::~RobotArmInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotArmInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn RobotArmInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RobotArmInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RobotArmInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmInterface"),"Initializing the Robot Hardware.......");
        position_commands_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        prev_position_commands_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        position_states_ = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotArmInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RobotArmInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type RobotArmInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }

        std::string msg;
        int waist = static_cast<int>((position_commands_.at(0))* 180) / M_PI;
        msg.append("b");
        msg.append(std::to_string(waist));
        msg.append(",");

        int shoulder = static_cast<int>((position_commands_.at(1))* 180) / M_PI;
        msg.append("s");
        msg.append(std::to_string(shoulder));
        msg.append(",");

        int elbow = static_cast<int>((position_commands_.at(2))* 180) / M_PI;
        msg.append("e");
        msg.append(std::to_string(elbow));
        msg.append(",");

        int wrist = static_cast<int>((position_commands_.at(3))* 180) / M_PI;
        msg.append("w");
        msg.append(std::to_string(wrist));
        msg.append(",");

        int wrist_twist = static_cast<int>((position_commands_.at(4))* 180) / M_PI;
        msg.append("t");
        msg.append(std::to_string(wrist_twist));
        msg.append(",");

        int gripper_twist = static_cast<int>((position_commands_.at(5))* 180) / M_PI;
        msg.append("j");
        msg.append(std::to_string(wrist_twist));
        msg.append(",");

        int tool_tip = static_cast<int>((position_commands_.at(6))* 180) / M_PI;
        tool_tip = 35 - tool_tip;
        msg.append("g");
        msg.append(std::to_string(tool_tip));
        msg.append(",");

  try
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotArmInterface"), "Sending new command " << msg);
    arduino_.Write(msg);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RobotArmInterface"),
                        "Something went wrong while sending the message "
                            << msg << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  prev_position_commands_ = position_commands_;

  return hardware_interface::return_type::OK;
}
}  

PLUGINLIB_EXPORT_CLASS(arm_controller::RobotArmInterface, hardware_interface::SystemInterface)