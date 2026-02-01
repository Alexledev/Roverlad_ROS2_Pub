#ifndef ROVERLAD_INTERFACE_HPP
#define ROVERLAD_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <roverlad_interfaces/msg/controller_data.hpp>
#include <roverlad_interfaces/msg/rover_data.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <iomanip>
#include <vector>
#include <string>


namespace roverlad_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RoverladInterface : public hardware_interface::SystemInterface
{
public:
  RoverladInterface();
  virtual ~RoverladInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  void pushCarData(const roverlad_interfaces::msg::RoverData::SharedPtr msg);
  void resetRover(const std_msgs::msg::String::SharedPtr msg);

  double meas_l_spd;
  double meas_r_spd;


private:
    std::string port_;
    std::vector<double> velocity_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    rclcpp::Time last_run_;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vel_send;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_pos;
    rclcpp::Subscription<roverlad_interfaces::msg::RoverData>::SharedPtr car_data_receive;

};
}  


#endif  // Roverlad_INTERFACE_HPP