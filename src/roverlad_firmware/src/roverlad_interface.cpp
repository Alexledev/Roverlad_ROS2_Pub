#include "roverlad_firmware/roverlad_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace roverlad_firmware
{
    RoverladInterface::RoverladInterface()
    {
        node = rclcpp::Node::make_shared("rover_node");
    }

    RoverladInterface::~RoverladInterface()
    {

    }

    CallbackReturn RoverladInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        vel_send = node->create_publisher<std_msgs::msg::String>("motor_control", 10); 
        // cmd_receive = node->create_subscription<roverlad_interfaces::msg::ControllerData>("controller_data", 10, std::bind(&RoverladInterface::pushCommand, this, std::placeholders::_1));
        car_data_receive = node->create_subscription<roverlad_interfaces::msg::RoverData>("received_car_data", 10, std::bind(&RoverladInterface::pushCarData, this, std::placeholders::_1));
        reset_pos =  node->create_subscription<std_msgs::msg::String>("reset_roverlad", 10, std::bind(&RoverladInterface::resetRover, this, std::placeholders::_1));

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> RoverladInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Provide only a position Interafce
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> RoverladInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // Provide only a velocity Interafce
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }


    CallbackReturn RoverladInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("RoverladInterface"), "Starting robot hardware ...");

        // Reset commands and states
        velocity_commands_ = { 0.0, 0.0 };
        position_states_ = { 0.0, 0.0 };
        velocity_states_ = { 0.0, 0.0 };

        RCLCPP_INFO(rclcpp::get_logger("RoverladInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn RoverladInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {

        RCLCPP_INFO(rclcpp::get_logger("RoverladInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }


    hardware_interface::return_type RoverladInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        auto dt = (rclcpp::Clock().now() - last_run_).seconds();

        velocity_states_[0] = meas_r_spd;
        velocity_states_[1] = meas_l_spd;

        position_states_[0] += velocity_states_.at(0) * dt;
        position_states_[1] += velocity_states_.at(1) * dt;

        rclcpp::spin_some(node);

        last_run_ = rclcpp::Clock().now();
        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type RoverladInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        try
        {
            // Create string in format "-20.0,20.0"
            std_msgs::msg::String send;
            
            std::ostringstream ss;
            // ss << std::fixed << std::setprecision(1) << target_r_cmd << "," << target_l_cmd;
            ss << std::fixed << std::setprecision(1) << velocity_commands_[0] << "," << velocity_commands_[1];

            send.data = ss.str();

            vel_send->publish(send);
        }
        catch (...)
        {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    void RoverladInterface::pushCarData(const roverlad_interfaces::msg::RoverData::SharedPtr msg)
    {
        meas_r_spd = msg->spd_r;
        meas_l_spd = msg->spd_l;
    }

    void RoverladInterface::resetRover(const std_msgs::msg::String::SharedPtr msg)
    {
        velocity_commands_ = { 0.0, 0.0 };
        position_states_ = { 0.0, 0.0 };
        velocity_states_ = { 0.0, 0.0 };
    }
}  // namespace Roverlad_firmware

PLUGINLIB_EXPORT_CLASS(roverlad_firmware::RoverladInterface, hardware_interface::SystemInterface)