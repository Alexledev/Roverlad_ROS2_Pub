#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <roverlad_interfaces/msg/controller_data.hpp>

class ControllerTeleop : public rclcpp::Node
{

public:
  ControllerTeleop() : Node("rover_comms")
  {
    cmd_receive = this->create_subscription<roverlad_interfaces::msg::ControllerData>("controller_data", 10, std::bind(&ControllerTeleop::pushCommand, this, std::placeholders::_1));
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
  }

  void pushCommand(const roverlad_interfaces::msg::ControllerData::SharedPtr msg)
  {
      // target_x_cmd = static_cast<double>(msg->lx) * 0.1;
      // target_y_cmd = static_cast<double>(msg->ry) * 0.1;

      geometry_msgs::msg::Twist twist;

      // Scale as needed
      twist.linear.x  = static_cast<double>(msg->rx) * 0.2;   // forward/back
      twist.angular.z = static_cast<double>(msg->ly) * 0.5;   // turn

      cmd_pub->publish(twist);
  }

private:
  // float target_x_cmd;
  // float target_y_cmd;

  rclcpp::Subscription<roverlad_interfaces::msg::ControllerData>::SharedPtr cmd_receive;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerTeleop>());
  rclcpp::shutdown();
  return 0;
}