#include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <roverlad_interfaces/msg/
#include <roverlad_interfaces/msg/controller_data.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sstream>

class ControllerReader : public rclcpp::Node
{
public:
    ControllerReader() : Node("controller_reader_node")
    {
        sub_ = this->create_subscription<roverlad_interfaces::msg::ControllerData>(
            "controller_data", 10,
            std::bind(&ControllerReader::callback, this, std::placeholders::_1));

        
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/roverlad_controller/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<roverlad_interfaces::msg::ControllerData>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;

    void callback(const roverlad_interfaces::msg::ControllerData::SharedPtr msg)
    {
        

        geometry_msgs::msg::TwistStamped t;
        t.header.stamp = this->get_clock()->now();

        t.twist.linear.x = static_cast<double>(msg->rx);
        t.twist.angular.z = static_cast<double>(msg->ly);

        pub_->publish(t);

        RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->rx << " " << msg->ry 
                                                     << " " << msg->lx << " " << msg->ly);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerReader>());
    rclcpp::shutdown();
    return 0;
}
