#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include "roverlad_interfaces/msg/rover_data.hpp"
#include "roverlad_firmware/MPU6050.hpp"
#include "roverlad_firmware/MotorInfo.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class RoverComms : public rclcpp::Node
{
public:
  RoverComms() : Node("rover_comms")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    get_parameter("port", port);

    serial_port.Open(port);
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    subscription = this->create_subscription<std_msgs::msg::String>(
      "motor_control", 10, std::bind(&RoverComms::topic_callback, this, _1));

    timer = this->create_wall_timer(10ms, std::bind(&RoverComms::timer_callback, this)); 
    pub = this->create_publisher<roverlad_interfaces::msg::RoverData>("received_car_data", 10);

    mpuInfo = std::make_shared<MPU6050>();
    motorInfo = std::make_shared<MotorInfo>();
  }

  ~RoverComms()
  {
    serial_port.Close();
  }


private:
  void write(const std::vector<float>& vec)
  {  
      LibSerial::DataBuffer buffer;
      buffer.reserve(vec.size() * sizeof(float));

      for (float f : vec) {
          uint8_t bytes[sizeof(float)];
          std::memcpy(bytes, &f, sizeof(float));
          buffer.insert(buffer.end(), bytes, bytes + sizeof(float));
      }

      serial_port.Write(buffer);
  }

  std::vector<float> parseFloatVector(const std::string& s)
  {
      std::vector<float> vec;
      std::stringstream ss(s);
      std::string item;
      while (std::getline(ss, item, ',')) {
          try {
              vec.push_back(std::stof(item));
          } catch (...) {
              // ignore invalid numbers
          }
      }
      return vec;
  }


  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {    
    std::vector<float> vec = parseFloatVector(msg->data);
    write(vec);    
  }


  void timer_callback()
  {

    std::vector<float> motorVec;
    std::vector<float> mpuVec;


    uint8_t buffer[SerialReader::PACKET_SIZE];
    if (SerialReader::isDataReady(serial_port, buffer))
    {
      motorVec = motorInfo->getData(buffer);
      mpuVec = mpuInfo->getData(buffer);
    }

    if (motorVec.size() == 2 && mpuVec.size() > 2)
    {
      auto message = roverlad_interfaces::msg::RoverData();
      message.spd_r = motorVec[0];
      message.spd_l = motorVec[1];

      message.lin_accel_x = mpuVec[0];
      message.lin_accel_y = mpuVec[1];
      message.lin_accel_z = mpuVec[2];

      message.ang_vel_x = mpuVec[3];
      message.ang_vel_y = mpuVec[4];
      message.ang_vel_z = mpuVec[5]; 

      pub->publish(message);
    }
    

  }

  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;

  rclcpp::Publisher<roverlad_interfaces::msg::RoverData>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  LibSerial::SerialPort serial_port;

  std::shared_ptr<MPU6050> mpuInfo;
  std::shared_ptr<MotorInfo> motorInfo;

  std::string port;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverComms>());
    rclcpp::shutdown();
    return 0;
}