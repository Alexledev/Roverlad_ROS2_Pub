#include "rclcpp/rclcpp.hpp"
#include "roverlad_firmware/Kalman.hpp"

// Definition of the static member
uint64_t Kalman1D::last_time_ms = 0;
