#ifndef MOTORINFO_H
#define MOTORINFO_H

#include <rclcpp/rclcpp.hpp>
#include "SerialReader.hpp"
#include <string>

class MotorInfo : public SerialReader
{
private:
    float l_speed = 0.0;
    float r_speed = 0.0;

public:   

    std::vector<float> getData(uint8_t* buf) override
    {
        std::vector<float> data = read(buf);

        if (data.size() > 1)
        {
            l_speed = data[0];
            r_speed = data[1];
        }

        return {l_speed, r_speed};
    }

};

#endif