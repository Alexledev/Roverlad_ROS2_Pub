#ifndef SERIALREADER_H
#define SERIALREADER_H

#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class SerialReader
{
public:
    static constexpr int PACKET_SIZE = 37; // 9 elements * 4 bytes (double) + 1 byte (\n)

    static bool isDataReady(LibSerial::SerialPort& serial_port, uint8_t* buf)
    {
        if (rclcpp::ok() && serial_port.IsDataAvailable())
        {           
            for (int i = 0; i < PACKET_SIZE; i++) {
                char b ;
                serial_port.ReadByte(b);   // blocking read of 1 byte
                buf[i] = static_cast<uint8_t>(b);
            }          
            return true;
        }

        return false;    
    }

    std::vector<float> read(uint8_t* buf)
    {
        std::vector<float> data;

        if (buf[PACKET_SIZE-1] != '\n') { // invalid packet, discard
            return data;
        }

        for (int i = 0; i < PACKET_SIZE - 5; i += 4)
        {
            float val;
            memcpy(&val, &buf[i], 4);
            data.push_back(val);
        }

        return data;        
    }

    virtual std::vector<float> getData(uint8_t* buf) = 0;
};

#endif