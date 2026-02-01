#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <rclcpp/rclcpp.hpp>
#include "SerialReader.hpp"
#include "Kalman.hpp"
#include <cstring> // memcpy

#define PI 3.14159265358979323846


uint64_t millis() {
    return rclcpp::Clock(RCL_STEADY_TIME).now().nanoseconds() / 1000000ULL;
}

class MPU6050 : public SerialReader
{
private:
    float meas_lin_x; 
    float meas_lin_y;
    float meas_lin_z;

    float meas_ang_x; 
    float meas_ang_y;
    float meas_ang_z;

    float linear_acceleration_x = 0.0;
    float linear_acceleration_y = 0.0;
    float linear_acceleration_z = 0.0;

    float angular_velocity_x = 0.0;
    float angular_velocity_y = 0.0;
    float angular_velocity_z = 0.0;

    float tm = 0.0;

    // Kalman filters (one per axis)
    Kalman1D k_lin_x, k_lin_y, k_lin_z;
    Kalman1D k_ang_x, k_ang_y, k_ang_z;

    const float lin_div = 1670.13f;
    const float ang_div = 7509.55f;

public:

    static constexpr int PACKET_SIZE = 37; // 9 elements * 4 bytes (double) + 1 byte (\n)

    MPU6050()
    {
        //Set up kalman filter

        float accel_Q = 1e-4f;
        float accel_R = 5e-2f;    // tune depending on accel noise
        float gyro_Q  = 1e-5f;
        float gyro_R  = 1e-3f;    // tune depending on gyro noise

        k_lin_x.setProcessNoise(accel_Q); k_lin_x.setMeasurementNoise(accel_R);
        k_lin_y.setProcessNoise(accel_Q); k_lin_y.setMeasurementNoise(accel_R);
        k_lin_z.setProcessNoise(accel_Q); k_lin_z.setMeasurementNoise(accel_R);

        k_ang_x.setProcessNoise(gyro_Q); k_ang_x.setMeasurementNoise(gyro_R);
        k_ang_y.setProcessNoise(gyro_Q); k_ang_y.setMeasurementNoise(gyro_R);
        k_ang_z.setProcessNoise(gyro_Q); k_ang_z.setMeasurementNoise(gyro_R);

    }

    std::vector<float> getData(uint8_t* buf) override    
    {

        std::vector<float> data = read(buf);

        if (data.size() < 2)
        {
            return data;                    
        }

        meas_lin_x = data[2] / lin_div; // g -> m/s^2 approximation
        meas_lin_y = data[3] / lin_div;
        meas_lin_z = data[4] / lin_div;

        meas_ang_x = data[5] / ang_div; // deg/s -> rad/s appro
        meas_ang_y = data[6] / ang_div;
        meas_ang_z = data[7] / ang_div;

        tm = data[8];

        float dt_s = Kalman1D::getDeltaT(millis());

        // Update kalman filters (returns filtered estimate)
        linear_acceleration_x = k_lin_x.update(meas_lin_x, dt_s);
        linear_acceleration_y = k_lin_y.update(meas_lin_y, dt_s);
        linear_acceleration_z = k_lin_z.update(meas_lin_z, dt_s);

        angular_velocity_x = k_ang_x.update(meas_ang_x, dt_s);
        angular_velocity_y = k_ang_y.update(meas_ang_y, dt_s);
        angular_velocity_z = k_ang_z.update(meas_ang_z, dt_s);

        return {linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
                angular_velocity_x, angular_velocity_y, angular_velocity_z};
    }

    std::string dataToString()
    {
        // std::string msg = "Angle X: " + std::to_string(angleX) + ", Angle Y: " + std::to_string(angleY) + ", Angle Z: " + std::to_string(angleZ) + "\n";
        std::string msg = "Linear X: " + std::to_string(linear_acceleration_x) + ", Linear Y: " + std::to_string(linear_acceleration_y) + ", Linear Z: " + std::to_string(linear_acceleration_z) 
                           + "Angular X: " + std::to_string(angular_velocity_x) + ", Angular Y: " + std::to_string(angular_velocity_y) + ", Angular Z: " + std::to_string(angular_velocity_z) + 
                           + "\n";

       
        return msg;
    }

    // Optional: allow tuning at runtime
    void setAccelNoise(float processNoise, float measNoise) {
        k_lin_x.setProcessNoise(processNoise); k_lin_x.setMeasurementNoise(measNoise);
        k_lin_y.setProcessNoise(processNoise); k_lin_y.setMeasurementNoise(measNoise);
        k_lin_z.setProcessNoise(processNoise); k_lin_z.setMeasurementNoise(measNoise);
    }

    void setGyroNoise(float processNoise, float measNoise) {
        k_ang_x.setProcessNoise(processNoise); k_ang_x.setMeasurementNoise(measNoise);
        k_ang_y.setProcessNoise(processNoise); k_ang_y.setMeasurementNoise(measNoise);
        k_ang_z.setProcessNoise(processNoise); k_ang_z.setMeasurementNoise(measNoise);
    }
};


#endif