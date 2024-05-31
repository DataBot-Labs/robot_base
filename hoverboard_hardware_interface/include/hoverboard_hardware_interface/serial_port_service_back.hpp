#pragma once

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "rclcpp/rclcpp.hpp"
#include "front_serial_port_protocol.hpp"


#define SERIAL_PORT_READ_BUF_SIZE 256

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace hoverboard_hardware_interface
{
    class SerialPortServiceBack
    {
    public:
        SerialPortServiceBack() = default;

        bool connect(const std::string &serial_device_back, int baud_rate, int timeout);
        bool disconnect();

        void read();
        void asyncRead();

        int write(const char *, const int &);

        void BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)>);

    private:
        boost::asio::io_service io_service;
        serial_port_ptr port1; // Declaration of 'port1' as a member variable

        boost::mutex mutex;

        uint16_t head_frame = 0;
        uint16_t msg_counter = 0;
        uint8_t msg_command = 0;

        char prev_byte = 0;
        char* p{};

        char read_buf_raw[SERIAL_PORT_READ_BUF_SIZE]{};

        void onReceive(const boost::system::error_code&, size_t);

        std::function<void(MotorWheelFeedback)> motorWheelFeedbackCallback;

        MotorWheelFeedback motorWheelFeedback {};
    };
} // namespace hoverboard_hardware_interface

