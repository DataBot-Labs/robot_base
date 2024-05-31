// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hoverboard_hardware_interface/serial_port_service_back.hpp"

using namespace hoverboard_hardware_interface;

bool SerialPortServiceBack::connect(const std::string &serial_device_back, int baud_rate, int timeout)
{
    boost::system::error_code ec;

    if (port1) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortServiceBack"), "Port is already opened...");
        return false;
    }

    port1 = serial_port_ptr(new boost::asio::serial_port(io_service));
    port1->open(serial_device_back, ec);

    if (ec) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortServiceBack"), "Connection to the %s failed..., error: %s",
            serial_device_back, ec.message().c_str());
        return false;
    }

    port1->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    port1->set_option(boost::asio::serial_port_base::character_size(8));
    port1->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    port1->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port1->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // TODO: try to run it asynchronously
    // boost::thread t([ObjectPtr = &io_service] { return ObjectPtr->run(); });
    // t.detach();

    return true;
}

bool SerialPortServiceBack::disconnect()
{
    boost::mutex::scoped_lock look(mutex);

    if (port1) {
        port1->cancel();
        port1->close();
        port1.reset();
    }

    io_service.stop();
    io_service.reset();

    return true;
}

void SerialPortServiceBack::read()
{
    boost::mutex::scoped_lock look(mutex);

    size_t bytes_transferred = port1->read_some(boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE));
    
    for (unsigned int i = 0; i < bytes_transferred; ++i) {

        head_frame = ((uint16_t)(read_buf_raw[i]) << 8) | (uint8_t) prev_byte;

        if (head_frame == HEAD_FRAME)
        {
            p = (char*) &motorWheelFeedback;
            *p++ = prev_byte;
            *p++ = read_buf_raw[i];
            msg_counter = 2;
        }
        else if (msg_counter >= 2 && msg_counter < sizeof(MotorWheelFeedback)) {
            *p++ = read_buf_raw[i];
            msg_counter++;
        }

        if (msg_counter == sizeof(MotorWheelFeedback))
        {
            motorWheelFeedbackCallback(motorWheelFeedback);

            msg_counter = 0;
        }

        prev_byte = read_buf_raw[i];
    }
}

void SerialPortServiceBack::asyncRead()
{
    if (port1.get() == nullptr || !port1->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortServiceBack"), "Port is already closed...");
        return;
    }

    port1->async_read_some(
            boost::asio::buffer(read_buf_raw, SERIAL_PORT_READ_BUF_SIZE),
            boost::bind(&SerialPortServiceBack::onReceive,
                    this, boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

void SerialPortServiceBack::onReceive(const boost::system::error_code& ec, size_t bytes_transferred)
{
    RCLCPP_INFO(rclcpp::get_logger("SerialPortServiceBack"), "onReceive async event...");

    // boost::mutex::scoped_lock look(mutex);

    // if (port.get() == nullptr || !port->is_open()) return;

    // if (ec) {
    //     asyncRead();
    //     return;
    // }
}

int SerialPortServiceBack::write(const char * message, const int & size)
{
    boost::system::error_code ec;

    if (port1.get() == nullptr || !port1->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPortServiceBack"), "Port is already closed...");
        return 0;
    }

    if (size == 0) {
        return 0;
    }

    return port1->write_some(boost::asio::buffer(message, size), ec);
}

void SerialPortServiceBack::BindMotorWheelFeedbackCallback(std::function<void(MotorWheelFeedback)> fn) {
    motorWheelFeedbackCallback = fn;
}