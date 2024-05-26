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

#include "hoverboard_hardware_interface/hoverboard_hardware_interface.hpp"

namespace hoverboard_hardware_interface
{
    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hardwareConfig.frontLeftWheelJointName = info.hardware_parameters.at("front_left_wheel_joint_name");
        hardwareConfig.frontRightWheelJointName = info.hardware_parameters.at("front_right_wheel_joint_name");
        hardwareConfig.backLeftWheelJointName = info.hardware_parameters.at("back_left_wheel_joint_name");
        hardwareConfig.backRightWheelJointName = info.hardware_parameters.at("back_right_wheel_joint_name");
        hardwareConfig.loopRate = std::stof(info.hardware_parameters.at("loop_rate"));
        hardwareConfig.encoderTicksPerRevolution = std::stoi(info.hardware_parameters.at("encoder_ticks_per_revolution"));

        serialPortConfig.frontDevice = info.hardware_parameters.at("front_device");
        serialPortConfig.backDevice = info.hardware_parameters.at("back_device");
        serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
        serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));

        frontLeftWheel = MotorWheel(hardwareConfig.frontLeftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
        frontRightWheel = MotorWheel(hardwareConfig.frontRightWheelJointName, hardwareConfig.encoderTicksPerRevolution);
        backLeftWheel = MotorWheel(hardwareConfig.backLeftWheelJointName, hardwareConfig.encoderTicksPerRevolution);
        backRightWheel = MotorWheel(hardwareConfig.backRightWheelJointName, hardwareConfig.encoderTicksPerRevolution);

        for (const hardware_interface::ComponentInfo & joint : info.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has %zu state interfaces. 2 expected.", joint.name.c_str(), joint.state_interfaces.size());
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("HoverboardHardwareInterface"),
                    "Joint '%s' has '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> HoverboardHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(frontLeftWheel.name, hardware_interface::HW_IF_POSITION, &frontLeftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontLeftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontLeftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(frontRightWheel.name, hardware_interface::HW_IF_POSITION, &frontRightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(frontRightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontRightWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(backLeftWheel.name, hardware_interface::HW_IF_POSITION, &backLeftWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backLeftWheel.name, hardware_interface::HW_IF_VELOCITY, &backLeftWheel.velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(backRightWheel.name, hardware_interface::HW_IF_POSITION, &backRightWheel.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(backRightWheel.name, hardware_interface::HW_IF_VELOCITY, &backRightWheel.velocity));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HoverboardHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontLeftWheel.name, hardware_interface::HW_IF_VELOCITY, &frontLeftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(frontRightWheel.name, hardware_interface::HW_IF_VELOCITY, &frontRightWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backLeftWheel.name, hardware_interface::HW_IF_VELOCITY, &backLeftWheel.command));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(backRightWheel.name, hardware_interface::HW_IF_VELOCITY, &backRightWheel.command));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Configuring... please wait a moment...");

        if (!connect())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        frontSerialPortService.BindMotorWheelFeedbackCallback(
            std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1)
        );
        backSerialPortService.BindMotorWheelFeedbackCallback(
            std::bind(&HoverboardHardwareInterface::motorWheelFeedbackCallback, this, std::placeholders::_1)
        );

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Cleaning up... please wait a moment...");

        if (!disconnect())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Activating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HoverboardHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // TODO: add some logic
        RCLCPP_INFO(rclcpp::get_logger("HoverboardHardwareInterface"), "Deactivating... please wait a moment...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HoverboardHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
    {
        frontSerialPortService.read();
        backSerialPortService.read();

        double lastPosition = frontLeftWheel.position;
        frontLeftWheel.position = frontLeftWheel.calculateEncoderAngle();
        frontLeftWheel.velocity = (frontLeftWheel.position - lastPosition) / period.seconds();

        lastPosition = frontRightWheel.position;
        frontRightWheel.position = frontRightWheel.calculateEncoderAngle();
        frontRightWheel.velocity = (frontRightWheel.position - lastPosition) / period.seconds();

        lastPosition = backLeftWheel.position;
        backLeftWheel.position = backLeftWheel.calculateEncoderAngle();
        backLeftWheel.velocity = (backLeftWheel.position - lastPosition) / period.seconds();

        lastPosition = backRightWheel.position;
        backRightWheel.position = backRightWheel.calculateEncoderAngle();
        backRightWheel.velocity = (backRightWheel.position - lastPosition) / period.seconds();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HoverboardHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        MotorWheelDriveControl frontMotorWheelDriveControl;
        MotorWheelDriveControl backMotorWheelDriveControl;

        const double frontSpeed = ((frontLeftWheel.command / 0.10472) + (frontRightWheel.command / 0.10472)) / 2.0;
        const double frontSteer = ((frontLeftWheel.command / 0.10472) - frontSpeed) * 2.0;

        const double backSpeed = ((backLeftWheel.command / 0.10472) + (backRightWheel.command / 0.10472)) / 2.0;
        const double backSteer = ((backLeftWheel.command / 0.10472) - backSpeed) * 2.0;

        // TODO: radius should be read from the urdf file, check calculations
        frontMotorWheelDriveControl.speed = static_cast<int16_t>(frontSpeed);
        frontMotorWheelDriveControl.steer = static_cast<int16_t>(frontSteer);
        frontMotorWheelDriveControl.checksum = static_cast<uint16_t>(frontMotorWheelDriveControl.head ^ frontMotorWheelDriveControl.steer ^ frontMotorWheelDriveControl.speed);

        backMotorWheelDriveControl.speed = static_cast<int16_t>(backSpeed);
        backMotorWheelDriveControl.steer = static_cast<int16_t>(backSteer);
        backMotorWheelDriveControl.checksum = static_cast<uint16_t>(backMotorWheelDriveControl.head ^ backMotorWheelDriveControl.steer ^ backMotorWheelDriveControl.speed);

        RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "Front: %i %i", frontMotorWheelDriveControl.speed, frontMotorWheelDriveControl.steer);
        RCLCPP_INFO(rclcpp::get_logger("SerialPortService"), "Back: %i %i", backMotorWheelDriveControl.speed, backMotorWheelDriveControl.steer);

        frontSerialPortService.write(reinterpret_cast<const char*>(&frontMotorWheelDriveControl), sizeof(MotorWheelDriveControl));
        backSerialPortService.write(reinterpret_cast<const char*>(&backMotorWheelDriveControl), sizeof(MotorWheelDriveControl));

        return hardware_interface::return_type::OK;
    }

    void HoverboardHardwareInterface::motorWheelFeedbackCallback(MotorWheelFeedback motorWheelFeedback)
    {
        frontLeftWheel.updateEncoderTicks(motorWheelFeedback.leftMotorEncoderCumulativeCount);
        frontRightWheel.updateEncoderTicks(motorWheelFeedback.rightMotorEncoderCumulativeCount);
        backLeftWheel.updateEncoderTicks(motorWheelFeedback.leftMotorEncoderCumulativeCount);
        backRightWheel.updateEncoderTicks(motorWheelFeedback.rightMotorEncoderCumulativeCount);
    }

    bool HoverboardHardwareInterface::connect()
    {
        bool frontConnected = frontSerialPortService.connect(serialPortConfig.frontDevice, serialPortConfig.baudRate, serialPortConfig.timeout);
        bool backConnected = backSerialPortService.connect(serialPortConfig.backDevice, serialPortConfig.baudRate, serialPortConfig.timeout);

        return frontConnected && backConnected;
    }

    bool HoverboardHardwareInterface::disconnect()
    {
        bool frontDisconnected = frontSerialPortService.disconnect();
        bool backDisconnected = backSerialPortService.disconnect();

        return frontDisconnected && backDisconnected;
    }
}

PLUGINLIB_EXPORT_CLASS(hoverboard_hardware_interface::HoverboardHardwareInterface, hardware_interface::SystemInterface)

