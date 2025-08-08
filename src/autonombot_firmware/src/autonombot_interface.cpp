#include "autonombot_firmware/autonombot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace autonombot_firmware
{
    AutonomBotInterface::AutonomBotInterface()
    {
    }

    AutonomBotInterface::~AutonomBotInterface()
    {
        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonomBotInterface"),
                                    "Something went wrong while closing connection with port " << port_);
            }
        }
    }

    CallbackReturn AutonomBotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("AutonomBotInterface"), "No Serial Port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            velocity_commands_.push_back(0.0);
            position_states_.push_back(0.0);
            velocity_states_.push_back(0.0);
        }

        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> AutonomBotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AutonomBotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn AutonomBotInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AutonomBotInterface"), "Starting robot hardware ...");

        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonomBotInterface"),
                                "Something went wrong while interacting with port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("AutonomBotInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn AutonomBotInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("AutonomBotInterface"), "Stopping robot hardware ...");

        if (arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("AutonomBotInterface"),
                                    "Something went wrong while closing connection with port " << port_);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("AutonomBotInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type AutonomBotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!arduino_.IsOpen())
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::string response;
            arduino_.ReadLine(response);

            std::string delimiter = ",";
            size_t del_pos = response.find(delimiter);
            std::string token_1 = response.substr(0, del_pos);
            std::string token_2 = response.substr(del_pos + delimiter.length());

            position_states_.at(0) = std::stod(token_1);
            position_states_.at(1) = std::stod(token_2);
        }
        catch (...)
        {
            auto logger = rclcpp::get_logger("AutonomBotInterface");
            auto clock = rclcpp::Clock();
            RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, 1000, "Something went wrong while reading from the port");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AutonomBotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!arduino_.IsOpen())
        {
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::string msg = std::to_string(velocity_commands_.at(0)) + "," + std::to_string(velocity_commands_.at(1)) + ",\r";
            arduino_.Write(msg);
        }
        catch (...)
        {
            auto logger = rclcpp::get_logger("AutonomBotInterface");
            auto clock = rclcpp::Clock();
            RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, 1000, "Something went wrong while writing to the port");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(autonombot_firmware::AutonomBotInterface, hardware_interface::SystemInterface)
