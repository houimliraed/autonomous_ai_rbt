#ifndef AUTONOMBOT_FIRMWARE__AUTONOMBOT_INTERFACE_HPP_
#define AUTONOMBOT_FIRMWARE__AUTONOMBOT_INTERFACE_HPP_

#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace autonombot_firmware
{
    class AutonomBotInterface : public hardware_interface::SystemInterface
    {
    public:
        AutonomBotInterface();
        virtual ~AutonomBotInterface();

        // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Implementing hardware_interface::SystemInterface
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        LibSerial::SerialPort arduino_;
        std::string port_;
        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
        rclcpp::Time last_run_;
    };
}

#endif // AUTONOMBOT_FIRMWARE__AUTONOMBOT_INTERFACE_HPP_
