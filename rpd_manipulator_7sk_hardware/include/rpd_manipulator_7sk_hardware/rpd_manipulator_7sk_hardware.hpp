#ifndef rpd_manipulator_7sk_HARDWARE_INTERFACE_HPP
#define rpd_manipulator_7sk_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "st_sc_servo_control_lib/SCServo.h"


namespace rpd_manipulator_7sk_hardware {

class RPDManipulator7SKHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    hardware_interface::return_type
        read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type
        write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    SMS_STS driver_;
    std::string port_;
    int baudrate_;
    size_t num_of_joints_;
    std::vector<hardware_interface::ComponentInfo> joints_;

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> cmd_positions_;

    std::vector<int> motor_ids_;
    std::vector<uint8_t> motor_ids_uint8_;
    uint8_t rx_packet_[4];


}; // class RPDManipulator7SKHardwareInterface

} // namespace rpd_manipulator_7sk_hardware

#endif // rpd_manipulator_7sk_HARDWARE_INTERFACE_HPP