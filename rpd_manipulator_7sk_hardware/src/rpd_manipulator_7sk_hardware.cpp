#include "rpd_manipulator_7sk_hardware/rpd_manipulator_7sk_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>

namespace rpd_manipulator_7sk_hardware {
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn RPDManipulator7SKHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
        if(hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;
        info_ = params.hardware_info;
        joints_ = info_.joints;
        num_of_joints_ = joints_.size();

        port_ = info_.hardware_parameters["port"];
        baudrate_ = std::stoi(info_.hardware_parameters["baudrate"]);

        hw_positions_.resize(num_of_joints_, 0.0);
        hw_velocities_.resize(num_of_joints_, 0.0);
        cmd_positions_.resize(num_of_joints_, 0.0);
        motor_ids_.resize(num_of_joints_);
        motor_ids_uint8_.resize(num_of_joints_);

        for (size_t i = 0; i < num_of_joints_; i++) {
            motor_ids_[i] = std::stoi(joints_[i].parameters.at("motor_id"));
            motor_ids_uint8_[i] = static_cast<uint8_t>(motor_ids_[i]);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    std::vector<hardware_interface::StateInterface> RPDManipulator7SKHardwareInterface::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < num_of_joints_; i++) {
            auto joint_name = joints_[i].name;
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    std::vector<hardware_interface::CommandInterface> RPDManipulator7SKHardwareInterface::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < num_of_joints_; i++) {
            auto joint_name = joints_[i].name;
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &cmd_positions_[i]));
        }
        return command_interfaces;
    }
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn RPDManipulator7SKHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
        if (!driver_.begin(baudrate_, port_.c_str())) return hardware_interface::CallbackReturn::ERROR;
        usleep(100000);
        driver_.syncReadBegin(num_of_joints_, sizeof(rx_packet_));
        return hardware_interface::CallbackReturn::SUCCESS;
    }
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn RPDManipulator7SKHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
        for (size_t i = 0; i < num_of_joints_; i++) {
            driver_.EnableTorque(motor_ids_[i], 1);
            usleep(50000);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn RPDManipulator7SKHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
        for (size_t i = 0; i < num_of_joints_; i++) {
            driver_.EnableTorque(motor_ids_[i], 0);
            usleep(50000);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::CallbackReturn RPDManipulator7SKHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) {
        driver_.syncReadEnd();
        driver_.end();
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::return_type RPDManipulator7SKHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
        driver_.syncReadPacketTx(motor_ids_uint8_.data(), num_of_joints_, SMS_STS_PRESENT_POSITION_L, sizeof(rx_packet_));
        size_t i;
        for (i = 0; i < num_of_joints_ - 1; i++) {
            if (!driver_.syncReadPacketRx(motor_ids_uint8_[i], rx_packet_)) continue;
            int16_t raw_pos = driver_.syncReadRxPacketToWrod(15);
            int16_t raw_speed = driver_.syncReadRxPacketToWrod(15);
            hw_positions_[i] = (raw_pos / 4096.0 - 0.5) * 2.0 * M_PI;
            hw_velocities_[i] = raw_speed * 0.0146 * 2.0 * M_PI / 60.0;
        }
        if (driver_.syncReadPacketRx(motor_ids_uint8_[i], rx_packet_)) {
            int16_t raw_pos = driver_.syncReadRxPacketToWrod(15);
            int16_t raw_speed = driver_.syncReadRxPacketToWrod(15);
            hw_positions_[i] = (raw_pos / 4096.0 - 0.5) * (4096 / 50625);
            hw_velocities_[i] = raw_speed * 0.0146 * (4096 / 50625) / 60.0;
        };
        return hardware_interface::return_type::OK;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    hardware_interface::return_type RPDManipulator7SKHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        std::vector<s16> positions(num_of_joints_, 0);
        std::vector<u16> velocities(num_of_joints_, 400);
        std::vector<u8> accelerations(num_of_joints_, 50);
        size_t i;
        for (i = 0; i < num_of_joints_ - 1; i++) {
            int target_pos = static_cast<int>((cmd_positions_[i] / M_PI / 2.0 + 0.5) * 4096.0);
            positions[i] = static_cast<s16>(std::max(0, std::min(4096, target_pos)));
        }
        int target_pos = static_cast<int>((cmd_positions_[i] / 4096 * 50625 + 0.5) * 4096.0);
        positions[i] = static_cast<s16>(std::max(0, std::min(4096, target_pos)));
        driver_.SyncWritePosEx(motor_ids_uint8_.data(), num_of_joints_, positions.data(), velocities.data(), accelerations.data());
        return hardware_interface::return_type::OK;
    }

} // namespace rpd_manipulator_7sk_hardware
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rpd_manipulator_7sk_hardware::RPDManipulator7SKHardwareInterface,
    hardware_interface::SystemInterface
)