// [Existing header and license comments]

#include "xarm_hardware/xarm_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <mutex>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace xarm_hardware
{
    hardware_interface::CallbackReturn XArmSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        joints = {
            Joint("xarm_6_joint"),
            Joint("xarm_5_joint"),
            Joint("xarm_4_joint"),
            Joint("xarm_3_joint"),
            Joint("xarm_2_joint"),
        };
        // [Existing todo comments]

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // [Existing validation code]
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> XArmSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joints[i].name, hardware_interface::HW_IF_POSITION, &joints[i].pos));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints[i].vel));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> XArmSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                joints[i].name, hardware_interface::HW_IF_POSITION, &joints[i].cmd));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn XArmSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Activating ...please wait...");
        xarm_control_.connect();
        RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn XArmSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Deactivating ...please wait...");
        xarm_control_.disconnect();
        RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type XArmSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Access the latest joint positions from the async read thread
        std::vector<double> positions = xarm_control_.getLatestJointPositions();

        if (positions.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("XArmSystemHardware"), "No joint positions received");
            return hardware_interface::return_type::OK;
        }

        // Update joint states
        for (size_t i = 0; i < joints.size(); i++)
        {
            double pos_prev = joints[i].pos;
            joints[i].pos = positions[i];
            joints[i].vel = (joints[i].pos - pos_prev) / (period.seconds() + 1e-6);
        }

        // Use DEBUG level to reduce log verbosity
        RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"),
                     "Reading positions: %f, %f, %f, %f, %f",
                     positions[0], positions[1], positions[2], positions[3], positions[4]);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type XArmSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        std::vector<std::string> joint_names = {"xarm_6_joint", "xarm_5_joint", "xarm_4_joint", "xarm_3_joint", "xarm_2_joint"};
        std::vector<double> positions = {joints[0].cmd, joints[1].cmd, joints[2].cmd, joints[3].cmd, joints[4].cmd};

        // Apply PID control or any other control strategy here if needed

        for (auto i = 0u; i < joints.size(); i++)
        {
            xarm_control_.setJointPosition(joints[i].name, joints[i].cmd, 1000);
        }

        return hardware_interface::return_type::OK;
    }

} // namespace xarm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    xarm_hardware::XArmSystemHardware, hardware_interface::SystemInterface)