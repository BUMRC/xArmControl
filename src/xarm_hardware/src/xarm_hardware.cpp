// Copyright 2021 ros2_control Development Team
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

#include "xarm_hardware/xarm_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

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
        // todo: setup config parameters
        // cfg_.joint_names = something; // resize this
        // cfg_.joints = something; // need to probably resize this

        // cfg_.loop_rate = 0.0;
        // cfg_.device = "";
        // cfg_.baud_rate = 0; // check if need for hid communication
        // cfg_.timeout_ms = 0;
        // cfg_.enc_counts_per_rev = 0; // check actual servos and what they output

        // todo: create a class for each joint which will store attributes of each joint, and then create a vector of these joints

        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        // hw_start_sec_ =
        //     hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
        // hw_stop_sec_ =
        //     hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]); // stod  -> string to double, use appropriate function for datatype when using config
        // // END: This part here is for exemplary purposes - Please do not copy to your production code
        // hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("XArmSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("XArmSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("XArmSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("XArmSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("XArmSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
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
        std::vector<std::string> joint_names = {"xarm_6_joint", "xarm_5_joint", "xarm_4_joint", "xarm_3_joint", "xarm_2_joint"};
        std::vector<double> positions = xarm_control_.readJointsPositions(joint_names);
        for (auto i = 0u; i < joints.size(); i++)
        {
            double pos_prev = joints[i].pos;
            joints[i].pos = positions[i];
            joints[i].vel = (joints[i].pos - pos_prev) / (period.seconds() + 1e-6);
        }

        RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Reading positions: %f, %f, %f, %f, %f", positions[0], positions[1], positions[2], positions[3], positions[4]);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type xarm_hardware ::XArmSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        std::vector<std::string> joint_names = {"xarm_6_joint", "xarm_5_joint", "xarm_4_joint", "xarm_3_joint", "xarm_2_joint"};
        std::vector<double> positions = {joints[0].cmd, joints[1].cmd, joints[2].cmd, joints[3].cmd, joints[4].cmd};

        // RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Writing positions: %f, %f, %f, %f, %f", positions[0], positions[1], positions[2], positions[3], positions[4]);
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
