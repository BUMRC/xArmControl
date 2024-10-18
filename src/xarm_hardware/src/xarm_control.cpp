#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware/xarm_control.hpp"
#define MAX_STR 255
#define PI 3.14159265359

using std::string;
namespace xarm_control
{
	XArmControl::XArmControl()
	{
		is_connected = false;
		// Dictionary of joint_names to joint_id
		joint_name_map.insert(std::make_pair("xarm_2_joint", 2));
		joint_name_map.insert(std::make_pair("xarm_3_joint", 3));
		joint_name_map.insert(std::make_pair("xarm_4_joint", 4));
		joint_name_map.insert(std::make_pair("xarm_5_joint", 5));
		joint_name_map.insert(std::make_pair("xarm_6_joint", 6));

		matrix_unit_transform["xarm_2_joint"][0][0] = 200;
		matrix_unit_transform["xarm_2_joint"][0][1] = 980;
		matrix_unit_transform["xarm_3_joint"][0][0] = 140;
		matrix_unit_transform["xarm_3_joint"][0][1] = 880;
		matrix_unit_transform["xarm_4_joint"][0][0] = 870;
		matrix_unit_transform["xarm_4_joint"][0][1] = 130;
		matrix_unit_transform["xarm_5_joint"][0][0] = 140;
		matrix_unit_transform["xarm_5_joint"][0][1] = 880;
		matrix_unit_transform["xarm_6_joint"][0][0] = 90;
		matrix_unit_transform["xarm_6_joint"][0][1] = 845;

		// First column values for -pi/2 and 2nd column pi/2
		matrix_unit_rad[0][0] = 100; // Gripper opened
		matrix_unit_rad[0][1] = 800; // Gripper closed
		matrix_unit_rad[1][0] = 200; /*  Joint 2 */
		matrix_unit_rad[1][1] = 980;
		matrix_unit_rad[2][0] = 140; /*  Joint 3*/
		matrix_unit_rad[2][1] = 880;
		matrix_unit_rad[3][0] = 130; /*  Joint 4 */
		matrix_unit_rad[3][1] = 870;
		matrix_unit_rad[4][0] = 140; /*  Joint 5 */
		matrix_unit_rad[4][1] = 880;
		matrix_unit_rad[5][0] = 90; /*  Joint 6 */
		matrix_unit_rad[5][1] = 845;
	}

	void XArmControl::connect()
	{
		if (is_connected)
		{
			RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Device already connected");
			return;
		}
		// Initialize the hidapi library
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Initializing hidapi library");
		if (hid_init() != 0)
		{
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to initialize hidapi library");
			return;
		}

		const unsigned short XARM_VENDOR_ID = 0x0483;  // Example vendor ID
		const unsigned short XARM_PRODUCT_ID = 0x5750; // Example product ID

		// Enumerate and print all HID devices for debugging
		struct hid_device_info *devs, *cur_dev;
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;
		while (cur_dev)
		{
			RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"),
						"Device Found\n  VID: %04hx\n  PID: %04hx\n  Path: %s\n  Serial Number: %ls\n  Manufacturer: %ls\n  Product: %ls",
						cur_dev->vendor_id, cur_dev->product_id, cur_dev->path,
						cur_dev->serial_number, cur_dev->manufacturer_string, cur_dev->product_string);
			cur_dev = cur_dev->next;
		}
		handle = hid_open(XARM_VENDOR_ID, XARM_PRODUCT_ID, NULL);

		if (!handle)
		{
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to open device. Error: %ls", hid_error(handle));
			hid_free_enumeration(devs);
			throw std::runtime_error("Failed to open xArm device");
		}
		is_connected = true;
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Device opened successfully");
		hid_free_enumeration(devs);
	}

	XArmControl::~XArmControl()
	{
		disconnect();
	}

	void XArmControl::disconnect()
	{
		hid_close(handle);
		hid_exit();
		is_connected = false;
	}

	void XArmControl::printDeviceInformation()
	{
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;
		while (cur_dev)
		{
			printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
			printf("\n");
			printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
			printf("  Product:      %ls\n", cur_dev->product_string);
			printf("  Release:      %hx\n", cur_dev->release_number);
			printf("  Interface:    %d\n", cur_dev->interface_number);
			printf("\n");

			cur_dev = cur_dev->next;
		}
	}

	int XArmControl::convertRadToUnit(std::string joint_name, double rad)
	{
		int unit;
		double m = (matrix_unit_transform[joint_name][0][1] - matrix_unit_transform[joint_name][0][0]) / (PI);
		double b = matrix_unit_transform[joint_name][0][1] - (m * PI / 2);
		unit = (int)((m * rad) + b);
		return unit;
	}

	double XArmControl::convertUnitToRad(std::string joint_name, int unit)
	{
		double rad;
		double m = (PI) / (matrix_unit_transform[joint_name][0][1] - matrix_unit_transform[joint_name][0][0]);
		double b = (PI / 2) - (m * matrix_unit_transform[joint_name][0][1]);
		rad = (m * unit) + b;
		return rad;
	}
	std::vector<double> XArmControl::readJointsPositions(std::vector<std::string> joint_names)
	{
		if (!is_connected)
		{
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Device not connected");
			return {};
		}
		std::vector<double> joint_positions;
		int res;
		// std::vector<double> joint_positions;
		unsigned char buf[65];

		joint_positions.resize(joint_names.size());
		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 9;
		buf[3] = 21;
		buf[4] = 6;
		buf[5] = 1;
		buf[6] = 2;
		buf[7] = 3;
		buf[8] = 4;
		buf[9] = 5;
		buf[10] = 6;
		res = hid_write(handle, buf, 17);

		if (res < 0)
		{
			printf("Unable to write()\n");
			printf("Error: %ls\n", hid_error(handle));
		}

		res = 0;

		while (res == 0)
		{
			res = hid_read(handle, buf, sizeof(buf));
			if (res == 0)
			{
				printf("waiting...\n");
			}
			if (res < 0)
				printf("Unable to read()\n");

			//  sleep for 500ms
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

		int id, p_lsb, p_msb, pos, unit, joint_id;
		for (int i = 0; i < joint_names.size(); i++)
		{
			joint_id = joint_name_map[joint_names[i]];
			id = buf[2 + 3 * joint_id];
			p_lsb = buf[2 + 3 * joint_id + 1];
			p_msb = buf[2 + 3 * joint_id + 2];
			unit = (p_msb << 8) + p_lsb;
			joint_positions[i] = convertUnitToRad(joint_names[i], unit);
			// printf("servo %d in joint_position %f \n", id, joint_positions[i]);
		}

		return joint_positions;
	}

	void XArmControl::setJointPosition(std::string joint_name, double position_rad, int time = 100)
	{
		if (!is_connected)
		{
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Device not connected");
			return;
		}

		unsigned char buf[65];
		unsigned char t_lsb, t_msb, p_lsb, p_msb;
		int res;
		int position_unit = int(convertRadToUnit(joint_name, position_rad));

		t_lsb = time & 0xFF;
		t_msb = time >> 8;
		p_lsb = position_unit & 0xFF;
		p_msb = position_unit >> 8;

		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 8;
		buf[3] = 0x03;
		buf[4] = 1;
		buf[5] = t_lsb;
		buf[6] = t_msb;
		buf[7] = joint_name_map[joint_name];
		buf[8] = p_lsb;
		buf[9] = p_msb;

		res = hid_write(handle, buf, 17);

		if (res < 0)
		{
			printf("Unable to write()\n");
			printf("Error: %ls\n", hid_error(handle));
		}
	}

}
