
#ifndef XARM_CONTROL_HPP_
#define XARM_CONTROL_HPP_
#include <hidapi/hidapi.h>
#include <sstream>
#include <map>

namespace xarm_control
{
	class xarm_control
	{
	public:
		xarm_control();
		~xarm_control();

		std::vector<double> readJointsPositions(std::vector<std::string> joint_names);
		void setJointPosition(std::string joint_name, double position_rad, int time);
		double convertUnitToRad(std::string joint_name, int unit);
		int convertRadToUnit(std::string joint_name, double rad);
		void connect();
		void disconnect();

	private:
		hid_device *handle;
		struct hid_device_info *devs, *cur_dev;
		void printDeviceInformation();
		int matrix_unit_rad[6][2];
		std::map<std::string, int> joint_name_map;
		std::map<std::string, int[1][2]> matrix_unit_transform;
	};
}

#endif