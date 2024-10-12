#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <string>
#include <vector>
#include <cmath>

namespace xarm_hardware
{
    class Joint
    {
    public:
        std::string name = "";

        // units here are temporary, change as needed
        int enc = 0;                 // encoder ticks
        double cmd = 0.0;            // command rad/s
        double pos = 0.0;            // radians
        double vel = 0.0;            // rad/s

        Joint(const std::string &joint_name)
        {
            setup(joint_name);
        }

        void setup(const std::string &joint_name)
        {
            name = joint_name;
        }
    };
}

#endif