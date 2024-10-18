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

        double kp = 0.05;             // proportional gain
        double ki = 0.0;             // integral gain
        double kd = 0.0;             // derivative gain
        double integral = 0.0;       // integral term
        double previous_error = 0.0; // previous error


        Joint(const std::string &joint_name, double kp = 0.05, double ki = 0.0, double kd = 0.0)
        {
            setup(joint_name);
        }

        void setup(const std::string &joint_name, double kp = 0.05, double ki = 0.0, double kd = 0.0)
        {
            name = joint_name;
            this->kp = kp;
            this->ki = ki;
            this->kd = kd;
        }
    };
}

#endif