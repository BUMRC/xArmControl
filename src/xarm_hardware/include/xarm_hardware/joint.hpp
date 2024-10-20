#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <string>
#include <vector>
#include <cmath>
#include "pid_controller.hpp"
using namespace pid_controller;

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

        PIDController pid;

        Joint(const std::string &joint_name, double kp, double ki, double kd)
        {
            pid = PIDController(kp, ki, kd);
            setup(joint_name, kp, ki, kd);
        }
        Joint(const std::string &joint_name, PIDController pid)
        {
            this->pid = pid;
            setup(joint_name, pid);
        }

        void setup(const std::string &joint_name, double kp, double ki, double kd)
        {
            name = joint_name;
            pid.setup(kp, ki, kd);
        }
        void setup(const std::string &joint_name, PIDController pid)
        {
            name = joint_name;
            this->pid = pid;
        }
        void reset()
        {
            pid.reset();
        }

        double calculateTarget(double dt)
        {
            return pid.calculate(cmd, pos, dt);
        }
    };
}

#endif