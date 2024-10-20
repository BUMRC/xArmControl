#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <cmath>

namespace pid_controller
{
    class PIDController
    {
    public:
        double kp = 0.05;             // proportional gain
        double ki = 0.001;             // integral gain
        double kd = 0.01;             // derivative gain

        double integral = 0.0;
        double previous_error = 0.0;

        double clamp_integral_max = 1.0;
        double clamp_integral_min = -1.0;

        PIDController(double kp = 0.05, double ki = 0.0, double kd = 0.0)
        {
            setup(kp, ki, kd);
        }

        void setup(double kp = 0.05, double ki = 0.0, double kd = 0.0)
        {
            this->kp = kp;
            this->ki = ki;
            this->kd = kd;
        }

        double calculate(double setpoint, double current, double dt)
        {
            dt = std::max(dt, 0.000001);
            double error = setpoint - current;
            integral += error * dt;
            integral = std::max(std::min(integral, clamp_integral_max), clamp_integral_min);
            double derivative = (error - previous_error) / dt;
            previous_error = error;
            return kp * error + ki * integral + kd * derivative;
        }

        void reset()
        {
            integral = 0.0;
            previous_error = 0.0;
        }
    };
}

#endif