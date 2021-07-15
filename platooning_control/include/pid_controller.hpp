#pragma once

#include <ros/ros.h>
#include "platoon_control_config.h"





namespace platoon_control
{


    class PIDController
    {
    public:

    	PIDController();

        PlatooningControlPluginConfig config_;
    	
    	// ~PIDController();

        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        // PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        // ~PID();

        void reset();

        




    private:

        double _pre_error = 0.0;
        double _integral = 0.0;

    };
}