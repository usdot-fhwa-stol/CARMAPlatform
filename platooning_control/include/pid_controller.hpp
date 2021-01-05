
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.

------------------------------------------------------------------------------*/

#pragma once

#include <ros/ros.h>





namespace platoon_control
{


    class PIDController
    {
    public:

    	PIDController();
    	
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

    	double _dt = 0.1;
        double _max = 100.0;
        double _min = -100.0;
        double _Kp = 1.0;
        double _Kd = -0.5;
        double _Ki = 0.0;
        double _pre_error = 0.0;
        double _integral = 0.0;

        double integratorMax = 100;
        double integratorMin = -100;

    };
}