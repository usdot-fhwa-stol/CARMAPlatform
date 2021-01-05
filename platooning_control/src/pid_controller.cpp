
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

#include "pid_controller.hpp"



namespace platoon_control
{
	PIDController::PIDController(){}
	
	double PIDController::calculate( double setpoint, double pv ){

		// Calculate error
	    double error = setpoint - pv;

	    // Proportional term
	    double Pout = _Kp * error;

	    // Integral term
	    _integral += error * _dt;
		if (_integral > integratorMax){
			 _integral = integratorMax;
		}
		else if (_integral < integratorMin){
			_integral = integratorMin;
		}
	    double Iout = _Ki * _integral;

	    // Derivative term
	    double derivative = (error - _pre_error) / _dt;
	    double Dout = _Kd * derivative;

	    // Calculate total output
	    double output = Pout + Iout + Dout;

	    // Restrict to max/min
	    if( output > _max )
	        output = _max;
	    else if( output < _min )
	        output = _min;

	    // Save error to previous error
	    _pre_error = error;

	    return output;

	}



    void PIDController::reset() {
        _integral = 0.0;
        _pre_error = 0.0;
    }

}