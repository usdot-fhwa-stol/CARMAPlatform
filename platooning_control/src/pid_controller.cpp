#include "pid_controller.hpp"



namespace platoon_control
{
	PIDController::PIDController(){}
	
	double PIDController::calculate( double setpoint, double pv ){

		// Calculate error
	    double error = setpoint - pv;
		ROS_DEBUG_STREAM("PID error: " << error);

	    // Proportional term
	    double Pout = config_.Kp * error;
		ROS_DEBUG_STREAM("Proportional term: " << Pout);

	    // Integral term
	    _integral += error * config_.dt;
		ROS_DEBUG_STREAM("Integral term: " << _integral);

		if (_integral > config_.integratorMax){
			 _integral = config_.integratorMax;
		}
		else if (_integral < config_.integratorMin){
			_integral = config_.integratorMin;
		}
	    double Iout = config_.Ki * _integral;
		ROS_DEBUG_STREAM("Iout: " << Iout);

	    // Derivative term
	    double derivative = (error - _pre_error) / config_.dt;
		ROS_DEBUG_STREAM("derivative term: " << derivative);
	    double Dout = config_.Kd * derivative;
		ROS_DEBUG_STREAM("Dout: " << Dout);

	    // Calculate total output
	    double output = Pout + Iout + Dout;
		ROS_DEBUG_STREAM("total controller output: " << output);

	    // Restrict to max/min
	    if( output > config_.max_value )
	        output = config_.max_value;
	    else if( output < config_.min_value )
	        output = config_.min_value;

	    // Save error to previous error
	    _pre_error = error;

	    return output;

	}



    void PIDController::reset() {
        _integral = 0.0;
        _pre_error = 0.0;
    }

}