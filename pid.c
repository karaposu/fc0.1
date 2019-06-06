#include "pid.h"

 
	



					float pid_calculate(float axis_angle  ,float set_point  , float kp,float ki ,float kd ,float dt , float old_axis_error,float current_axis_error)
{
float axis_angle_error=0;                     float angle_difference=0;float iTerm=0;  dt=1;
	int16_t pwm_cikis=0;
	
axis_angle_error=set_point-axis_angle;
current_axis_error=axis_angle_error;

//------------------------------------------------------------------------------------------CALCULATE RAW_PID
float pTerm = kp * current_axis_error;
iTerm += ki * current_axis_error * dt;  
float dTerm = kd  * (current_axis_error - old_axis_error)/dt;


// raw_pid = pTerm + dTerm+ iTerm;
pwm_cikis = pTerm + dTerm;  

	return pwm_cikis;
 
                     
}		
							 

