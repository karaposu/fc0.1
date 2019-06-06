#ifndef PID_H_ 
#define PID_H_ 

#include "main.h" 
	float pid_calculate(float axis_angle  ,float set_point  , float kp,float ki ,float kd ,float dt , float old_axis_error,float current_axis_error);


extern uint8_t inside_the_deadzone(float g );
	void speedless_zone();
	void pid_control( );
// extern void Serial_write(char ptr)   ; 



#endif