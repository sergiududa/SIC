#ifndef _CONTROL_H
#define _CONTROL_H

/**************************************************************
	Calculate the command for a process using a PID controller.
	
	u(t) = Kp * e(t) + Ki * [sum(e(t))] + Kd * [d/dt * e(t)]
	
	The corespondence for each element in our pid function is:
	command 	- u(t)
	error 		- e(t) 
	integral 	- sum(e(t))
	derivative 	- d/dt * e(t) 
	
	To calculate te error we need the setpoint (reference) and
	the measured_value 
 	
	In a discrete environment: 
	* the derivative can be aproximated as follows: 
		( e[k] - e[k-1])/ dt, where dt is the sampling time

	* the integra; can be aproximated as a sum of all the previous 
	errors 
	
	source: https://en.wikipedia.org/wiki/PID_controller
*/

double pid(double Kp, double Ki, double Kd, double dt, double setpoint, double measured_value, double min, double max)
{
	double error, derivative, command;
	static double integral = 0.0, previous_error = 0.0, previous_setpoint = 0;
	
	if(previous_setpoint != setpoint)
	{	
		integral = 0.0;
		previous_error = 0.0;
		previous_setpoint = setpoint;	
	}	
	error = setpoint - measured_value;
  	integral = integral + error*dt;
  	derivative = (error - previous_error)/dt;
  	command = Kp*error + Ki*integral + Kd*derivative;
  	previous_error = error;
	
	if(command > max)
		command = max;
	if (command < min)
		command = min;
	
	return command;
}

double pid_incremental(double Kr, double Ti, double Td, double Te, double rk, double yk, double min, double max)
{
	static double uk = 0.0;
	static double uk_1 = 1;    // Valoare initiala comanda
	static double ek = 0.0;
	static double ek_1 = 0.0;
	static double ek_2 = 0.0;
	float q0 = 0.0,q1 = 0.0,q2 = 0.0;
	
	q0 = Kr*(1+Te/Ti+Td/Te);
	q1 = -Kr*(1+2*Td/Te);
	q2 = Kr*Te/Ti;
	
	ek = rk - yk;
	uk = uk_1 + q0*ek + q1*ek_1 + q2*ek_2;
	
	if(uk > max)
		uk = max;
	if (uk < min)
		uk = min;
	
	uk_1 = uk;
	ek_2 = ek_1;
	ek_1 = ek;
	
	
	return uk;
}

#endif
