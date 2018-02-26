# SIC
PLC project in ANSI-C

The control.h header contains the definition and the implementation of a discrete PID function.

#How to use it

command = pid(Kp, Ki, Kd, dt, setpoint, measured_value)
where,
Kp,Ki,Kd are the PID parameters
dt is the sampling time
setpoint is the reference 
measured_value is the output of the process at the current time.
