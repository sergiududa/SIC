#ifndef _CONTROL_H
#define _CONTROL_H

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
