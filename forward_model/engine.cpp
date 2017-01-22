#include "engine.h"
#include <iostream>
double rpmToRads(double rpm)
{
	return rpm/5282*543.594;
}
double Engine::getTorque(double accel, int gear, double rpm)
{
	double rads = rpmToRads(rpm);
	for(size_t i = 0; i < rpms.size(); i++)
		if(rads < rpms[i])
			return alpha[i]*rads + b[i];
	std::cerr<<"Obroty poza zakresem\n";
	return 0;
}
int Engine::getNewRpm(int rpm)
{
	return rpm;
}