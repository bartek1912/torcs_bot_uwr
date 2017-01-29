#include "engine.h"
#include <iostream>
#include <assert.h>
Engine::Engine()
	:rpm(-1), gear(-1), 
	rpms{104.72, 209.44, 314.159, 418.879, 523.599, 628.319, 733.038, 837.758, 942.478, 1047.2},
	alpha{0.572958, 0.286479, 0.859437, 0.668451, 0.525211, 0.362873, 0.210084, 0.171887, -0.649352, -0.525211},
	b{100, 130, 10, 70, 130, 215, 311, 339, 1027, 910}
	{}
double rpmToRads(double rpm)
{
	return rpm/5282*543.594;
}
double Engine::getGearMul()
{
	switch(gear)
	{
		case 0: return 0;
		case 1: return 13.5;
		case 2: return 8.55;
		case 3: return 6.3;
		case 4: return 4.95;
		case 5: return 4.05;
		case 6: return 3.465;
		default:
		std::cerr<<"Nieznany bieg "<<gear<<" w engine.cpp";
		return 0;
	}
}
double Engine::getTorque(double accel)
{
	assert(accel >= 0 && accel <= 1);
	double et = getEngineTorque(accel);
	return et * getGearMul();
}
double Engine::getEngineTorque(double accel)
{
	assert(accel >= 0 && accel <= 1);
	double rads = rpmToRads(rpm);
	for(size_t i = 0; i < rpms.size(); i++)
	{
		if(rads < rpms[i])
		{
			double TqMax = alpha[i]*rads + b[i];
			double engineBrk =  0.33* (rads - 94.2478) / (1047.2 - 94.2478);//engine.cpp:134
			double tq = TqMax * (accel * (1.0 + engineBrk) - engineBrk);
			//std::cerr<<"torque "<<tq<<" => "<<fixed<<tq*13.5<<"\t\t\t\t\t\n"; 
			return tq;
		}
	}
	std::cerr<<"Obroty poza zakresem\n";
	return 0;
}
int Engine::getNewRpm(int rpm)
{
	return rpm;
}

void Engine::setRpm(double newRpm)
{
	rpm = newRpm;
}


void Engine::setGear(int newGear)
{
	gear = newGear;
}

