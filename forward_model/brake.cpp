#include "brake.h"

BrakeSystem::BrakeSystem(Car* car) : car(car)
{
	maxPressure = 29000000;	
	repartition = 0.54;
}

Brake::Brake(int index)
{
	diameter = index < 2 ? 0.038 : 0.030;
	radius = 0.5 * diameter;
	area = index < 2 ? 0.005 : 0.0025;
	frictionCoeff = 0.4;
	pressure = 0;
	inertia = index < 2 ? 0.1241 : 0.0714;
	torque = 0;
}

void BrakeSystem::applyBrake(double brake)
{
	brake *= maxPressure;
	car->wheels[FRNT_RGT]->brake->pressure = car->wheels[FRNT_LFT]->brake->pressure = brake * repartition;
	car->wheels[REAR_RGT]->brake->pressure = car->wheels[REAR_LFT]->brake->pressure = brake * (1-repartition);

	for(int i=0;i<4;i++)
	{
		Brake* brake = car->wheels[i]->brake;
		//Promien * siła tarcia = moment siły
		brake->torque = brake->radius * (brake->pressure * brake->area * brake->frictionCoeff);
	}
}
