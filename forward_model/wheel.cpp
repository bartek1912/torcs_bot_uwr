#include "wheel.h"

Wheel::Wheel(int index, Car* car) 
{
	brake = new Brake(index);
	//radius = rimdiam / 2 + tirewidth * tireratio;
	radius = 0.33 * 0.5 + 0.145 * 0.75;
	mass = 20;
	frictionCoeff = 1.0;

	inertia = 1.5;
	inertia += brake->inertia;

	steer = 0;
	
	if(index < 2)
	{
		pos.lin.x = 1.313;
		pos.lin.y = index % 2 ? -0.84 : 0.84;
		weightOnWheel = (car->mass * 0.5 * 1.287) / (1.313 * (1 + 1.287));
	}
	else
	{
		pos.lin.x = -1.287;
		pos.lin.y = index % 2 ? -0.80 : 0.80;
		weightOnWheel = (car->mass * 0.5 * 1.313) / (1.287 * (1 + 1.313));
	}

	globalPos.lin = pos.lin;
}

void Wheel::UpdateWheel(double deltaTime)
{
}

void Wheel::UpdateForces(double deltaTime)
{
}


