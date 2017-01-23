#ifndef __STEER_H__
#define __STEER_H__


#include "vector.h"
#include "car.h"


struct Steer
{
	double steer;
	double steerLock;
	double maxSpeed;
	
	Car* car;	
	
	Steer(Car* car);
	Steer(const Steer& steer);

	void applySteer(double newSteer, double deltaTime);
};

#endif //__STEER_H__
