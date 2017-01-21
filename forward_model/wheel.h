#ifndef __WHEEL_H__
#define __WHEEL_H__

struct Brake;
struct Car;

#include "vector.h"
#include "car.h"
#include "brake.h"

struct Wheel
{
	//statyczna pozycja
	linalg::transform pos;
	//globalna pozycja
	linalg::transform globalPos;

	//Promien felgi + grubosc opony
	double radius;
	double frictionCoeff;
	double mass;
	//Nacisk na koło
	double weightOnWheel;
	double inertia;


	//Rad/s
	double spinVel;
	//Radiany?
	double steer;

	Brake* brake;

	double rollResistance;
	linalg::vector force;

	Wheel(int index, Car* car);

	void UpdateWheel(double deltaTime);
	void UpdateForces(double deltaTime);
};

#endif //__WHEEL_H_
