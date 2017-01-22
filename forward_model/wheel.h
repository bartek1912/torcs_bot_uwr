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

	int index;

	//Promien felgi + grubosc opony
	double radius;
	double frictionCoeff;
	double mass;
	//Nacisk na koło
	double weightOnWheel;
	double inertia;


	//Rad/s
	double spinVel;
	double prespinVel;

	double spinTorque;
	//Radiany?
	double steer;

	Car* car;
	Brake* brake;

	double rollResistance;
	linalg::vector force;

	//Magic formula coefficients
	double mfB;
	double mfC;
	double mfE;

	double simSkidFactor;
	double averageTrackFriction;
	double averageOutOfTrackFriction;
	double trackRollResistance;

	double prevFn, prevFt;

	Wheel(int index, Car* car);

	void UpdateForces(double deltaTime);
};

void UpdateFreeWheels(Car* car, int axleNumber, double deltaTime);
void UpdateWheelsRotation(Car* car, double deltaTime);
void UpdateWheels(Car* car, double deltaTime);

#endif //__WHEEL_H_
