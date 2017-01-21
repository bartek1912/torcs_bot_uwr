#ifndef __BRAKE_H__
#define __BRAKE_H__

struct Car;
#include "car.h"

struct BrakeSystem
{
	//Rozdzial hamowania na przednie i tylne kola
	double repartition;
	//Cisnienie hamulcow
	double maxPressure;

	Car* car;

	BrakeSystem(Car* car);

	void applyBrake(double brake);
};

struct Brake
{
	double area;
	double diameter;
	double radius;
	double frictionCoeff;
	double pressure;

	//Moment bezwladnosci hamulcow
	double inertia;
	double torque;

	Brake(int index);
};

#endif //__BRAKE_H__
