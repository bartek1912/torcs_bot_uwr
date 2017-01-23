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
	BrakeSystem(const BrakeSystem& brakeSystem);

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
	Brake(const Brake& brake);
};

#endif //__BRAKE_H__
