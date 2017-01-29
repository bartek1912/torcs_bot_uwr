#include "ForwardModel.h"

ForwardModel::ForwardModel() : car(Car())
{}

ForwardModel::
	ForwardModel(CarState& cs):
	car()
{ updateModel(cs);}

void ForwardModel::updateModel(CarState& cs)
{
	car.set(cs);
}

void ForwardModel::simulate(double deltaTime, CarControl& c)
{
	for(double act = 0.0; act < deltaTime; act+= 0.002)
		car.simulate(act, c);
}

Car ForwardModel::getCarState()
{
	return car;
}
