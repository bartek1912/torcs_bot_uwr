#include "ForwardModel.h"

ForwardModel::
	ForwardModel(CarState& cs):
	car()
{ }

void ForwardModel::updateModel(CarState& cs)
{
	car.set(cs);
}

void ForwardModel::simulate(double deltaTime, CarControl& c)
{
	car.simulate(deltaTime, c);
}

Car ForwardModel::getCarState()
{
	return car;
}
