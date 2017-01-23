#include "ForwardModel.h"

#define SIGN(x) (x < 0 ? -1 : x == 0 ? 0 : 1)

ForwardModel::
	ForwardModel(CarState& cs):
	car(cs)
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
