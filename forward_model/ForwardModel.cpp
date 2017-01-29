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
	car.simulate(deltaTime, c);
}

Car ForwardModel::getCarState()
{
	return car;
}
