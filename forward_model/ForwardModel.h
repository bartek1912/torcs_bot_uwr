#ifndef FORWARD_MODEL_H_
#define FORWARD_MODEL_H_
#include "car.h"
#include "vector.h"
#include "matrix.h"
#include "CarState.h"
#include "CarControl.h"
#include <array>
#include <vector>

struct ForwardModel
{
	ForwardModel();
	ForwardModel(CarState& cs);

	void updateModel(CarState& cs);
	void simulate(double deltaTime,CarControl& c);
	Car getCarState();
private:
	Car car;
};
#endif
