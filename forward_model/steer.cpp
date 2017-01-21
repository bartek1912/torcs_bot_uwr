#include "steer.h"

Steer::Steer(Car* car) : car(car) 
{
	steer = 0;
	steerLock = 0.43;
	maxSpeed = 1.0;
}

void Steer::applySteer(double newSteer, double deltaTime)
{
	double oldSteer, steer2;
	double deltaSteer;
	double tanSteer;

	oldSteer = steer;

	/* input control */
	newSteer *= steerLock;
	deltaSteer = newSteer - oldSteer;

	if ((fabs(deltaSteer) / deltaTime) > maxSpeed) {
		steer = SIGN(deltaSteer) * maxSpeed * deltaTime + oldSteer;
	}

	tanSteer = fabs(tan(steer));
	steer2 = atan2((car->wheelbase * tanSteer) , (car->wheelbase - tanSteer * car->wheeltrack));

	if (steer > 0) {
		car->wheels[FRNT_RGT]->steer = steer2;
		car->wheels[FRNT_LFT]->steer = steer;
	} else {
		car->wheels[FRNT_RGT]->steer = steer;
		car->wheels[FRNT_LFT]->steer = -steer2;
	}
}
