#include "car.h"
#include "matrix.h"

Car::Car()
{
	brakeSystem = new BrakeSystem(this);
	for(int i=0;i<4;i++)
	{
		wheels[i] = new Wheel(i, this);
	}
	steer = new Steer(this);

	wheelbase = (wheels[FRNT_RGT]->pos.lin.x 
				+ wheels[FRNT_LFT]->pos.lin.x
				- wheels[REAR_RGT]->pos.lin.x
				- wheels[REAR_LFT]->pos.lin.x) / 2.0;

	wheeltrack = (wheels[FRNT_RGT]->pos.lin.y 
				+ wheels[FRNT_LFT]->pos.lin.y
				- wheels[REAR_RGT]->pos.lin.y
				- wheels[REAR_LFT]->pos.lin.y) / 2.0;
}

void Car::updateAcceleration(double deltaTime)
{
	linalg::vector totalForce;
	linalg::vector totalTorque;	

	for(int i=0;i<4;i++)
	{
		totalForce += wheels[i]->force;
		totalTorque += linalg::crossproduct(wheels[i]->pos.lin, wheels[i]->force);
	}


	/* Rolling Resistance */
	double speed = sqrt(vel.lin.x * vel.lin.x + vel.lin.y * vel.lin.y);
	//Resistance force with direction opposite to the velocity
	double R = 0;
	double Rv = 0;
	//Moment siły wokol osi Z wynikający z oporu
	double Rmoment = 0;

	for(int i = 0; i < 4; i++)
	{
		R += wheels[i]->rollResistance;
	}
	if (speed > 0.00001) {
		//Rv to "znormalizowana" przez predkosc sila, sluzy do skalowania na oś X i Y
		Rv = R / speed;
		if (((Rv * deltaTime) / mass) > speed) {
			Rv = speed * mass / deltaTime;
		}
	} else {
		Rv = 0;
	}

	linalg::vector rollResistanceLin = {Rv * vel.lin.x, Rv * vel.lin.y, 0};
	
	if ((R * wheelbase / 2.0 / inertia.z) > fabs(vel.ang.z)) {
		Rmoment = vel.ang.z * inertia.z ;
	} else {
		Rmoment = SIGN(vel.ang.z) * R * wheelbase / 2.0;
	}

	acc.lin = (totalForce - rollResistanceLin) / mass;	
	acc.ang.x = totalTorque.x / inertia.x; 
	acc.ang.y = totalTorque.y / inertia.y; 
	acc.ang.z = (totalTorque.z - Rmoment) / inertia.z; 
}

void Car::updateVelocity(double deltaTime)
{
	vel.lin += acc.lin * deltaTime;
	vel.ang += acc.ang * deltaTime;
}

void Car::updatePosition(double deltaTime)
{
	linalg::vector deltaAng = vel.ang * deltaTime;
	pos.ang += deltaAng;
	linalg::normalize2PI(pos.ang);

	linalg::vector deltaLin = vel.lin * deltaTime;
	linalg::matrix rotationMatrix = linalg::matrix::rotation(pos.ang.z);
	linalg::vector globalDeltaLin = rotationMatrix * deltaLin; 

	pos.lin += globalDeltaLin;

	//Zaktualizowac global pozycje kol
	for(int i=0;i<4;i++)
	{
		wheels[i]->globalPos.lin = pos.lin + rotationMatrix * wheels[i]->pos.lin; 
		//wheels[i]->globalPos.ang nas nie obchodzi
	}
}

void Car::updatePhysics(double deltaTime)
{
	updateAcceleration(deltaTime);
	updateVelocity(deltaTime);
	updatePosition(deltaTime);
}
