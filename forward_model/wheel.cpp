#include "wheel.h"

Wheel::Wheel(int index, Car* car) : index(index), car(car)
{
	brake = new Brake(index);
	//radius = rimdiam / 2 + tirewidth * tireratio;
	mass = 20;
	frictionCoeff = 1.6;

	inertia = 1.5;
	inertia += brake->inertia;

	steer = 0;
	
	if(index < 2)
	{
		radius = 0.330600;
		pos.lin.x = 1.267200;
		pos.lin.y = index % 2 ? -0.84 : 0.84;
		weightOnWheel = (car->mass * 0.5 * 1.372800) / (1.267200 * (1 + 1.372800)) * G;
	}
	else
	{
		radius = 0.327600;
		pos.lin.x = -1.372800;
		pos.lin.y = index % 2 ? -0.80 : 0.80;
		weightOnWheel = (car->mass * 0.5 * 1.267200) / (1.372800 * (1 + 1.267200)) * G;
	}

	globalPos.lin = pos.lin;

	mfB = 14.187762;
	mfC = 1.409666;
	mfE = 0.7;
	simSkidFactor = 0.3;
	averageTrackFriction = 1.15;
	averageOutOfTrackFriction = 0.50;
	trackRollResistance = 0.001200; 
}

void Wheel::UpdateForces(double deltaTime)
{
	double angleZ = steer + pos.ang.z;
	double cosAngZ = cos(angleZ);
	double sinAngZ = sin(angleZ);

	//Using rigid speed function to find out the local velocity of the wheel
	//V(x) = v_0 + crossProd(omega, x)
	linalg::vector localVel = car->vel.lin + linalg::crossproduct(car->vel.ang, pos.lin);

	//tangent velocity
	double vt = localVel.x * cosAngZ + localVel.y * sinAngZ;	
	double v2 = localVel.x * localVel.x + localVel.y * localVel.y;
	double v = sqrt(v2);

	// slip angle
	double slipAng = 0;
	double slip, slipX, slipY;
	if (v < 0.000001f) {
		slipAng = 0.0f;
	} else {
		slipAng = atan2(localVel.y, localVel.x) - angleZ;
	}
	NORM_PI_PI(slipAng);

	double linearRollSpeed = spinVel * radius;
	if (v < 0.000001f) {
		slipX = linearRollSpeed;
		slipY = 0.0f;
	} else {
		slipX = (vt - linearRollSpeed) / fabs(vt);
		slipY = sin(slipAng);
	}

	//Siły dzialajace na kolo ale nie wiem jakie (wzdluz kola i prostopadla do kola???)
	double Ft = 0.0f;
	double Fn = 0.0f;
	slip = sqrt(slipX*slipX+slipY*slipY);

	//Temporary slip? tak to wyglada
	double stmp = MIN(slip, 1.5f);

	// MAGIC FORMULA
	double Bx = mfB * stmp;
	double F = sin(mfC * atan(Bx * (1.0f - mfE) + mfE * atan(Bx))) * (1.0f + stmp * simSkidFactor);

	//W oryginale to bylo force.z a nie weight on wheel
	//Sila bardzo niedoszacowana - force.z jest tym wieksze im szybciej sie jedzie
	F *= weightOnWheel * frictionCoeff * averageTrackFriction * (1.0f + 0.05f * sin(-pos.ang.x * 18.0f));	/* coeff */

	rollResistance = weightOnWheel * trackRollResistance;

	if (slip > 0.000001f) {
		// wheel axis based
		Ft -= F * slipX / slip;
		Fn -= F * slipY / slip;
	}

	RELAXATION2(Fn, prevFn, 50.0f);
	RELAXATION2(Ft, prevFt, 50.0f);

	force.x = Ft * cosAngZ - Fn * sinAngZ;
	force.y = Ft * sinAngZ + Fn * cosAngZ;

	//Nie wiem, czy spin torque sie w ogole do czegokolwiek przydaje
	spinTorque = Ft * radius;
}

void UpdateFreeWheels(Car* car, int axleNumber, double deltaTime)
{
	int i;
	Wheel* wheel;
	double BrTq;		// brake torque
	double ndot;		// rotation acceleration

	for (i = axleNumber * 2; i < axleNumber * 2 + 2; i++) {
		wheel = car->wheels[i];

		ndot = deltaTime * wheel->spinTorque / wheel->inertia;
		wheel->spinVel -= ndot;

		BrTq = - SIGN(wheel->spinVel) * wheel->brake->torque;
		ndot = deltaTime * BrTq / wheel->inertia;

		if (fabs(ndot) > fabs(wheel->spinVel)) {
			ndot = -wheel->spinVel;
		}

		wheel->spinVel += ndot;
	}
}

void UpdatePropelledWheels(Car* car, int axleNumber, double deltaTime)
{
	int i;
	Wheel* wheel;
	double BrTq;		// brake torque
	double ndot;		// rotation acceleration

	for (i = axleNumber * 2; i < axleNumber * 2 + 2; i++) {
		wheel = car->wheels[i];

		ndot = deltaTime * wheel->engineTorque / wheel->inertia;
		wheel->spinVel += ndot;

		ndot = deltaTime * wheel->spinTorque / wheel->inertia;
		wheel->spinVel -= ndot;

		BrTq = - SIGN(wheel->spinVel) * wheel->brake->torque;
		ndot = deltaTime * BrTq / wheel->inertia;

		if (fabs(ndot) > fabs(wheel->spinVel)) {
			ndot = -wheel->spinVel;
		}

		wheel->spinVel += ndot;
	}
}

void UpdateWheelsRotation(Car* car, double deltaTime)
{
	int i;
	Wheel *wheel;

	for (i = 0; i < 4; i++) {
		wheel = car->wheels[i];

		RELAXATION2(wheel->spinVel, wheel->prespinVel, 50.0f);
	}
}

void UpdateWheels(Car* car, double deltaTime)
{
	for(int i =0; i<4;i++)
		car->wheels[i]->UpdateForces(deltaTime);

	UpdatePropelledWheels(car, REAR, deltaTime);
	UpdateFreeWheels(car, FRNT, deltaTime);
	UpdateWheelsRotation(car, deltaTime);
}
