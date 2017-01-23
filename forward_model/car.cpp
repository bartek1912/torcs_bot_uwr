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

	mass = 1150;

	dimensions = {4.7, 1.9, 1.2};
	inertia = 
		{dimensions.y * dimensions.y + dimensions.z * dimensions.z,
		dimensions.x * dimensions.x + dimensions.z * dimensions.z,
		dimensions.x * dimensions.x + dimensions.y * dimensions.y} ; 
	inertia *= (mass / 12);
}

void Car::set(CarState& cs)
{
	//Czy angle, distFromStart, trackPos nie powinno wpływać na pos.lin, pos.ang?
	vel.lin = linalg::vector(cs.getSpeedX(), cs.getSpeedY(), 0 /*cs.getSpeedZ*/);	
	for(int i=0; i<4;i++)
	{
		wheels[i]->spinVel = cs.getWheelSpinVel(i);
	}
	engine.setRpm(cs.getRpm());
	engine.setGear(cs.getGear());
}

void Car::simulate(double deltaTime, CarControl& c)
{
	applyControl(deltaTime, c.getSteer(), c.getBrake(), c.getAccel());
	updatePhysics(deltaTime);
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
	linalg::normalizePI_PI(pos.ang);

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
	//Symulacja fizyki kół, która oblicza siły działające na koła
	UpdateWheels(this, deltaTime);

	//Aktualizacja fizyki całego samochodu, która korzysta z sił kół
	updateAcceleration(deltaTime);
	updateVelocity(deltaTime);
	updatePosition(deltaTime);
}

void Car::applyControl(double deltaTime, double steer, double brake, double accel)
{
	this->steer->applySteer(steer, deltaTime);
	this->brakeSystem->applyBrake(brake);
	auto torque = engine.getTorque(accel);
	wheels[REAR_LFT]->engineTorque += torque/2;
	wheels[REAR_RGT]->engineTorque += torque/2;
	//this->engine->applyEngine(aceel, gear, clutch);
}

Car::Car(const Car& car)
{
	pos = car.pos;
	vel = car.vel;
	acc = car.acc;

	for(int i=0;i<4;i++)
	{
		wheels[i] = new Wheel(*car.wheels[i]);
		wheels[i]->car = this;
	}

	brakeSystem = new BrakeSystem(*car.brakeSystem);
	brakeSystem->car = this;

	steer = new Steer(*car.steer);
	steer->car = this;

	engine = Engine(car.engine);

	mass = car.mass;
	inertia = car.inertia;
	wheelbase = car.wheelbase;
	wheeltrack = car.wheeltrack;
	dimensions = car.dimensions;
}
