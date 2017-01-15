#include "ForwardModel.h"

ForwardModel::ForwardModel(
		linalg::vector pos, 
		linalg::vector orient,
		double rpm, 
		linalg::vector vel, 
		int gear, 
		double clutch,
		std::array<double, 4> wheelSpeeds,
		std::pair<std::vector<linalg::vector>, std::vector<linalg::vector> >& track ) : 
	pos{pos}, orient{orient}, vel{vel}, rpm{rpm}, gear{gear}, clutch{clutch}, wheelSpeeds{wheelSpeeds}, track{track}
{ 
	configure(); 
}

void ForwardModel::reset(
		linalg::vector pos,
		linalg::vector orient,
		double rpm,
		linalg::vector vel,
		int gear,
		double clutch,
		std::array<double, 4> wheelSpeeds)
{
	this->pos = pos;
	this->orient = orient;
	this->rpm = rpm;
	this->vel = vel;
	this->gear = gear;
	this->clutch = clutch;
	this->wheelSpeeds = wheelSpeeds;

	configure();	
}

void ForwardModel::applyMove(double deltaTime, int gear, double brakes, double accel, double steer, double clutch)
{
	applyGear(gear);
	applyBrakes(brakes);
	applyAcceleration(accel);
	applySteer(steer);
	applyClutch(clutch);
	simulateTimestep(deltaTime);
}

void ForwardModel::configure()
{

}

void ForwardModel::applyGear(int gear)
{

}

void ForwardModel::applyBrakes(double brakes)
{

}

void ForwardModel::applyAcceleration(double accel)
{

}

void ForwardModel::applySteer(double steer)
{

}

void ForwardModel::applyClutch(double clutch)
{

}

void ForwardModel::simulateTimestep(double deltaTime)
{

}

bool ForwardModel::isOutOfTrack()
{
	return false;
}
