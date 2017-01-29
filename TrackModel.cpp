#include "TrackModel.h"

TrackModel::TrackModel() : position{linalg::transform()}, middle{{}}, left{{}}, right{{}}, madeLoop{false}, lastSearch{0}, firstUpdateDone{false}, firstDataProcessed{false}, trackWidth{0}, lastDistFromStart{0}
{

}

void TrackModel::continueBuilding(const CarState& cs)
{
	double distFromStart = cs.getDistFromStart();
	double angle = cs.getAngle();
	double trackPos = cs.getTrackPos();
	double leftDistance = cs.getTrack(0);
	double rightDistance = cs.getTrack(18);


	if(!firstDataProcessed)
	{
		trackWidth = leftDistance + rightDistance;
		firstDataProcessed = true;
		std::cout << "TrackWidth = " << trackWidth << "\n";
	}

	if(!firstUpdateDone && distFromStart - lastDistFromStart < 0)
	{
		position.lin.x = distFromStart;
		position.lin.y = trackPos * trackWidth * 0.5;
		firstUpdateDone = true;
	}
	if(firstUpdateDone)
	{
		std::cout << "------------\n";
		std::cout << "distFromStart = " << distFromStart << "\n";
		std::cout << "angle = " << angle << "\n";
		std::cout << "trackPos = " << trackPos << "\n";
		std::cout << "leftDistance = " << leftDistance << "\n";
		std::cout << "rightDistance = " << rightDistance << "\n";
		std::cout << "Position lin " << position.lin << "\n";
		std::cout << "Position ang " << position.ang << "\n";
		std::cout << "------------\n";
	}

	linalg::vector vel = 1000. / 3600. * linalg::vector{cs.getSpeedX(), cs.getSpeedY(), 0};
	// 20ms
	double deltaTime = 0.020;

	position.ang.z += deltaTime * angle;
	linalg::matrix rotationMatrix = linalg::matrix::rotation(position.ang.z);
	auto globalVel = rotationMatrix * vel;
	auto globalDeltaPos = deltaTime * globalVel;
	position.lin = position.lin + globalDeltaPos;

	lastDistFromStart = distFromStart;
	lastAngle = angle;





}

bool TrackModel::doneBuilding()
{
	return false;
}

void TrackModel::initialize()
{

}

linalg::vector TrackModel::getPosition(const CarState& cs, const ForwardModel& fModel)
{
	return position.lin;
}

bool TrackModel::outOfTrack(const ForwardModel& fModel, const linalg::vector& point)
{
	return false;
}




