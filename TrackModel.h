#ifndef __TRACK_MODEL_H__
#define __TRACK_MODEL_H__

#include "interfaces/CarState.h"
#include "forward_model/vector.h"
#include "forward_model/ForwardModel.h"
#include <vector>
#include <string>

struct TrackModel
{
	linalg::transform position;
	vector<linalg::vector> middle, left, right;	
	//Kiedy dojechalismy do poczatku
	bool madeLoop;
	//Indeks w middle, w ktorym ostatnio bylismy / szukalismy czegos
	int lastSearch;
	//firstUpdate
	bool firstUpdateDone;
	//Pierwsze dane jakie dostajemy - ideany trackwidth
	bool firstDataProcessed;

	double trackWidth;
	double lastDistFromStart;
	double lastAngle;

	TrackModel();

	void initialize();
	void continueBuilding(const CarState& cs);
	bool doneBuilding();

	linalg::vector getPosition(const CarState& cs, const ForwardModel& fModel);
	bool outOfTrack(const ForwardModel& fModel, const linalg::vector& point);

	void readFromFile(std::string filename);
	void saveToFile(std::string filename);
};

#endif //__TRACK_MODEL_H__

