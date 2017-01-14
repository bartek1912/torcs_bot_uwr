/***************************************************************************
 
    file                 : QLearningDriver.cpp
    copyright            : (C) 2016 Jarosław Dzikowski
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "QLearningDriver.h"
#include "QLearningConfig.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <chrono>


/* Gear Changing Constants*/
const int QLearningDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int QLearningDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };

/* Stuck constants*/
const int QLearningDriver::stuckTime = 25;
const float QLearningDriver::stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float QLearningDriver::maxSpeedDist=70;
const float QLearningDriver::maxSpeed=150;
const float QLearningDriver::sin5 = 0.08716;
const float QLearningDriver::cos5 = 0.99619;

/* Steering constants*/
const float QLearningDriver::steerLock=0.366519 ;
const float QLearningDriver::steerSensitivityOffset=80.0;
const float QLearningDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float QLearningDriver::wheelRadius[4]={0.3306,0.3306,0.3276,0.3276};
const float QLearningDriver::absSlip=2.0;
const float QLearningDriver::absRange=3.0;
const float QLearningDriver::absMinSpeed=3.0;

/* Clutch constants */
const float QLearningDriver::clutchMax=0.5;
const float QLearningDriver::clutchDelta=0.05;
const float QLearningDriver::clutchRange=0.82;
const float QLearningDriver::clutchDeltaTime=0.02;
const float QLearningDriver::clutchDeltaRaced=10;
const float QLearningDriver::clutchDec=0.01;
const float QLearningDriver::clutchMaxModifier=1.3;
const float QLearningDriver::clutchMaxTime=1.5;

const float INF = numeric_limits<float>::max(); 	

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
};

//File desc for track data file
ifstream centroidFile;
string centroidFilename = "centroids.dat";
//2d vector - centroids
vector<vector<float> > centroids;

//File desc for Q function:
fstream qFuncFile;
string qFuncFilename = "qFunc.dat";
//hashtable of hashtables 
unordered_map<pair<int,int>, vector<pair<float, int> >, pair_hash> Q;
//Q-Learning configuration
string qLearningConfigFilename = "qLearnConfig.dat";
QLearningConfig qLearnConfig;

const int TRACK_SENSORS = 19;
const int RESTART_ITERATION = 1000;

int iterationCount = 0;

const pair<int,int> NONE = {-1,-1};
//Needed for Q function update
pair<int,int> lastState, lastAction;
CarState lastCarState;
CarControl lastControl;

int makeRandom(int a, int b)
{
	return rand() % (b-a+1) + a;
}

template<typename TimeT = std::chrono::minutes, 
    typename ClockT=std::chrono::high_resolution_clock,
    typename DurationT=double>
struct Stopwatch
{
private:
    std::chrono::time_point<ClockT> _start, _end;
public:
    Stopwatch() { start(); }
    void start() { _start = _end = ClockT::now(); }
    DurationT stop() { _end = ClockT::now(); return elapsed();}
    DurationT elapsed() { 
        auto delta = std::chrono::duration_cast<TimeT>(_end-_start);
        return delta.count(); 
    }
};

Stopwatch<> stopwatch;
double saveFrequencyInMin = 5;

void readCentroids()
{
	//Jesli juz wczytane, to nie ma sensu znowu wczytywac
	if(!centroids.empty())
		return;

	cout<< "Opening centroids data file " << centroidFilename << "...\n";
	centroidFile.open(centroidFilename, ios::in);
	centroids.clear();	
	
	while(centroidFile.peek() != ifstream::traits_type::eof())
	{
		centroids.push_back({});
		centroids.back().resize(TRACK_SENSORS, -1.0);
		for(int i=0;i<TRACK_SENSORS;i++)
			centroidFile >> centroids.back()[i];
	}

	centroidFile.close();
}

void readQFunction()
{
	//Jesli juz wczytane, to nie ma sensu znowu wczytywac
	if(!Q.empty())
		return;

	cout<< "Opening qFunc data file " << qFuncFilename << "...\n";
	qFuncFile.open(qFuncFilename, ios::in);
	Q.clear();
	while(qFuncFile.peek() != ifstream::traits_type::eof())
	{
		//Track centroid, speed zone, steer zone, acceleration(-1 break, 1 gas)
		int track, speed, steer, accel, visits;
		float qFuncVal;

		qFuncFile >> track >> speed >> steer >> accel >> qFuncVal >> visits;
		pair<int,int> state = {track, speed};
		pair<int,int> action = {steer, accel};

		if(Q.find(state) == Q.end())
			Q[state] = vector<pair<float, int> >(qLearnConfig.getActionCount(), {0,0});
		Q[state][qLearnConfig.getActionId(action)] = {qFuncVal,visits};
	}
	qFuncFile.close();
}

void saveQFunction()
{
	qFuncFile.open(qFuncFilename, ios::out | ios::trunc);

	for(auto& stateActions : Q)
	{
		pair<int,int> state = stateActions.first;
		for(unsigned int i = 0; i< stateActions.second.size(); i++)
		{
			stringstream buffer;
			pair<int,int> action = qLearnConfig.getIdAction(i);
			buffer << state.first << " " << state.second << " " << action.first << " " << action.second << " " << stateActions.second[i].first << " " << stateActions.second[i].second << "\n";
			qFuncFile << buffer.str();
		
		}
	}

	qFuncFile.close();
}

float getDeltaDistance(CarState& before, CarState& after)
{
	return after.getDistRaced() - before.getDistRaced();
}

float distance(const vector<float>& A, const vector<float>& B)
{
	float dist = 0;
	for(unsigned int i=0;i<A.size();i++)
		dist += (A[i]-B[i])*(A[i]-B[i]);
	return sqrt(dist);
}

pair<int,int> getState(CarState& cs)
{
	vector<float> track;
	for(int i=0;i<TRACK_SENSORS;i++)
		track.push_back(cs.getTrack(i));

	int closest = 0;
	float closestDist = INF;
	for(unsigned int i = 0; i < qLearnConfig.tracks.size(); i++)
	{
		float dist = distance(track, qLearnConfig.tracks[i]);
		if(dist < closestDist)
		{
			closestDist = dist;
			closest = i;
		}
	}

	float speedX, speedY, speedZ;
	speedX = cs.getSpeedX();
	speedY = cs.getSpeedY();
	speedZ = cs.getSpeedZ();

	float speed = sqrt(speedX*speedX + speedY*speedY + speedZ*speedZ);

	unsigned int speedZone = 0;
	for(speedZone = 0; speedZone < qLearnConfig.speeds.size() && speed >= qLearnConfig.speeds[speedZone];speedZone++);
	speedZone = min((int)(qLearnConfig.speeds.size() - 1), (int)speedZone);

	return {closest, speedZone};
}

int getBestActionId(const pair<int,int>& state)
{
	if(Q.find(state) == Q.end())
		Q[state] = vector<pair<float,int> >(qLearnConfig.getActionCount(), {0,0});

	float bestUtility = numeric_limits<float>::lowest();
	int bestUtilVisits = 0;
	int action = 0;

	for(unsigned int i=0; i<Q[state].size();i++)
	{
		if(Q[state][i].first > bestUtility || (Q[state][i].first == bestUtility && Q[state][i].second < bestUtilVisits))
		{
			bestUtility = Q[state][i].first;
			bestUtilVisits = Q[state][i].second;
			action = i;
		}
	}

	return action;
}

pair<int,int> takeAction(const pair<int,int>& state)
{
	if(qLearnConfig.randomStep() || Q.find(state) == Q.end())
	{
		int random = makeRandom(0, qLearnConfig.getActionCount() - 1);
		return qLearnConfig.getIdAction(random); 
	}

	return qLearnConfig.getIdAction(getBestActionId(state));
}


void updateReward(const pair<int,int>& lastState, const pair<int,int>& lastAction, CarState& lastCarState, const pair<int,int>& currentState, CarState& currentCarState)
{
	if(lastState == NONE)
		return;

	float reward = 0;
	if(qLearnConfig.isTerminal(currentState))
		reward = qLearnConfig.offTrackReward;
	else
	{
		reward = getDeltaDistance(lastCarState, currentCarState);
		if(reward < qLearnConfig.notMovingThreshold)
			reward = qLearnConfig.notMovingReward;
	}

	if(Q.find(lastState) == Q.end())
		Q[lastState] = vector<pair<float,int> >(qLearnConfig.getActionCount(), {0,0});

	int lastActionId = qLearnConfig.getActionId(lastAction);
	float alpha = qLearnConfig.alpha;
	//float alpha = 1 / (1 + Q[lastState][lastActionId].second);
	float gamma = qLearnConfig.gamma;
	float bestActionId = getBestActionId(currentState);

	Q[lastState][lastActionId].first += alpha * (reward + gamma * Q[currentState][bestActionId].first - Q[lastState][lastActionId].first);
	Q[lastState][lastActionId].second ++;
}

int
QLearningDriver::getGear(CarState &cs)
{

    int gear = cs.getGear();
    int rpm  = cs.getRpm();

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear<1)
        return 1;
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear <6 && rpm >= gearUp[gear-1])
        return gear + 1;
    else
    	// check if the RPM value of car is lower than the one suggested 
    	// to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear-1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}

float
QLearningDriver::getSteer(CarState &cs)
{
	// steering angle is compute by correcting the actual car angle w.r.t. to track 
	// axis [cs.getAngle()] and to adjust car position w.r.t to middle of track [cs.getTrackPos()*0.5]
    float targetAngle=(cs.getAngle()-cs.getTrackPos()*0.5);
    // at high speed reduce the steering command to avoid loosing the control
    if (cs.getSpeedX() > steerSensitivityOffset)
        return targetAngle/(steerLock*(cs.getSpeedX()-steerSensitivityOffset)*wheelSensitivityCoeff);
    else
        return (targetAngle)/steerLock;

}
float
QLearningDriver::getAccel(CarState &cs)
{
    // checks if car is out of track
    if (cs.getTrackPos() < 1 && cs.getTrackPos() > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor=cs.getTrack(10);
        // reading of sensor parallel to car axis
        float cSensor=cs.getTrack(9);
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor=cs.getTrack(8);

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor>maxSpeedDist || (cSensor>=rxSensor && cSensor >= sxSensor))
            targetSpeed = maxSpeed;
        else
        {
            // approaching a turn on right
            if(rxSensor>sxSensor)
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = rxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                // computing approximately the "angle" of turn
                float h = cSensor*sin5;
                float b = sxSensor - cSensor*cos5;
                float sinAngle = b*b/(h*h+b*b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed*(cSensor*sinAngle/maxSpeedDist);
            }

        }

        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2/(1+exp(cs.getSpeedX() - targetSpeed)) - 1;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}

CarControl
QLearningDriver::wDrive(CarState cs)
{
	//iterationCount++;
	if(iterationCount++ % 10 == 0)
	{
		auto state = getState(cs);
		updateReward(lastState, lastAction, lastCarState, state, cs);

		if(qLearnConfig.isTerminal(state))
		{
			int accel = 0;
			int brake = 0;
			int gear = 0;
			int steer = 0;
			int focus = 0;
			int meta = 1;
			return CarControl(accel,brake,gear,steer,clutch,focus,meta);
		}

		auto action = takeAction(state);

		lastCarState = cs;
		lastState = state;
		lastAction = action;
		
		float steer = qLearnConfig.steers[action.first];
		float accel = qLearnConfig.accelBrakes[action.second].first;
		float brake = qLearnConfig.accelBrakes[action.second].second;

		// compute gear 
		int gear = getGear(cs);

		// Calculate clutching
		clutching(cs,clutch);

		// focus
		int focus = 0;
		int meta = 0;

		// build a CarControl variable and return it
		lastControl = CarControl (accel,brake,gear,steer,clutch,focus,meta);
	}
	return lastControl;
}

float
QLearningDriver::filterABS(CarState &cs,float brake)
{
	// convert speed to m/s
	float speed = cs.getSpeedX() / 3.6;
	// when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;
    
    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip/4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip)/absRange;
    }
    
    // check brake is not negative, otherwise set it to zero
    if (brake<0)
    	return 0;
    else
    	return brake;
}

void
QLearningDriver::onShutdown()
{
	cout << "Saving Q function values to file " << qFuncFilename << "...\n";
	saveQFunction();
	cout << "Iterations: " << iterationCount << "\n";
	cout << "Distance raced: " << lastCarState.getDistRaced() << "\n";
    cout << "Bye bye!" << endl;
}

void
QLearningDriver::onRestart()
{
	if(stopwatch.stop() >= saveFrequencyInMin)
	{
		saveQFunction();
		stopwatch = Stopwatch<>();
		cout << "Saved Q Function values to file " << qFuncFilename << "\n";
	}
	cout << "Iterations: " << iterationCount << "\n";
	cout << "Distance raced: " << lastCarState.getDistRaced() << "\n";
    cout << "Restarting the race!" << endl;
}

void
QLearningDriver::clutching(CarState &cs, float &clutch)
{
  double maxClutch = clutchMax;

  // Check if the current situation is the race start
  if (cs.getCurLapTime()<clutchDeltaTime  && stage==RACE && cs.getDistRaced()<clutchDeltaRaced)
    clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0)
  {
    double delta = clutchDelta;
    if (cs.getGear() < 2)
	{
      // Apply a stronger clutch output when the gear is one and the race is just started
	  delta /= 2;
      maxClutch *= clutchMaxModifier;
      if (cs.getCurLapTime() < clutchMaxTime)
        clutch = maxClutch;
	}

    // check clutch is not bigger than maximum values
	clutch = min(maxClutch,double(clutch));

	// if clutch is not at max value decrease it quite quickly
	if (clutch!=maxClutch)
	{
	  clutch -= delta;
	  clutch = max(0.0,double(clutch));
	}
	// if clutch is at max value decrease it very slowly
	else
		clutch -= clutchDec;
  }
}

void
QLearningDriver::init(float *angles)
{
	cout<< "Init function\n";

	// set angles as {-90,-75,-60,-45,-30,20,15,10,5,0,5,10,15,20,30,45,60,75,90}

	for (int i=0; i<5; i++)
	{
		angles[i]=-90+i*15;
		angles[18-i]=90-i*15;
	}

	for (int i=5; i<9; i++)
	{
			angles[i]=-20+(i-5)*5;
			angles[18-i]=20-(i-5)*5;
	}
	angles[9]=0;

	iterationCount = 0;

	if(!qLearnConfig.initialized())
		qLearnConfig = fromFile(qLearningConfigFilename);

	readQFunction();


	lastState = NONE;
	lastAction = NONE;
	lastCarState = CarState();	
}
