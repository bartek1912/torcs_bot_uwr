/***************************************************************************
 
    file                 : SimpleDriver.h
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifndef SIMPLEDRIVER_H_
#define SIMPLEDRIVER_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include "BaseDriver.h"
#include "CarState.h"
#include "CarControl.h"
#include "SimpleParser.h"
#include "WrapperBaseDriver.h"
#include "track_model.h"
#include "car_calculator.h"
#include <bits/stdc++.h>


using namespace std;
class SimpleDriver : public WrapperBaseDriver
{
public:
	
	// Constructor
	SimpleDriver():
		WrapperBaseDriver(6, DEBUG_NONE)
	{};

	// SimpleDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// We need an init function
	virtual void init2(float* angles);

private:
	// counter of stuck steps
	int stuck;
	
	// current clutch
	float clutch;

	// Solves the gear changing subproblems
	int getGear(CarState &cs);

	// Solves the steering subproblems
	float getSteer(CarState &cs);
	
	// Solves the gear changing subproblems
	float getAccel(CarState &cs);
	
	// Apply an ABS filter to brake command
	float filterABS(CarState &cs,float brake);

	// Solves the clucthing subproblems
	void clutching(CarState &cs, float &clutch);

	CarControl simpleDriving(CarState &cs);

	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	static const float wheelRadius[4];
	// min slip to prevent ABS
	static const float absSlip;						
	// range to normalize the ABS effect on the brake
	static const float absRange;
	// min speed to activate ABS
	static const float absMinSpeed;

	static const int gearUp[6];
	static const int gearDown[6];

	/* Stuck static constants*/
	static const int stuckTime;
	static const float stuckAngle; //PI/6

	/* Accel and Brake Constants*/
	static const float maxSpeedDist;
	static const float maxSpeed;
	static const float sin5;
	static const float cos5;

	/* Steering static constants*/
	static const float steerLock;
	static const float steerSensitivityOffset;
	static const float wheelSensitivityCoeff;

	/* Clutch static constants */
	static const float clutchMax;
	static const float clutchDelta;
	static const float clutchRange;
	static const float clutchDeltaTime;
	static const float clutchDeltaRaced;
	static const float clutchDec;
	static const float clutchMaxModifier;
	static const float clutchMaxTime;

	Track_model track;
	CarCalculator calc;
};

#endif /*SIMPLEDRIVER_H_*/
