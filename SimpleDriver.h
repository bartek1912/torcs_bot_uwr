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

private:
	float filterABS(CarState &cs,float brake) const;

	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	static const float wheelRadius[4];
	// min slip to prevent ABS
	static const float absSlip;						
	// range to normalize the ABS effect on the brake
	static const float absRange;
	// min speed to activate ABS
	static const float absMinSpeed;
	Track_model track;
	CarCalculator calc;
};

#endif /*SIMPLEDRIVER_H_*/
