/***************************************************************************
 
    file                 : WrapperBaseDriver.h
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
#ifndef WRAPPERBASEDRIVER_H_
#define WRAPPERBASEDRIVER_H_

#include "CarState.h"
#include "CarControl.h"
#include "BaseDriver.h"
#include <cmath>
#include <cstdlib>
#include <vector>
#include <fstream>
#define DEBUG_NONE	 0x00000
#define DEBUG_ALL	 0xFFFFF
#define IN_DAMAGE	 0x00001
#define IN_GEAR		 0x00002
#define IN_FOCUS	 0x00004
#define IN_TRACK	 0x00008
#define IN_OPPONENTS 0x00010
#define IN_ALL_SENSORS (IN_FOCUS|IN_TRACK|IN_OPPONENTS)
#define IN_RPM		 0x00020
//wolne 6 bitów 0x000040-0x0000800
#define OUT_ACCEL   0x01000
#define	OUT_BRAKE	0x02000
#define OUT_GEAR	0x04000
#define OUT_STEER	0x08000
#define OUT_CLUTCH	0x10000

class WrapperBaseDriver : public BaseDriver
{
	const int max_gear;
	const unsigned debug_mode;
	void print_debug_in(const CarState& cs) const;
	void print_debug_out(const CarControl& cc) const;
	void validate_gear(CarControl& cc, const CarState& cs) const;
public:
	WrapperBaseDriver(int mx_gear = 6, unsigned debug = DEBUG_ALL) :max_gear(mx_gear), debug_mode(debug){}
	
	// the drive function wiht string input and output
	virtual string drive(string sensors);
	
	// drive function that exploits the CarState and CarControl wrappers as input and output.
	virtual CarControl wDrive(CarState cs)=0;
	// Initialization of the desired angles for the rangefinders
	virtual void init2(float *angles);
private:
	void clutching(const CarState &cs, double &clutch) const;
	mutable double last_clutch;
	static const float clutchMax;
	static const float clutchDelta;
	static const float clutchRange;
	static const float clutchDeltaTime;
	static const float clutchDeltaRaced;
	static const float clutchDec;
	static const float clutchMaxModifier;
	static const float clutchMaxTime;
};

#endif /*WRAPPERBASEDRIVER_H_*/
