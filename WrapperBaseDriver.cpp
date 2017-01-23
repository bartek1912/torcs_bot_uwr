/***************************************************************************
 
    file                 : WrapperBaseDriver.cpp
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
#include "WrapperBaseDriver.h"
#include "forward_model/ForwardModel.h"
string 
WrapperBaseDriver::drive(string sensors)
{
	CarState cs(sensors);
	print_debug_in(cs);
	CarControl cc = wDrive(cs);
	validate_gear(cc, cs);
	print_debug_out(cc);
	linalg::vector zero = {0,0};
	ForwardModel fm(cs);
	return cc.toString();	
}
void WrapperBaseDriver::validate_gear(CarControl& cc, const CarState& cs) const
{ 
	static int t = 0;
	if(cc.getGear() == -1 || cs.getGear() == -1)
		return ;
 	int gear = cs.getGear();
 	double clutch = 0.00;
 	t+= cs.getRpm() > 9000 && cs.getCurLapTime() > 0;
 	if(((gear == 1  && last_clutch != 0) || gear == 0 ) && t < 20)
 	{
 		clutch = clutchMax;
 		clutching(cs, clutch);
 		gear = 1;
 	}
 	else if(cs.getRpm() > 9000 && gear < max_gear) 
   		gear++;
   	else if(cs.getRpm() < 6000 - (gear == 2)*3000 - (gear == 3)*2000 && gear > 1)
    	gear--;
    cc.setGear(gear);
    cc.setClutch(clutch);
    if(cs.getCurLapTime() < 0)
    	last_clutch = 1;
    else
    	last_clutch = clutch;
}

void 
WrapperBaseDriver::print_debug_in(const CarState& cs) const
{
	if(debug_mode & IN_DAMAGE)
		cerr<<"DMG: "<<cs.getDamage()<<"\n";
	if(debug_mode & IN_GEAR)
		cerr<<"GEAR: "<<cs.getGear()<<"\n";
	if(debug_mode & IN_RPM)
		cerr<<"RPM: "<<cs.getRpm()<<"\n";
	if(debug_mode & IN_FOCUS)
	{
		cerr<<"Focus sensor: ";
		for(int i = 0; i < FOCUS_SENSORS_NUM; i++)
			cerr<<cs.getFocus(i)<<" ";
		cerr<<"\n";
	}
	if(debug_mode & IN_TRACK)
	{
		cerr<<"Track sensor ("<<TRACK_SENSORS_NUM<<"):";
		for(int i = 0; i < TRACK_SENSORS_NUM; i+= TRACK_SENSORS_NUM/4)
			cerr<<cs.getTrack(i)<<" ";
		cerr<<"\n";
	}
	if(debug_mode & IN_OPPONENTS)
	{
		cerr<<"Opponents sensor: ";
		for(int i = 0; i < OPPONENTS_SENSORS_NUM; i++)
			cerr<<cs.getOpponents(i)<<" ";
		cerr<<"\n";
	}
}

// Initialization of the desired angles for the rangefinders
void WrapperBaseDriver::init2(float *angles)
{
}

void 
WrapperBaseDriver::print_debug_out(const CarControl& cc) const
{
	if(debug_mode & OUT_ACCEL)
		cerr<<"ACC: "<<cc.getAccel()<<"\n";
	if(debug_mode & OUT_BRAKE)
		cerr<<"BREAK: "<<cc.getBrake()<<"\n";
	if(debug_mode & OUT_STEER)
		cerr<<"Steer: "<<cc.getSteer()<<"\n";
	if(debug_mode & OUT_CLUTCH)
		cerr<<"Clutch:  "<<cc.getClutch()<<" (0 - 100% trakcji, 1 - brak trakcji)\n";
}


void WrapperBaseDriver::clutching(const CarState &cs, double &clutch) const {

  double maxClutch = clutchMax;

  // Check if the current situation is the race start
  if (cs.getDistRaced() < clutchDeltaRaced)
  	clutch = maxClutch;

  // Adjust the current value of the clutch
  if(clutch > 0) {
    double delta = clutchDelta;
    if (cs.getGear() < 2) {
    // Apply a stronger clutch output when the gear is one and the race is just started
      delta /= 2;
      maxClutch *= clutchMaxModifier;
       if (cs.getCurLapTime() < clutchMaxTime)
      	 clutch = maxClutch;
    }
    // check clutch is not bigger than maximum values
    clutch = min(maxClutch,double(clutch));

    // if clutch is not at max value decrease it quite quickly
    if (clutch != maxClutch) {
      clutch -= delta;
      clutch = max(0.0,double(clutch));
    }
    // if clutch is at max value decrease it very slowly
    else
      clutch -= clutchDec;
  }
}
const float WrapperBaseDriver::clutchMax=0.5;
const float WrapperBaseDriver::clutchDelta=0.05;
const float WrapperBaseDriver::clutchRange=0.82;
const float WrapperBaseDriver::clutchDeltaTime=0.02;
const float WrapperBaseDriver::clutchDeltaRaced=10;
const float WrapperBaseDriver::clutchDec=0.04;
const float WrapperBaseDriver::clutchMaxModifier=1.3;
const float WrapperBaseDriver::clutchMaxTime=1.5;