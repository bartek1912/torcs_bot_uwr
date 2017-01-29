/***************************************************************************
 
    file                 : SimpleDriver.cpp
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
#include "SimpleDriver.h"
#include "forward_model/ForwardModel.h"
#define VERBOSE

const double mnoznik_kier = 8.5711/2;

ForwardModel fModel;

CarControl SimpleDriver::wDrive(CarState cs)
{
  static CarState before = cs;
	if(cs.getCurLapTime() > 0)
	{
    #ifdef VERBOSE
    cout << "Model before: Speed\t " << before.getSpeedX()<<"\t\t\t\n";
		cout << "Model after simulation:\n\tPosition = " << fModel.getCarState().pos.lin << "\n";
		cout << "\tSpeed " << fModel.getCarState().vel.lin << "\n";
    #endif
		fModel.updateModel(cs);	
	}
  #ifdef VERBOSE
  cout << "Model after update:\n";
  cout << "\tSpeed " << fModel.getCarState().vel.lin << "\n";;
  if(cs.getCurLapTime() > 0)
    cout << "\e[A\e[A\e[A\e[A\e[A\e[A\r";
  else
    cout << "\e[A\e[A\r";
  #endif
	
  calc.update_pos(cs);
  double dir;
  int id_mx = TRACK_SENSORS_NUM/2;
  for(int i = 0; i < TRACK_SENSORS_NUM; i++)
    if(cs.getTrack(i) > cs.getTrack(id_mx))
        id_mx = i;
  dir = -(id_mx - TRACK_SENSORS_NUM/2)/static_cast<double>(TRACK_SENSORS_NUM) * mnoznik_kier;
  double acc = (cs.getSpeedX() > 50 ? 1/*0.5*/: 1), br = 0;
  //100 - 30
  //200 - 60
  //300 - 270
  if(cs.getLastLapTime() <= 0)
  {
    track.add_new_points(calc.get_point_moved(cs.getTrack(TRACK_SENSORS_NUM/2 - 2), 8),
                         calc.get_point_moved(cs.getTrack(TRACK_SENSORS_NUM/2 + 2), 8));
  }
  if((cs.getTrack(id_mx) < 150 && cs.getTotalSpeed() > 260)
    || (cs.getTrack(id_mx) < 100 && cs.getTotalSpeed() > 220)
    || (cs.getTrack(id_mx) < 60 && cs.getTotalSpeed() > 150)
    || (cs.getTrack(id_mx) < 50 && cs.getTotalSpeed() > 120)
    || (cs.getTrack(id_mx) < 40 && cs.getTotalSpeed() > 100)
    || (cs.getTrack(id_mx) < 30 && cs.getTotalSpeed() > 85)
    || (cs.getTrack(id_mx) < 20 && cs.getTotalSpeed() > 70)
    || cs.getSpeedX() > 270)
  {
    br = 0;
    acc = 0;
  }
  //track.car_on_the_track(calc.get_car_position());
  filterABS(cs, br);
  calc.update_angle(dir);
  if(dir < 0)
    dir = max(dir, -0.2);
  else
    dir = min(dir, 0.2);

	CarControl toReturn = CarControl(acc, br, cs.getGear(), dir, 0.00);

	if(cs.getCurLapTime() > 0)
		fModel.simulate(0.010, toReturn);
  before = cs;
  return toReturn;
} 


float SimpleDriver::filterABS(CarState &cs,float brake) const 
{
  float speed = cs.getSpeedX() / 3.6;
  
    if (speed < absMinSpeed)
        return brake;
    
    float slip = 0.0f;
    
    for (int i = 0; i < 4; i++){
        slip += cs.getWheelSpinVel(i) * wheelRadius[i];}
        
    slip = speed - slip/4.0f;
    
    if (slip > absSlip){
        brake = brake - (slip - absSlip)/absRange; }
    
    if (brake<0)
      return 0;
    else
      return brake;
}

void
SimpleDriver::onShutdown()
{
   // cout << "Bye bye!" << endl;
}

void
SimpleDriver::onRestart() 
{
}

void
SimpleDriver::init2(float* angles)
{
	cout << "Stage: ";
	switch(stage)
	{
		case tstage::WARMUP:
			cout << "WARMUP\n";		
			cout << "Building track model\n";
			break;
		case tstage::QUALIFYING:
			cout << "QUALIFYING\n";
			cout << "Drive as fast as possible without falling out of the track\n";
			break;
		case tstage::RACE:
			cout << "RACE\n";
			cout << "Win the race against\n";
			break;
		default:
			cout << "UNKNOWN\n";
			break;
	}
	
	//BUG TO SPRAWIA, ZE WHEEL, BRAKESYSTEM I STEER MAJA W SRODKU SMIECI
	fModel = ForwardModel();
}

/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3179,0.3179,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;
