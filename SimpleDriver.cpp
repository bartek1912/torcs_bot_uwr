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
const double mnoznik_kier = 8.5711/2;



CarControl SimpleDriver::wDrive(CarState cs)
{
  double dir;
  int id_mx = TRACK_SENSORS_NUM/2;
  for(int i = 0; i < TRACK_SENSORS_NUM; i++)
    if(cs.getTrack(i) > cs.getTrack(id_mx))
        id_mx = i;
  dir = -(id_mx - TRACK_SENSORS_NUM/2)/static_cast<double>(TRACK_SENSORS_NUM) * mnoznik_kier;
  double acc = 1, br = 0;
  //100 - 30
  //200 - 60
  //300 - 270
  if((cs.getTrack(id_mx) < 150 && cs.getTotalSpeed() > 260)
    || (cs.getTrack(id_mx) < 100 && cs.getTotalSpeed() > 220)
    || (cs.getTrack(id_mx) < 60 && cs.getTotalSpeed() > 150)
    || (cs.getTrack(id_mx) < 50 && cs.getTotalSpeed() > 120)
    || (cs.getTrack(id_mx) < 40 && cs.getTotalSpeed() > 100)
    || (cs.getTrack(id_mx) < 30 && cs.getTotalSpeed() > 85)
    || (cs.getTrack(id_mx) < 20 && cs.getTotalSpeed() > 70))
  {
    br = 1;
    acc = 0;
  }
  filterABS(cs, br);
  return CarControl(acc, br, cs.getGear(), dir, 0.00);
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



/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3179,0.3179,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;