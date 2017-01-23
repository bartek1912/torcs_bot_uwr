/***************************************************************************
 
    file                 : CarState.h
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
#ifndef CARSTATE_H_
#define CARSTATE_H_

#define FOCUS_SENSORS_NUM 5
#define TRACK_SENSORS_NUM 19
#define OPPONENTS_SENSORS_NUM 36

#include <iostream>
#include <sstream>
#include <cstring>
#include <cassert>
#include "SimpleParser.h"

using namespace std;

class CarState
{

private:
        float angle;
        float curLapTime;
        float damage;
        float distFromStart;
        float distRaced;
        float focus[FOCUS_SENSORS_NUM];
        float fuel;
        int   gear;
        float lastLapTime;
        float opponents[OPPONENTS_SENSORS_NUM];
        int   racePos;
        int   rpm;
        float speedX;
        float speedY;
        float speedZ;
        float track[TRACK_SENSORS_NUM];
        float trackPos;
        float wheelSpinVel[4];
        float z;
        

public:
	
		CarState(){};

        CarState(string sensors);

        string toString();

        /* Getter and setter methods */
        
        float getAngle() const; 
        
        void setAngle(float angle);
        
        float getCurLapTime() const;
        
        void setCurLapTime(float curLapTime);
        
        float getDamage() const;
        
        void setDamage(float damage);
        
        float getDistFromStart() const;
        
        void setDistFromStart(float distFromStart);
        
        float getDistRaced() const;
        
        void setDistRaced(float distRaced);

        float getFocus(int i) const;

        void setFocus(int i, float value);

        float getFuel() const;
        
        void setFuel(float fuel);
        
        int getGear() const;
        
        void setGear(int gear);
        
        float getLastLapTime() const;
        
        void setLastLapTime(float lastLapTime);
        
        float getOpponents(int i) const;
        
        void setOpponents(int i, float value);
        
        int getRacePos() const;
        
        void setRacePos(int racePos);
        
        int getRpm() const;
        
        void setRpm(int rpm);
        
        float getSpeedX() const;
        
        void setSpeedX(float speedX);
        
        float getSpeedY() const;
        
        void setSpeedY(float speedY);
        
        float getSpeedZ() const;

        void setSpeedZ(float speedZ);

        float getTrack(int i) const;
        
        void setTrack(int i, float value);
        
        float getTrackPos() const;
        
        void setTrackPos(float trackPos);
        
        float getWheelSpinVel(int i) const;
        
        void setWheelSpinVel(int i, float value);

        float getZ() const;

        void setZ(float z);

        float getTotalSpeed();


};

#endif /*CARSTATE_H_*/
