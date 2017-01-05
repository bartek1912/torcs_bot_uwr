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
#include "CarState.h"
#include <cmath>

CarState::CarState(string sensors)
{
        SimpleParser::parse(sensors, "angle", this->angle);
        SimpleParser::parse(sensors, "curLapTime", this->curLapTime);
        SimpleParser::parse(sensors, "damage", this->damage);
        SimpleParser::parse(sensors, "distFromStart", this->distFromStart);
        SimpleParser::parse(sensors, "distRaced", this->distRaced);
        SimpleParser::parse(sensors, "focus", this->focus, FOCUS_SENSORS_NUM);
        SimpleParser::parse(sensors, "fuel", this->fuel);
        SimpleParser::parse(sensors, "gear", this->gear);
        SimpleParser::parse(sensors, "lastLapTime", this->lastLapTime);
        SimpleParser::parse(sensors, "opponents", this->opponents, OPPONENTS_SENSORS_NUM);
        SimpleParser::parse(sensors, "racePos", this->racePos);
        SimpleParser::parse(sensors, "rpm", this->rpm);
        SimpleParser::parse(sensors, "speedX", this->speedX);
        SimpleParser::parse(sensors, "speedY", this->speedY);
        SimpleParser::parse(sensors, "speedZ", this->speedZ);
        if(!SimpleParser::parse(sensors, "track", this->track, TRACK_SENSORS_NUM))
            cerr<<"Nie udało się odczytać track sensorów z "<<sensors<<"\n";
        SimpleParser::parse(sensors, "trackPos", this->trackPos);
        SimpleParser::parse(sensors, "wheelSpinVel", this->wheelSpinVel, 4);
        SimpleParser::parse(sensors, "z", this->z);
}

string
CarState::toString()
{
	string str;
	str  = SimpleParser::stringify("angle", this->angle);
	str += SimpleParser::stringify("curLapTime", this->curLapTime);
	str += SimpleParser::stringify("damage", this->damage);
	str += SimpleParser::stringify("distFromStart", this->distFromStart);
	str += SimpleParser::stringify("distRaced", this->distRaced);
	str += SimpleParser::stringify("focus", this->focus, FOCUS_SENSORS_NUM);
	str += SimpleParser::stringify("fuel", this->fuel);
	str += SimpleParser::stringify("gear", this->gear);
	str += SimpleParser::stringify("lastLapTime", this->lastLapTime);
	str += SimpleParser::stringify("opponents", this->opponents, OPPONENTS_SENSORS_NUM);
	str += SimpleParser::stringify("racePos", this->racePos);
	str += SimpleParser::stringify("rpm", this->rpm);
	str += SimpleParser::stringify("speedX", this->speedX);
	str += SimpleParser::stringify("speedY", this->speedY);
	str += SimpleParser::stringify("speedZ", this->speedZ);
	str += SimpleParser::stringify("track", this->track, TRACK_SENSORS_NUM);
	str += SimpleParser::stringify("trackPos", this->trackPos);
	str += SimpleParser::stringify("wheelSpinVel", this->wheelSpinVel, 4);
	str += SimpleParser::stringify("z", this->z);
	
	return str;
	        
}

float 
CarState::getAngle() const
{
        return angle;
};

void 
CarState::setAngle(float angle)
{
        this->angle = angle;
};

float 
CarState::getCurLapTime() const
{
        return curLapTime;
};

void 
CarState::setCurLapTime(float curLapTime)
{
        this->curLapTime = curLapTime;
};

float
CarState::getDamage() const
{
        return damage;
};

void
CarState::setDamage(float damage)
{
        this->damage = damage;
};

float
CarState::getDistFromStart() const
{
        return distFromStart;
};

void
CarState::setDistFromStart(float distFromStart)
{
        this->distFromStart = distFromStart;
};

float
CarState::getDistRaced() const
{
        return distRaced;
};

void
CarState::setDistRaced(float distRaced)
{
        this->distRaced = distRaced;
};

float 
CarState::getTotalSpeed()
{
    return std::sqrt(pow(getSpeedX(),2) + pow(getSpeedY(),2) + pow(getSpeedZ(),2));
}
float
CarState::getFocus(int i) const
{
        assert(i>=0 && i<FOCUS_SENSORS_NUM);
        return focus[i];
};


void
CarState::setFocus(int i, float value)
{
        assert(i>=0 && i<FOCUS_SENSORS_NUM);
        std::cerr<<" focus["<<i<<"] = "<<value<<"; ";
        this->focus[i] = value;
};

float
CarState::getFuel() const
{
        return fuel;
};

void
CarState::setFuel(float fuel)
{
        this->fuel = fuel;
};

int
CarState::getGear() const
{
        return gear;
};

void
CarState::setGear(int gear)
{
        this->gear = gear;
};

float 
CarState::getLastLapTime() const
{
        return lastLapTime;
};

void 
CarState::setLastLapTime(float lastLapTime)
{
        this->lastLapTime = lastLapTime;
};

float
CarState::getOpponents(int i) const
{
        assert(i>=0 && i<OPPONENTS_SENSORS_NUM);
        return opponents[i];
        
};

void
CarState::setOpponents(int i, float value)
{
        assert(i>=0 && i<OPPONENTS_SENSORS_NUM);
        this->opponents[i] = value;
};

int
CarState::getRacePos() const
{
        return racePos;
};

void
CarState::setRacePos(int racePos)
{
        this->racePos = racePos;
};

int
CarState::getRpm() const
{
        return rpm;
};

void
CarState::setRpm(int rpm)
{
        this->rpm = rpm;
};

float
CarState::getSpeedX() const
{
        return speedX;
};

void
CarState::setSpeedX(float speedX)
{
        this->speedX = speedX;
};

float
CarState::getSpeedY() const
{
        return speedY;
};

void
CarState::setSpeedY(float speedY)
{
        this->speedY = speedY;
};

float
CarState::getSpeedZ() const
{
        return speedZ;
};


void
CarState::setSpeedZ(float speedZ)
{
        this->speedZ = speedZ;
};

float
CarState::getTrack(int i) const
{
        assert(i>=0 && i<TRACK_SENSORS_NUM);
        return track[i];
};


void
CarState::setTrack(int i, float value)
{
        assert(i>=0 && i<TRACK_SENSORS_NUM);
        this->track[i] = value;
};

float
CarState::getTrackPos() const
{
        return trackPos;
};

void
CarState::setTrackPos(float prackPos)
{
        this->trackPos = trackPos;
};

float 
CarState::getWheelSpinVel(int i) const
{
	assert(i>=0 && i<4);
	return wheelSpinVel[i];
}

void 
CarState::setWheelSpinVel(int i, float value)
{
	assert(i>=0 && i<4);
	wheelSpinVel[i]=value;
}

float
CarState::getZ() const
{
        return z;
};

void
CarState::setZ(float z)
{
    this->z = z;
};
