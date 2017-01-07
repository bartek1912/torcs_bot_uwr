
#ifndef CARCALCULATOR_H_
#define CARCALCULATOR_H_
#include "CarState.h"
#include <cmath>
#define MAX_ANG_DELTA 0.366519
#define TIME_TICK     0.01
#define KPTOMS		  1/3.6
using namespace std;
#define PI 3.14159265
struct CarCalculator
{
	CarCalculator(pair<long double, long double> position = make_pair(0.0,0.0)):
		pos(position),
		angle(0),
		last_angle(0),
		last_delta(0)
	{}
	void update_pos(const CarState& cs)
	{
		static long double last_speed = 0;
		pos.first += sinl(angle) * (cs.getSpeedX()) * TIME_TICK * KPTOMS;
		pos.second += cosl(angle) * (cs.getSpeedX()) * TIME_TICK * KPTOMS;
		last_speed = cs.getSpeedX();
	}
	void update_angle(long double delta)
	{
		angle += delta * TIME_TICK * MAX_ANG_DELTA * 20;//magic 20 - without doesnt work
		if(angle > PI)
			angle -= 2*PI;
		if(angle < -PI)
			angle += 2*PI;
		last_delta = delta;
	}
	long double degToRad(long double deg) const
	{
		return deg/180*PI;
	}
	pair<long double, long double> get_point_moved(long double length, long double angle_delta_rad) const
	{
		return pos;
	}
	pair<long double, long double> get_car_position() const
	{
		return pos;
	}
private:	
	pair<long double, long double> pos;
	long double angle, last_angle, last_delta;
};
#endif /*CARCALCULATOR_H_*/