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
#include "TrackModel.h"

#define VERBOSE

const double mnoznik_kier = 8.5711/2;

ForwardModel fModel;
TrackModel tModel;

CarControl SimpleDriver::wDrive(CarState cs)
{
	if(stage == WARMUP)
	{
		if(tModel.doneBuilding())
		{
			stage = QUALIFYING;
		}
		else
		{
			tModel.continueBuilding(cs);	
		}

		CarControl cc = simpleDriving(cs);
		return cc;
	}
	else
	{
	  #ifdef VERBOSE
		cout << "TrackPos " << cs.getTrackPos() << "\n";
		cout << "Left + Right = " << cs.getTrack(0) + cs.getTrack(18) << "\n";
		cout << "DistFromStart = " << cs.getDistFromStart() << "\n";
		if(cs.getCurLapTime() > 0)
		{
			cout << "Model after simulation:\n";
			cout << "\tPosition = " << fModel.getCarState().pos.lin << "\n";
			cout << "\tAngles = " << fModel.getCarState().pos.ang << "\n";
			cout << "\tSpeed " << fModel.getCarState().vel.lin << "\n";
			cout << "\tAngular Speed " << fModel.getCarState().vel.ang << "\n";
			cout << "\tAcceleration " << fModel.getCarState().acc.lin << "\n";
			cout << "\tAngular Acceleration " << fModel.getCarState().acc.ang << "\n";
			for(int i=0;i<4;i++)
			{
				cout << "\tWheel " << i << " force " << fModel.getCarState().wheels[i].force << "\n";
				cout << "\tWheel " << i << " spinVel " << fModel.getCarState().wheels[i].spinVel << "\n";
			}
			fModel.updateModel(cs);	
		}
	  cout << "Model after update:\n";
	  cout << "\tSpeed " << fModel.getCarState().vel.lin << "\n";;
		for(int i=0;i<4;i++)
		{
			cout << "\tWheel " << i << " spinVel " << fModel.getCarState().wheels[i].spinVel << "\n";
		}
	  
	  /*
	  if(cs.getCurLapTime() > 0)
	  {
		cout << "\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\e[A\r";
	  }
	  else
		cout << "\e[A\e[A\r";
		*/
		
	  #endif
		
	  calc.update_pos(cs);
	  double dir;
	  int id_mx = TRACK_SENSORS_NUM/2;
	  for(int i = 0; i < TRACK_SENSORS_NUM; i++)
		if(cs.getTrack(i) > cs.getTrack(id_mx))
			id_mx = i;
	  dir = -(id_mx - TRACK_SENSORS_NUM/2)/static_cast<double>(TRACK_SENSORS_NUM) * mnoznik_kier;
	  double acc = (cs.getSpeedX() > 50 ? 0.5: 1), br = 0;
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
		|| cs.getSpeedX() > 70)
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
			fModel.simulate(0.020, toReturn);

	  return toReturn;
	}
} 


float SimpleDriver::filterABS(CarState &cs,float brake) 
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

void generateTrackWDrive(CarState& cs)
{

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
			tModel.initialize();
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

/* Gear Changing Constants*/
const int SimpleDriver::gearUp[6]=
    {
        5000,6000,6000,6500,7000,0
    };
const int SimpleDriver::gearDown[6]=
    {
        0,2500,3000,3000,3500,3500
    };

/* Stuck constants*/
const int SimpleDriver::stuckTime = 25;
const float SimpleDriver::stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float SimpleDriver::maxSpeedDist=1;
const float SimpleDriver::maxSpeed=10;
const float SimpleDriver::sin5 = 0.08716;
const float SimpleDriver::cos5 = 0.99619;

/* Steering constants*/
const float SimpleDriver::steerLock=0.366519 ;
const float SimpleDriver::steerSensitivityOffset=80.0;
const float SimpleDriver::wheelSensitivityCoeff=1;

/* ABS Filter Constants */
const float SimpleDriver::wheelRadius[4]={0.3306,0.3306,0.3276,0.3276};
const float SimpleDriver::absSlip=2.0;
const float SimpleDriver::absRange=3.0;
const float SimpleDriver::absMinSpeed=3.0;

/* Clutch constants */
const float SimpleDriver::clutchMax=0.5;
const float SimpleDriver::clutchDelta=0.05;
const float SimpleDriver::clutchRange=0.82;
const float SimpleDriver::clutchDeltaTime=0.02;
const float SimpleDriver::clutchDeltaRaced=10;
const float SimpleDriver::clutchDec=0.01;
const float SimpleDriver::clutchMaxModifier=1.3;
const float SimpleDriver::clutchMaxTime=1.5;

void
SimpleDriver::clutching(CarState &cs, float &clutch)
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

int
SimpleDriver::getGear(CarState &cs)
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
SimpleDriver::getSteer(CarState &cs)
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
SimpleDriver::getAccel(CarState &cs)
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
SimpleDriver::simpleDriving(CarState& cs)
{
	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > stuckAngle )
    {
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }

    else // car is not stuck
    {
    	// compute accel/brake command
        float accel_and_brake = getAccel(cs);
        // compute gear 
        int gear = getGear(cs);
        // compute steering
        float steer = getSteer(cs);
        

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        
        // set accel and brake from the joint accel/brake command 
        float accel,brake;
        if (accel_and_brake>0)
        {
            accel = accel_and_brake;
            brake = 0;
        }
        else
        {
            accel = 0;
            // apply ABS to brake
            brake = filterABS(cs,-accel_and_brake);
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc(accel,brake,gear,steer,clutch);
        return cc;
    }
}
