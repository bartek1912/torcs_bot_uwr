/***************************************************************************
 
    file                 : BaseDriver.h
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
#ifndef BASEDRIVER_H_
#define BASEDRIVER_H_

#include<iostream>

using namespace std;

class BaseDriver
{
public:
	
	typedef enum {Left90 = 0, Left60, Left30, Left15, Left7, Forward, Right7, 
			Right15, Right30, Right60, Right90} Action;
	typedef enum{WARMUP,QUALIFYING,RACE,UNKNOWN} tstage;

	tstage stage;
	char trackName[100];

	// Default Constructor
	BaseDriver(){};
	
	// Default Destructor;
	virtual ~BaseDriver(){};
	
	// Initialization of the desired angles for the rangefinders
	virtual void init(float *angles){
  		cerr<<"BaseDriver::init\n";
		for (int i = 0; i < 19; ++i)
			angles[i]=-90+i*10;
		int rad = 6;
		for(int i = 19/2 - rad; i <= 19/2 + rad; i++)
			angles[i] = (i - 19/2)*5;
		init2(angles);
	};
	virtual void init2(float *angles){}

	// The main function: 
	//     - the input variable sensors represents the current world sate
	//     - it returns a string representing the controlling action to perform    
	virtual string drive(string sensors)=0;

	// Callback function called at shutdown
	virtual void onShutdown(){};
	
	// Callback function called at server restart
	virtual void onRestart(){};
};
#endif /*BASEDRIVER_H_*/
