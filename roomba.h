/*
 *  roomba.h
 *  
 *
 *  Created by David Pietrocola on 9/18/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef ROOMBA_H
#define ROOMBA_H

#include <iostream>
#include <string>

using namespace std;

class Roomba {
private:
	int defaultVelocity, commandPause, debugVal,serialPointer;
	string portpath;
	uint8_t sensor_bytes[26];
	void drive(int velocity, int radius);
	void delay( int millisecs);
	int initComm(const string serialport, speed_t baud);
	
public:
	Roomba(const string portname);
	~Roomba();
	void stop();
	void forward(int vel);
	void backward(int vel);
	
	
	
};
#endif