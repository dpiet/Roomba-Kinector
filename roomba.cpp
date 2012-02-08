/**
 Roomba.cpp: Class provides interface to a Roomba vacuum object.
 OO-ifies the roombalib C-library available at hackingroomba.com
 Author: David Pietrocola
 Date: October 2, 2011
 */

#include <sstream>
#include <iostream>
#include <stdio.h>    /* Standard input/output definitions */
#include <stdint.h>   /* Standard types */
#include <stdlib.h>   /* calloc, strtol */
#include <string>
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include "roomba.h"

int roombadebug = 0;

using namespace std;

Roomba::Roomba(const string portname) : 
	portpath(portname), defaultVelocity(200),commandPause(100),debugVal(0) {
	serialPointer = initComm(portpath, B57600 );
    if( serialPointer == -1 ) exit(1);
    uint8_t cmd[1];
    
    cmd[0] = 128;      // START
    int n = write(serialPointer, cmd, 1);
    if( n!=1 ) {
        perror("open_port: Unable to write to port ");
        exit(1);
    }
    delay(commandPause);
    
    cmd[0] = 130;   // CONTROL
    n = write(serialPointer, cmd, 1);
    if( n!=1 ) {
        perror("open_port: Unable to write to port ");
        exit(1);
    }
    delay(commandPause);
}

Roomba::~Roomba()
{
	// Close serial comm
	if( serialPointer ) {
		close( serialPointer ); 
		serialPointer = 0; }
}

int Roomba::initComm(const string serialport, speed_t baud) {
	// Helper function to initialize serial port comm
	struct termios toptions;
    int fd;
	
    if(roombadebug)
        fprintf(stderr,"roomba_init_serialport: opening port %s\n",serialport);
	
    fd = open( serialport, O_RDWR | O_NOCTTY | O_NDELAY );
    if (fd == -1)  {     // Could not open the port.
        perror("roomba_init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("roomba_init_serialport: Couldn't get term attributes");
        return -1;
    }
    
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);
	
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
	
    toptions.c_cflag    |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag    &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	
    toptions.c_lflag    &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag    &= ~OPOST; // make raw
	
    toptions.c_cc[VMIN]  = 26;
    toptions.c_cc[VTIME] = 2;           // FIXME: not sure about this
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("roomba_init_serialport: Couldn't set term attributes");
        return -1;
    }
	
    return fd;
	
}

void Roomba :: drive(int velocity, int radius)
{
	// low-level drive command
	uint8_t vhi = velocity >> 8;
    uint8_t vlo = velocity & 0xff;
    uint8_t rhi = radius   >> 8;
    uint8_t rlo = radius   & 0xff;
    if(roombadebug) 
        fprintf(stderr,"roomba_drive: %.2hhx %.2hhx %.2hhx %.2hhx\n",
                vhi,vlo,rhi,rlo);
    uint8_t cmd[5] = { 137, vhi,vlo, rhi,rlo };  // DRIVE
    int n = write(serialPointer, cmd, 5);
    if( n!=5 )
        perror("roomba_drive: couldn't write to roomba");
}

void Roomba :: stop() {
	drive( 0, 0 );
}

void forward(int vel = defaultVelocity) {
	drive( vel, 0x8000 );
}

void backward(int vel = defaultVelocity) {
	drive( -vel, 0x8000 );
}
	
void delay( int millisecs )
{
    usleep( millisecs * 1000 );
}

