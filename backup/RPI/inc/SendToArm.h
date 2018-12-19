
#ifndef __SENDTOARM_H__
#define __SENDTOARM_H__

#include <wiringPi.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h> 
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include "main.h"

typedef struct Rpi {
	int setValue;
} RPI;

extern RPI rpi;
extern bool autoPilotMode;
extern bool startFlying;
extern bool ArmState;
extern int ARM_Init(int _id);
	
extern void lBeep();
extern void sendToArm();
extern void ARM_close();
extern void setZeroProp();

#endif

