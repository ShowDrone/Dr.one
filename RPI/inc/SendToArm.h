
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
#define FLYING_BYTE 18
#define READY_TO_FLY_BYTE 61

typedef struct Rpi {
	int setValue;
} RPI;

extern RPI rpi;
extern bool autoPilotMode;
extern int ARM_Init(int _id);
	
extern void lBeep();
extern void send_Arm_ReadyToFly();
extern void send_Arm_Flying();

#endif

