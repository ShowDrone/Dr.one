
#ifndef __SENDTOARM_H__
#define __SENDTOARM_H__

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <i2c-dev.h>
#include "main.h"
#define FLYING_BYTE 18
#define READY_TO_FLY_BYTE 40

typedef struct Rpi {
	int setValue;
} RPI;

extern RPI rpi;
extern bool autoPilotMode;
extern int linux_kbhit();
extern void setI2C (int _id);
	
extern void lBeep();
extern void init_keyboard();
extern void close_keyboard();
extern int _kbhit();
extern int _getch();
extern int _putch(int c);
extern void send_Arm_ReadyToFly();
extern void send_Arm_Flying();

#endif

