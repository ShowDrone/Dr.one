
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/    
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <math.h>
#include <RTIMULib.h>
#include <mosquitto.h>
#include <signal.h>
#include <softPwm.h>
#include <serial.h>
#include "Sonar.h"
#include "gps.h"
#include "Mqtt.h"
#include "Lidar.h"
#include "SendToArm.h"

/* Private define ------------------------------------------------------------*/
#define MAIN_DEBUG 1  // Debug msg output
#define CONTROL_PERIOD 10000 		// 10ms
#define CONTROL_PERIOD_GPS 100000	// 100ms
#define DISPLAY_PERIOD 100000		// 100ms

#define I2C_ADDRESS_SONAR0 0x71
#define I2C_ADDRESS_SONAR1 0x72
#define I2C_ADDRESS_MPU9150 0x68
#define SERVO_LANDING 1


/* Private Declaration -------------------------------------------------------*/

typedef struct Servo {
    float y;
    float x;
    bool mode;
} SERVO;

typedef struct Apart {
    float temp;
    char integerL;
    char integerH;
    char decimalL;
    char decimalH;
} APART;

typedef struct Angle {
    float server;
    float y;
    float bias;
    APART data;
} ANGLE;

typedef struct Pid {
    float p;
    float i;
    float d;
    APART pGain;
    APART iGain;
    APART dGain;
} PID;

extern ANGLE pitch;
extern ANGLE roll;
extern ANGLE yaw;
extern PID   dc0;
extern PID   dc1;
extern PID   dc2;
extern PID   dc3;
extern PID   bl;
extern SERVO servo;
extern float bldcSpeed;

#endif