#ifndef __MAIN_H__
#define __MAIN_H__


/* Includes ------------------------------------------------------------------*/    
#include <RTIMULib.h>
#include "Sonar.h"
#include <mosquitto.h>
#include "gps.h"
#include "Mqtt.h"
#include <Lidar.h>
#include "SendToArm.h"
#include <wiringPi.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define MAIN_DEBUG 1  // Debug msg output
#define CONTROL_PERIOD 10000 		// 10ms
#define CONTROL_PERIOD_GPS 100000	// 100ms
#define DISPLAY_PERIOD 100000		// 100ms

/* Private Declaration -------------------------------------------------------*/
typedef struct Angle {
    float server;
    float y;
    float bias;
    APART data;
} ANGLE;

typedef struct Servo {
    float y;
    bool mode;
} SERVO;

typedef struct Apart {
    float temp;
    uint8_t integerL;
    uint8_t integerH;
    uint8_t decimalL;
    uint8_t decimalH;
} APART;

typedef struct Pid {
    float p;
    float i;
    float d;
    APART pGain;
    APART iGain;
    APART dGain;
} PID;

extern ANGLE pitch;
extern ANGLE roll ;
extern ANGLE yaw  ;
extern PID   dc0;
extern PID   dc1;
extern PID   dc2;
extern PID   dc3;
extern PID   bl;

extern SERVO servo;
extern uint8_t bldcSpeed;

float lidar_distance = 0;
uint64_t rateTimer, rateTimer_s;
uint64_t tok, tokold, toktime;
uint64_t sonarTimer, displayTimer;
uint64_t now; 
uint16_t fd;
float ref = 0;
int cnt = 0, tokcnt = 1;
bool sonarFlag = true;
RTIMU_DATA imuData;
char *imuresult;
LIDAR lidar;


#endif

