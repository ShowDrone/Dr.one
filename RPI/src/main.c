/*
* main.cxx
*
* Copyright 2017  <pi@raspberrypi>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* MA 02110-1301, USA.
*
*
* Commented by Seunghyun Lee
* Code modification by Seunghyun Lee
*
*/

// Include 

#include "main.h"

// Declaration

using namespace std;
void  INThandler(int sig);
void GPS_Init(loc_t *gps);	// GPS 초기화 함수
void IMU_Init(); // IMU 초기화 함수
void SONAR_Init(SONAR *sonar0, SONAR *sonar1); // 소나 초기화 함수
void MQTT_Init(); // MQTT 초기화 함수
void PI_Init();   // PI 세팅 초기화 함수
void PIDGAIN_Init();
void SERVO_Init();
void readPidGain();
void writePidGain();
void setSeparateAngle(ANGLE *axis);
void setSeparatePID(PID *pid);

// 구조체 선언
ANGLE pitch = {0,0,0,0,{0,0,0,0,0}};
ANGLE roll  = {0,0,0,0,{0,0,0,0,0}};				
ANGLE yaw   = {0,0,0,0,{0,0,0,0,0}};				
ANGLE arm   = {0,0,0,0,{0,0,0,0,0}};			
SERVO servo = {15,15,0};							
PID   dc0   = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc1   = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc2   = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc3   = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

PID   bl    = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

float bldcSpeed = 0;	
int fpidgain = 0;
char fpidgainBuf[1024]; 
RTIMU_DATA imuData;
char *imuresult;
RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
RTIMU *imu = RTIMU::createIMU(settings);

int main(int argc, char **argv) {
		
	/* Initialize Variable */
	float lidar_distance = 0;
	uint64_t rateTimerSonar, rateTimerGps;
	uint64_t tok, tokold, toktime;
	uint64_t sonarTimer, displayTimer;
	uint64_t now;
	int16_t fd;
	float ref = 0;
	int cnt = 0, tokcnt = 1;
	bool sonarFlag = true;
	LIDAR lidar;
	
	/* Initialize Object */
	loc_t gps;
	SONAR sonar0 = SONAR(I2C_ADDRESS_SONAR0);
	SONAR sonar1 = SONAR(I2C_ADDRESS_SONAR1);
	
	/* Interrupt for EXIT */
	signal(SIGINT, INThandler);
		
	/* Initialize all configured peripherals */			    
	PI_Init();
	IMU_Init();					
	GPS_Init(&gps);				
	MQTT_Init();		
	SONAR_Init(&sonar0, &sonar1);			
	SERVO_Init();
	ARM_Init(0x30);
	PIDGAIN_Init();
	lidar.begin(0, 0x62);
	readPidGain();	
	setSeparatePID(&dc0);
	setSeparatePID(&dc1);
	setSeparatePID(&dc2);
	setSeparatePID(&dc3);
	setSeparatePID(&bl);
	printf("Main\r\n");
	// 프로그램 루프 시간 기록을 위해 진입 전에 시작 시간 대입
	rateTimerSonar = displayTimer = rateTimerGps = micros();

	while (1) {
	   	// imu로 9축 데이터 읽어 오는 곳
		while(imu->IMURead()); 
		imuData = imu->getIMUData();
		// 칼만필터링을 통해 각도로 변환한 걸 문자열로 저장한걸 roll, pitch, yaw로 구분
		imuresult=(char *)RTMath::displayDegrees("",imuData.fusionPose);
		imuresult=strtok(imuresult,":");
		imuresult=strtok(NULL,",");
		roll.y =atof(imuresult)+180;  // roll
		imuresult=strtok(NULL,":");

		imuresult=strtok(NULL,",");
		pitch.y=atof(imuresult)+180; //pitch
		
		imuresult=strtok(NULL,":");
		imuresult=strtok(NULL,":");
		yaw.y=atof(imuresult) + 180.; //yaw
		yaw.y = yaw.y - yaw.prev;
		yaw.prev = yaw.y;
		tokold = micros();
		//printf("pitch: %3.3f roll: %3.3f yaw %3.3f\n", pitch.y, roll.y, yaw.y);

		
		/* ARM으로 0~360의 데이터를 보내기 위해, 정수 부분을 2byte, 소수 부분을 2byte로 나누는 작업*/
		// 소수점 뗴기

		setSeparateAngle(&roll);
		setSeparateAngle(&pitch);
		setSeparateAngle(&yaw); 	
		sendToArm();
		softPwmWrite(SERVO_LANDING, servo.y);
		//lidar_distance = lidar.distance();

		// FIXME, setValue에 값이 1이라면 pitch만 제어, 2라면 roll만 제어, 3이라면 yaw제어 , 4라면 전체 다 제어, [테스트 용도]
		if (rpi.setValue == 1 || rpi.setValue == 4) {			
			// Add control data read
			//~ dc0.setValue = ;
			//~ dc2.setValue = ;
			if (MAIN_DEBUG == 1) {
				//printf("dc0.setValue=\f, dc2,setValue=\f\r\n", dc0.setValue, dc2.setValue);
			}
		}

		if (rpi.setValue == 2 || rpi.setValue == 4) {
			// Add control data read
			//~ dc1.setValue = ;
			//~ dc3.setValue = ;
			if (MAIN_DEBUG == 1) {
				//printf("dc1.setValue=\f, dc3,setValue=\f\r\n", dc1.setValue, dc3.setValue);
			}
		}

		if (rpi.setValue == 3 || rpi.setValue == 4) {
			// Add control data read
			//~ yawgain = ;
			if (MAIN_DEBUG == 1) {
				//printf("yawgain=\f\r\n", yawgain);
			}
		}



		
		/* GPS Data */
		now = micros();
		/*if ((now - rateTimerGps) >= CONTROL_PERIOD_GPS) { // 100ms 마다 실행
			//gps_location(&gps);
			rateTimerGps = now;			
			//printf("%f %f %f\n", gps.latitude, gps.longitude, gps.altitude);
		}*/
	    

		
		if (MAIN_DEBUG == 1) {/*
			printf("--------Data-------\n");
			printf("pitch: %3.3f roll: %3.3f yaw %3.3f\n", pitch, roll, yaw);
			printf("Lidar: %3.3f\n", lidar_distance);
			printf("GPS:   %3.3f %3.3f %3.3f\n", gps.latitude, gps.longitude, gps.altitude);
			printf("--------END--------\n\n");
			fflush(stdout);*/
		}

		tok = micros();
		toktime = toktime + tok - tokold;
		tokcnt++;

		/* Data display (100ms) */
		// mqtt에 소나, gps, imu 데이터를 보내는 작업,
		if ((now - displayTimer) > DISPLAY_PERIOD) {
			sprintf(mqbuf, "%i,%i\r", sonar0.id, sonar0.distance);
			mq_send("pidrone/SONAR", mqbuf);
			sprintf(mqbuf, "%i,%i\r", sonar1.id, sonar1.distance);
			mq_send("pidrone/SONAR", mqbuf);
			sprintf(mqbuf, "%lf,%lf,%lf,%i,%lf,%i\r", gps.latitude, gps.longitude, gps.altitude, gps.satellites, gps.speed*1.8);
			//printf("%lf %lf %lf \n", gps.latitude, gps.longitude, gps.altitude);
			mq_send("pidrone/GPS", mqbuf);
			sprintf(mqbuf,"%s\r",RTMath::displayDegrees("",imuData.fusionPose));
			mq_send("pidrone/IMU",mqbuf);
			toktime = toktime / tokcnt;
			tokcnt = 0;
			sprintf(mqbuf, "measured time is %llds.\r\n", toktime);
			mq_send("pidrone/PI", mqbuf);
			fflush(stdout);
			displayTimer = now;
		}

		
		/* Sonar Data */
		// 소나 데이터 요청
		if (!sonarFlag) {
			sonar0.RequestData();  // Request Data
			sonar1.RequestData();
			sonarTimer = micros(); // Save Time
			sonarFlag = true;
		}

		/* 10ms 마다 실행되며, 데이터가 요청되었을 경우, reading */
		while ((now - rateTimerSonar) < CONTROL_PERIOD) {
			if (((now - sonarTimer) >= 60000) && sonarFlag) {
				sonar0.distance = sonar0.GetValues();
				sonar1.distance = sonar1.GetValues();
				sonarFlag = false;
			}
			now = micros();
		}
		rateTimerSonar = now;
		cnt++;
	}
	return 0;
}

void GPS_Init(loc_t *gps) {
	gps_init();
	gps->latitude = 0;
	gps->longitude = 0;
	gps->altitude = 0;			
	gps->speed = 0.0;
	gps->satellites = 0;
}

void IMU_Init() {
	if ((imu==NULL) || (imu->IMUType()==RTIMU_TYPE_NULL)) {
		printf("No IMU found\n");
		//exit(1);
	}
	if(!imu->IMUInit()) {
		printf("Imu Init Failed\r\n");
	}
	else {
		printf("IMU Init Succeeded\r\n");
	}
	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(true);
}

void SONAR_Init(SONAR *sonar0, SONAR *sonar1) {
	sonar0->distance = 0;
	sonar1->distance = 0;
}

void MQTT_Init() {
	mq_init();
	mq_start();
}

void PI_Init() {
	if (wiringPiSetup() == -1) {
		if (MAIN_DEBUG == 1) {
			printf("Failed to wiringPiSetup\n");
		}
	}	
}

void PIDGAIN_Init() {
	fpidgain = open("/home/pi/Dr.ONE/RPI/src/pidgain.txt", O_RDWR);
	if(fpidgain == -1) {
		printf("pidgain.txt open failed\r\n");
	}
}

void SERVO_Init() {
	softPwmCreate(SERVO_LANDING,0,500);
}


void setSeparateAngle(ANGLE *axis) {
	axis->data.temp = floor(axis->y);
	if(abs(axis->data.temp) > 254) {
		axis->data.integerL = 254;
		axis->data.integerH = axis->data.temp-254;
	}
	else {
		axis->data.integerL = axis->data.temp;
		axis->data.integerH = 0;
		
	}
	
	axis->data.temp = (axis->y - axis->data.temp) * 1000;
	axis->data.decimalL = (int)axis->data.temp & 0xff;
	axis->data.decimalH = (int)axis->data.temp >> 8;
}


void setSeparatePID(PID *pid) {
	pid->pGain.temp = floor(pid->p);
	pid->iGain.temp = floor(pid->i);		
	pid->dGain.temp = floor(pid->d);

	pid->pGain.integerL = pid->pGain.temp;
	pid->iGain.integerL = pid->iGain.temp;
	pid->dGain.integerL = pid->dGain.temp;

	pid->pGain.temp = (pid->p - pid->pGain.temp) * 1000;
	pid->iGain.temp = (pid->i - pid->iGain.temp) * 1000;
	pid->dGain.temp = (pid->d - pid->dGain.temp) * 1000;

	pid->pGain.decimalL = (int)pid->pGain.temp & 0xff;
	pid->iGain.decimalL = (int)pid->iGain.temp & 0xff;
	pid->dGain.decimalL = (int)pid->dGain.temp & 0xff;

	pid->pGain.decimalH = (int)pid->pGain.temp >> 8;
	pid->iGain.decimalH = (int)pid->iGain.temp >> 8;
	pid->dGain.decimalH = (int)pid->dGain.temp >> 8;
}

void readPidGain() {
	char *s1;
	read(fpidgain, fpidgainBuf, 1024);
	s1 = strtok(fpidgainBuf, "=");
	s1 = strtok(NULL, "\n");
	dc0.p = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc0.i = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc0.d = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc1.p = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc1.i = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc1.d = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc2.p = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc2.i = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc2.d = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc3.p = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc3.i = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	dc3.d = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	bl.p = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	bl.i = atof(s1);
	s1 = strtok(NULL, "=");
	s1 = strtok(NULL, "\n");
	bl.d = atof(s1);
	close(fpidgain);
	printf("a %f %f %f \r\n", dc0.p, dc0.i, dc0.d);
}

void writePidGain() {
	PIDGAIN_Init();
	sprintf(fpidgainBuf,
	"dc0.kp=%f\ndc0.ki=%f\ndc0.kd=%f\ndc1.kp=%f\ndc1.ki=%f\ndc1.kd=%f\ndc2.kp=%f\ndc2.ki=%f\ndc2.kd=%f\ndc3.kp=%f\ndc3.ki=%f\ndc3.kd=%f\nbl.kp=%f\nbl.ki=%f\nbl.kd=%f\n",
	dc0.p,dc0.i,dc0.d,dc1.p,dc1.i,dc1.d,dc2.p,dc2.i,dc2.d,dc3.p,dc3.i,dc3.d,bl.p,bl.i,bl.d);
	PIDGAIN_Init();	
	write(fpidgain, fpidgainBuf, strlen(fpidgainBuf));
	close(fpidgain);
}

void INThandler(int sig) {
	writePidGain();
	mq_close();
	setZeroProp();
	GPS_close();
	ARM_close();
	exit(0);
}

