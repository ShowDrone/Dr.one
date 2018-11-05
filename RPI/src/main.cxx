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
char getKey();			// fixme, don't use
static void GPS_init();	// GPS 초기화 함수
static void IMU_Init(); // IMU 초기화 함수
static void SONAR_Init(uint16_t id0, uint16_t id1); // 소나 초기화 함수
static void MQTT_Init(); // MQTT 초기화 함수
static void PI_Init();   // PI 세팅 초기화 함수

// 구조체 선언
ANGLE pitch = {0,0,0,{0,0,0,0,0}};			
ANGLE roll  = {0,0,0,{0,0,0,0,0}};			
ANGLE yaw   = {0,0,0,{0,0,0,0,0}};			
SERVO servo = {0,0};							
PID   dc0   = {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc1	= {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc2	= {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   dc3	= {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
PID   bl0	= {0,0,0,{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

uint8_t bldcSpeed = 0;						

int main(int argc, char **argv) {

	/* Initialize Object */
	loc_t gps;
	RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
	RTIMU *imu = RTIMU::createIMU(settings);
	SONAR sonar0;
	SONAR sonar1;

	/* Interrupt for EXIT */
	signal(SIGINT, INThandler);
		
	/* Initialize all configured peripherals */
	SONAR_Init(0x71, 0x72);		
	IMU_Init();				    
	PI_Init();					
	GPS_Init();					
	MQTT_Init();				
	lidar.begin(0, 0x62);

	// 프로그램 루프 시간 기록을 위해 진입 전에 시작 시간 대입
	rateTimer = displayTimer = rateTimer_s = micros();

	while (1) {
	
	    // imu로 9축 데이터 읽어 오는 곳.
		while(imu->IMURead()); 
		imuData = imu->getIMUData();
		
		// 칼만필터링을 통해 각도로 변환한 걸 문자열로 저장한걸 roll, pitch, yaw로 구분
		imuresult=(char *)RTMath::displayDegrees("",imuData.fusionPose);
		imuresult=strtok(imuresult,":");
		imuresult=strtok(NULL,",");
		roll.y =atof(imuresult) + 180;  // roll
		imuresult=strtok(NULL,":");

		imuresult=strtok(NULL,",");
		pitch.y=atof(imuresult) + 180; //pitch
		
		imuresult=strtok(NULL,":");
		imuresult=strtok(NULL,":");
		yaw.y=atof(imuresult) + 180.; //yaw
		tokold = micros();

		/* ARM으로 0~360의 데이터를 보내기 위해, 정수 부분을 2byte, 소수 부분을 2byte로 나누는 작업*/
		// 소수점 뗴기
		roll.data.temp  = floor(roll.y);		
		pitch.data.temp = floor(pitch.y);
		yaw.data.temp = floor(yaw.y);

		// 정수 부분의 데이터가 1byte[255] 초과될 경우 Low, High로 데이터 분리
		if (abs(roll.data.temp) > 255 ) {
			roll.data.integerL = 255;
			roll.data.integerH = roll.data.temp - 255;
		}
		if (abs(pitch.data.temp) > 255 ) {
			pitch.data.integerL = 255;
			pitch.data.integerH = pitch.data.temp - 255;
		}
		if (abs(yaw.data.temp) > 255 ) {			
			yaw.data.integerL = 255;				
			yaw.data.integerH = yaw.data.temp - 255;	
		}  									
		
		// 소수는 통신이 안되니 정수형으로 전환
		roll.data.temp = (roll.y - roll.data.temp) 	  * 10000;
		pitch.data.temp = (pitch.y - pitch.data.temp) * 10000;
		yaw.data.temp = (yaw.y - yaw.data.temp) 	  * 10000;
    	
		// 실수 데이터를 2byte로 구분
		roll.data.decimalL = roll.data.temp 	& 0xff;
		pitch.data.decimalL = pitch.data.temp & 0xff;
		yaw.data.decimalL = yaw.data.temp 	& 0xff;

		roll.data.decimalH = roll.data.temp 	>> 8;
		pitch.data.decimalH = pitch.data.temp >> 8;
		yaw.data.decimalH = yaw.data.temp 	>> 8;

	
		lidar_distance = lidar.distance();

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
		if ((now - rateTimer_GPS) >= CONTROL_PERIOD_GPS { // 100ms 마다 실행
			//gps_location(&gps);
			lBeep();
			rateTimer_s = now;			
			//printf("%f %f %f\n", gps.latitude, gps.longitude, gps.altitude);
		}
	    

		
		if (MAIN_DEBUG == 1) {
			printf("--------Data-------\n");
			printf("pitch: %3.3f roll: %3.3f yaw %3.3f\n", pitch, roll, yaw);
			printf("Lidar: %3.3f\n", lidar_distance);
			printf("GPS:   %3.3f %3.3f %3.3f\n", gps.latitude, gps.longitude, gps.altitude);
			printf("--------END--------\n\n");
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
			/*
			sprintf(mqbuf, "%lf,%lf,%lf,%i,%lf,%i\r", gps.latitude, gps.longitude, gps.altitude, gps.satellites, gps.speed*1.8);
			printf("%lf %lf %lf \n", gps.latitude, gps.longitude, gps.altitude);
			mq_send("pidrone/GPS", mqbuf);
			*/
			sprintf(mqbuf,"%s\r",RTMath::displayDegrees("",imuData.fusionPose));
			mq_send("pidrone/IMU",mqbuf);

			toktime = toktime / tokcnt;
			tokcnt = 0;
			/*
			sprintf(mqbuf, "measured time is %llds.\r\n", toktime);
			mq_send("pidrone/PI", mqbuf);
			*/
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
		while ((now - rateTimer) < CONTROL_PERIOD) {
			if (((now - sonarTimer) >= 60000) && sonarFlag) {
				sonar0.distance = sonar0.GetValues();
				sonar1.distance = sonar1.GetValues();
				sonarFlag = false;
			}
			now = micros();
		}
		rateTimer = now;
		cnt++;
	}
	return 0;
}

static void GPS_init() {
	gps_init();
	gps.latitude = 0;
	gps.longitude = 0;
	gps.altitude = 0;			
	gps.speed = 0.0;
	gps.satellites = 0;
}

static void IMU_Init() {
	if ((imu==NULL) || (imu->IMUType()==RTIMU_TYPE_NULL)) {
		printf("No IMU found\n");
		exit(1);
	}
	imu->IMUInit();
	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(true);
}

static void SONAR_Init(uint16_t id0, uint16_t id1) {
	sonar0 = SONAR(sonarid0);
	sonar1 = SONAR(sonarid1);
	sonar0.distance = 0;
	sonar1.distance = 0;
}

static void MQTT_Init() {
	mq_init();
	mq_start();
}

static void PI_Init() {
	if (wiringPiSetup() == -1) {
		if (MAIN_DEBUG == 1) {
			printf("Failed to wiringPiSetup\n");
		}
		return 1;
	}	
}


void  INThandler(int sig) {
	// Closing file and turning off Matrix
	unsigned short int clear[] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
	displayImage(clear, res, daddress, file);
	printf("Closing file and turning off \r\n");
	daddress = 0x20;
	for (daddress = 0xef; daddress >= 0xe0; daddress--) {
		res = i2c_smbus_write_byte(file, daddress);
	}
	gps_off();
	signal(sig, SIG_IGN);
	exit(0);
}

char getKey() {
	if(linux_kbhit()) 
		return getchar();
	return '\0';
}

