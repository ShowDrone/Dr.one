#ifndef __MQTT_H__
#define __MQTT_H__

#include <mosquitto.h>
#include "SendToArm.h"
#include <string.h>
#include <cstdlib>
#include "main.h"

#define MQTT_DEBUG 1 // MQTT �κ� ����� �޽��� ��� ���� 1�̸� ���, 0�̸� �����

struct mosquitto *mosq = NULL;
char *mqbuf = (char *)malloc(70 * sizeof(char));
int mq_send(const char *topic, const char *msg);
char *user = "pi";      // Raspberry Pi ID
char *pw = "vkdlemfhs"; // Raspberry Pi Password

void setSeparate(DC dc);
void setSeparate(BL bl);

void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {

	bool match = 0;
	char *msg;
	char *s1;
	char *s2;
	int getid;
	float getValue;

	mosquitto_topic_matches_sub("pidrone/CMD/PI", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		if (MQTT_DEBUG == 1)
			printf("pi command= %i\r\n", atoi(msg));

		rpi.setValue = atoi(msg);

		if (rpi.setValue == 0) {
			// 0으로 모터 및 전부 셋하는 코드 
		}
	}

	mosquitto_topic_matches_sub("pidrone/CMD/DC", message->topic, &match);
	if (match) {	
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		s2 = strtok(NULL, "\r");
		getid = atoi(s1);
		getValue = atof(s2);
		//~ printf("getid= %i getValue=%.1f\r\n",getid,getValue);
		//~ printf("getid= %i getValue=%.1f\r\n",dc.id,getValue); 
		//~ if(getid==dc.id)
		if (getid == 1) {
			/*
			dcgm.setValue = getValue;
			if(MQTT_DEBUG == 1)
				printf("dcgm setValue=%.1f\r\n", dcgm.setValue);
				*/
		}
		else if (getid == 2) {
			/*
			dcgp.setValue = getValue;
			if(MQTT_DEBUG == 1)
				printf("dcgp setValue=%.1f\r\n", dcgp.setValue);
				*/
		}
		else if (getid == 3) {
			/*
			dcgh.setValue = getValue;
			if(MQTT_DEBUG == 1)
				printf("dcgh setValue=%.1f\r\n", dcgh.setValue);
			*/
		}
		else if (getid == 4) { // pitch
   	   		pitch.server = getValue;
   			if(MQTT_DEBUG == 1)
        		printf("pitch.server=%.1f\r\n", pitch.server);
   		}
    	else if (getid == 5) { // pitch
			/*
     		em1.setValue = getValue;
      		if (MQTT_DEBUG == 1)
        		printf("em1 setValue=%.1f\r\n", em1.setValue);
			*/
    	}
   		else if (getid == 6) { // roll
      		roll.server = getValue;
      		if (MQTT_DEBUG == 1)
        		printf("roll.server=%.1f\r\n", roll.server);
    	}
    	else if (getid == 7) { // roll
      		/*
			em3.setValue = getValue;
      		if (MQTT_DEBUG == 1)
        		printf("roll.ser=%.1f\r\n", em3.setValue);
			*/
    	}
    	else if (getid == 10) { // Servo
     		servo.y = getValue;
      		if (MQTT_DEBUG == 1)
        		printf("servo=%.1f\r\n", servo.y);
  	    }
    	else if (getid == 11) {
			/*
      		sm1.setValue = getValue;
      		if (MQTT_DEBUG == 1)
        		printf("sm1 setValue=%.1f\r\n", sm1.setValue);
			*/
   		}
	}
	mosquitto_topic_matches_sub("pidrone/CMD/BL", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		s2 = strtok(NULL, "\r");
		getid = atoi(s1);
		getValue = atof(s2);
		if (getid == 8) { // yaw
			yaw.y = (int)getValue;
			if (MQTT_DEBUG == 1);
				printf("yaw=%i\r\n", yaw.y);
		}
		if (getid == 9) {	// bldcSpeed
			bldcSpeed = (int)getValue;
			if (MQTT_DEBUG == 1);
				printf("bldcSpeed=%i\r\n", bldcSpeed);
		}
	}

	// PID GAIN, 스피드 등으로 return 해주는 코드		
	mosquitto_topic_matches_sub("pidrone/CMD/FG/get", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, "");
		if (atoi(s1) == 0) {
			// ��û�� ���� ����̶� ����׿� ���� ��� ����ó���� ���� ����.
			/*
			printf("dc0 max=%f,min=%f,tmax=%f,tmin=%f\r\n", dc0.max, dc0.min, dc0.tmax, dc0.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", dc0.id, dc0.max, dc0.min, dc0.tmax, dc0.tmin);
			mq_send("pidrone/FG", mqbuf);
			printf("dc1 max=%f,min=%f,tmax=%f,tmin=%f\r\n", dc1.max, dc1.min, dc1.tmax, dc1.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", dc1.id, dc1.max, dc1.min, dc1.tmax, dc1.tmin);
			mq_send("pidrone/FG", mqbuf);
			printf("dc2 max=%f,min=%f,tmax=%f,tmin=%f\r\n", dc2.max, dc2.min, dc2.tmax, dc2.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", dc2.id, dc2.max, dc2.min, dc2.tmax, dc2.tmin);
			mq_send("pidrone/FG", mqbuf);
			printf("dc3 max=%f,min=%f,tmax=%f,tmin=%f\r\n", dc3.max, dc3.min, dc3.tmax, dc3.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", dc3.id, dc3.max, dc3.min, dc3.tmax, dc3.tmin);
			mq_send("pidrone/FG", mqbuf);

			printf("bl0 max=%i,min=%i,tmax=%f,tmin=%f\r\n", bl0.max, bl0.min, bl0.tmax, bl0.tmin);
			sprintf(mqbuf, "%u,%i,%i,%f,%f\r\n", bl0.id, bl0.max, bl0.min, bl0.tmax, bl0.tmin);
			mq_send("pidrone/FG", mqbuf);
			printf("bl1 max=%i,min=%i,tmax=%f,tmin=%f\r\n", bl1.max, bl1.min, bl1.tmax, bl1.tmin);
			sprintf(mqbuf, "%u,%i,%i,%f,%f\r\n", bl1.id, bl1.max, bl1.min, bl1.tmax, bl1.tmin);
			mq_send("pidrone/FG", mqbuf);

			printf("cm0 max=%f,min=%f,tmax=%f,tmin=%f\r\n", cm0.max, cm0.min, cm0.tmax, cm0.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", cm0.id, cm0.max, cm0.min, cm0.tmax, cm0.tmin);
			mq_send("pidrone/FG", mqbuf);
			printf("cm1 max=%f,min=%f,tmax=%f,tmin=%f\r\n", cm1.max, cm1.min, cm1.tmax, cm1.tmin);
			sprintf(mqbuf, "%u,%f,%f,%f,%f\r\n", cm1.id, cm1.max, cm1.min, cm1.tmax, cm1.tmin);
			mq_send("pidrone/FG", mqbuf);
			*/
		}
	}

	mosquitto_topic_matches_sub("pidrone/CMD/FG/set", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		getid = atoi(s1);
		if (getid == 4) {
			/*
			s1 = strtok(NULL, ",");
			em0.max = atof(s1);
			s1 = strtok(NULL, ",");
			em0.min = atof(s1);
			s1 = strtok(NULL, ",");
			int tmax = atof(s1);	// 필요없는 데이터
			s1 = strtok(NULL, "");
			int tmin = atof(s1);	// 필요없는 데이터
			*/
		}

		if (getid == 5) {
			/*
			s1 = strtok(NULL, ",");
			em1.max = atof(s1);
			s1 = strtok(NULL, ",");
			em1.min = atof(s1);
			s1 = strtok(NULL, ",");
			int tmax = atof(s1);
			s1 = strtok(NULL, "");
			int tmin = atof(s1);
			*/
		}

		if (getid == 6) {
			/*
			s1 = strtok(NULL, ",");
			em2.max = atof(s1);
			s1 = strtok(NULL, ",");
			em2.min = atof(s1);
			s1 = strtok(NULL, ",");
			em2.tmax = atof(s1);
			s1 = strtok(NULL, "");
			em2.tmin = atof(s1);
			*/
		}

		if (getid == 7) {
			/*
			s1 = strtok(NULL, ",");
			em3.max = atof(s1);
			s1 = strtok(NULL, ",");
			dc3.min = atof(s1);
			s1 = strtok(NULL, ",");
			dc3.tmax = atof(s1);
			s1 = strtok(NULL, "");
			dc3.tmin = atof(s1);
			*/
		}

		if (getid == 8) {
			/*
			s1 = strtok(NULL, ",");
			bl0.max = atoi(s1);
			s1 = strtok(NULL, ",");
			bl0.min = atoi(s1);
			s1 = strtok(NULL, ",");
			bl0.tmax = atof(s1);
			s1 = strtok(NULL, "");
			bl0.tmin = atof(s1);
			*/
		}

		if (getid == 9) {
			/*
			s1 = strtok(NULL, ",");
			bl1.max = atoi(s1);
			s1 = strtok(NULL, ",");
			bl1.min = atoi(s1);
			s1 = strtok(NULL, ",");
			bl1.tmax = atof(s1);
			s1 = strtok(NULL, "");
			bl1.tmin = atof(s1);
			*/
		}
		if (getid == 10) {
			/*
			s1 = strtok(NULL, ",");
			cm0.max = atof(s1);
			s1 = strtok(NULL, ",");
			cm0.min = atof(s1);
			s1 = strtok(NULL, ",");
			cm0.tmax = atof(s1);
			s1 = strtok(NULL, "");
			cm0.tmin = atof(s1);
			*/
		}
		if (getid == 11) {
			/*
			s1 = strtok(NULL, ",");
			cm1.max = atof(s1);
			s1 = strtok(NULL, ",");
			cm1.min = atof(s1);
			s1 = strtok(NULL, ",");
			cm1.tmax = atof(s1);
			s1 = strtok(NULL, "");
			cm1.tmin = atof(s1);
			*/
		}
	}
	
	mosquitto_topic_matches_sub("pidrone/PID/DC", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		getid = atoi(s1);
		if(getid == 0) {
			s1 = strtok(NULL, ",");
		    dc0.p = atof(s1);
			s1 = strtok(NULL, ",");
		    dc0.i = atof(s1);
			s1 = strtok(NULL, "");
		    dc0.d = atof(s1);
			setSeparate(dc0);
		}
		if(getid == 1) {
			s1 = strtok(NULL, ",");
		    dc1.p = atof(s1);
			s1 = strtok(NULL, ",");
		    dc1.i = atof(s1);
			s1 = strtok(NULL, "");
		    dc1.d = atof(s1);
			setSeparate(dc1);
		}
		if(getid == 2) {
			s1 = strtok(NULL, ",");
		    dc2.p = atof(s1);
			s1 = strtok(NULL, ",");
		    dc2.i = atof(s1);
			s1 = strtok(NULL, "");
		    dc2.d = atof(s1);
			setSeparate(dc2);
		}
		if(getid == 3) {
			s1 = strtok(NULL, ",");
		    dc3.p = atof(s1);
			s1 = strtok(NULL, ",");
		    dc3.i = atof(s1);
			s1 = strtok(NULL, "");
		    dc3.d = atof(s1);
			setSeparate(dc3);
		}
	}
	mosquitto_topic_matches_sub("pidrone/PID/BL", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		bl.p = atof(s1);
		s1 = strtok(NULL, ",");
		bl.i = atof(s1);
		s1 = strtok(NULL, ",");
		bl.d = atof(s1);
		s1 = strtok(NULL, "");
		setSeparate(bl);
	}
}


void mq_init() {
	mosquitto_lib_init();
	mosq = mosquitto_new(NULL, true, NULL);
	mosquitto_username_pw_set(mosq, user, pw);
	mosquitto_message_callback_set(mosq, message_callback);
	if (mosquitto_connect(mosq, "168.188.40.28", 1883, 60)) {
		printf("mqtt connect error\r\n");;
	}
	mosquitto_subscribe(mosq, NULL, "pidrone/CMD/#", 0);
}

void mq_start() {
	int stat = mosquitto_loop_start(mosq);
	while (stat) {
		printf("connection error!\r\n");
		//usleep(20000);
		mosquitto_reconnect(mosq);
	}
}

int mq_send(const char *topic, const char *msg) {
	return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 0, 0);
}

void mq_close() {
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
}


void setSeparate(DC dc) {
	dc.pGain.temp = floor(dc.p);
	dc.iGain.temp = floor(dc.i);		
	dc.dGain.temp = floor(dc.d);

	dc.pGain.integerL = dc.pGain.temp;
	dc.iGain.integerL = dc.iGain.temp;
	dc.dGain.integerL = dc.dGain.temp;

	dc.pGain.temp = (dc.p - dc.pGain.temp) * 10000;
	dc.iGain.temp = (dc.i - dc.iGain.temp) * 10000;
	dc.dGain.temp = (dc.d - dc.dGain.temp) * 10000;

	dc.pGain.decimalL = dc.pGain.temp & 0xff;
	dc.iGain.decimalL = dc.iGain.temp & 0xff;
	dc.dGain.decimalL = dc.dGain.temp & 0xff;

	dc.pGain.decimalH = dc.pGain.temp >> 8;
	dc.iGain.decimalH = dc.iGain.temp >> 8;
	dc.dGain.decimalH = dc.dGain.temp >> 8;
}

void setSeparate(BL bl) {
	bl.pGain.temp = floor(bl.p);
	bl.iGain.temp = floor(bl.i);    
	bl.dGain.temp = floor(bl.d);

	bl.pGain.integerL = bl.pGain.temp;
	bl.iGain.integerL = bl.iGain.temp;
	bl.dGain.integerL = bl.dGain.temp;

	bl.pGain.temp = (bl.p - bl.pGain.temp) * 10000;
	bl.iGain.temp = (bl.i - bl.iGain.temp) * 10000;
	bl.dGain.temp = (bl.d - bl.dGain.temp) * 10000;

	bl.pGain.decimalL = bl.pGain.temp & 0xff;
	bl.iGain.decimalL = bl.iGain.temp & 0xff;
	bl.dGain.decimalL = bl.dGain.temp & 0xff;

	bl.pGain.decimalH = bl.pGain.temp >> 8;
	bl.iGain.decimalH = bl.iGain.temp >> 8;
	bl.dGain.decimalH = bl.dGain.temp >> 8;
}



#endif
