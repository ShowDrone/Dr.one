#include "Mqtt.h"
#define MQTT_DEBUG 1 // MQTT

char *user = "pi";      // Raspberry Pi ID
char *pw = "vkdlemfhs"; // Raspberry Pi Password
char *mqbuf = (char *)malloc(70 * sizeof(char));

struct mosquitto *mosq = NULL;

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

	mosquitto_topic_matches_sub("pidrone/CMD/MV",message->topic,&match);
	if (match)
	{
		msg=(char *)message->payload;

		s1=strtok(msg,",");
		roll.server=round(atof(s1)*45) + 45;
		s1=strtok(NULL,",");
		pitch.server=round(atof(s1)*45) + 45;
		s1=strtok(NULL,",");
		yaw.server=round(atof(s1)*45) + 45;
		s1=strtok(NULL,",");
		bldcSpeed = atoi(s1)*255;
		s1=strtok(NULL,",");
		servo.y = atof(s1)*50;
		s1=strtok(NULL,"\r");
		servo.x = atof(s1)*50;
		
		//printf("input : roll %d, pitch %d, yaw %d, bl %d, ser %f, ser %f\r\n",roll.server,pitch.server,yaw.server,bldcSpeed,servo.y,servo.x);	
	}

	// PID GAIN, return	
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
			s1 = strtok(NULL, "\r");
		    	dc0.d = atof(s1);
			setSeparatePID(&dc0);
		    	printf("dc0: %.3f %.3f %.3f\r\n", dc0.p, dc0.i, dc0.d);
		}
		if(getid == 1) {
			s1 = strtok(NULL, ",");
		    	dc1.p = atof(s1);
			s1 = strtok(NULL, ",");
		    	dc1.i = atof(s1);
			s1 = strtok(NULL, "\r");
		    	dc1.d = atof(s1);
			setSeparatePID(&dc1);
			printf("dc1: %.3f %.3f %.3f\r\n", dc1.p, dc1.i, dc1.d);
		}
		if(getid == 2) {
			s1 = strtok(NULL, ",");
		    	dc2.p = atof(s1);
			s1 = strtok(NULL, ",");
		    	dc2.i = atof(s1);
			s1 = strtok(NULL, "\r");
		    	dc2.d = atof(s1);
			setSeparatePID(&dc2);
			printf("dc2: %.3f %.3f %.3f\r\n", dc2.p, dc2.i, dc2.d);
		}
		if(getid == 3) {
			s1 = strtok(NULL, ",");
		    	dc3.p = atof(s1);
			s1 = strtok(NULL, ",");
		    	dc3.i = atof(s1);
			s1 = strtok(NULL, "\r");
		    	dc3.d = atof(s1);
			setSeparatePID(&dc3);
			printf("dc3: %.3f %.3f %.3f\r\n", dc3.p, dc3.i, dc3.d);
		}
		if(getid == 4) {
			s1 = strtok(NULL, ",");
			//startFlying = (int)atof(s1);

		}
		//int16_t testVariable = 0;
		//testVariable = (dc0.pGain.decimalH << 8) | (dc0.pGain.decimalL);
		//float ta = (float)testVariable / 10000;
		//printf("dc0.data: %d %d %d %d %d %f\r\n", dc0.pGain.integerL, dc0.pGain.integerH, dc0.pGain.decimalL, dc0.pGain.decimalH,testVariable, ta);
		
	}
	mosquitto_topic_matches_sub("pidrone/PID/BL", message->topic, &match);
	if (match) {
		msg = (char *)message->payload;
		s1 = strtok(msg, ",");
		bl.p = atof(s1);
		s1 = strtok(NULL, ",");
		bl.i = atof(s1);
		s1 = strtok(NULL, "\r");
		bl.d = atof(s1);;
		printf("BL: %f %f %f\r\n", bl.p, bl.i, bl.d); 
		setSeparatePID(&bl);
	}
}


void mq_init() {
	mosquitto_lib_init();
	mosq = mosquitto_new(NULL, true, NULL);
	mosquitto_username_pw_set(mosq, user, pw);
	mosquitto_message_callback_set(mosq, message_callback);
	if (mosquitto_connect(mosq, "168.188.40.28", 1883, 60)) {
		printf("Mqtt connect error\r\n");
	}
	printf("Mqtt connect Ok\r\n");
	mosquitto_subscribe(mosq, NULL, "pidrone/CMD/#", 0);
	mosquitto_subscribe(mosq, NULL, "pidrone/PID/#", 0);
}

void mq_start() {
	int stat = mosquitto_loop_start(mosq);
	while (stat) {
		printf("Mqtt connection error!\r\n");
		//usleep(20000);
		mosquitto_reconnect(mosq);
	}
	printf("Mqtt Start Ok\r\n");
}

int mq_send(const char *topic, const char *msg) {
	return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 0, 0);
}

void mq_close() {
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
}
