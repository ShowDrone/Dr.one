#ifndef __MQTT_H__
#define __MQTT_H__

#include <mosquitto.h>
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include "main.h"
#include "SendToArm.h"

extern struct mosquitto *mosq;

extern char *mqbuf;
extern int mq_send(const char *topic, const char *msg);
extern void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
extern void mq_init();
extern void mq_start();
extern int mq_send(const char *topic, const char *msg);
extern void mq_close();
#endif
