#!/bin/sh
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "0,20.111,10.33,10.44\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "1,20.111,10.33,10.44\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "2,20.111,10.33,10.44\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "3,20.111,10.33,10.44\r\n"