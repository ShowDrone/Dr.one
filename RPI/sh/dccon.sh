#!/bin/sh
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/DC -m "4,30\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "0,10.35,20.333,30.123\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "1,10.35,50.333,40.123\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "2,20.35,40.333,50.123\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "3,30.35,30.333,60.123\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/BL -m "2,1,3\r\n"