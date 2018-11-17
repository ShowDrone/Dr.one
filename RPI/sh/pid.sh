#!/bin/sh
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/DC -m "1,50,1,0\r\n"
