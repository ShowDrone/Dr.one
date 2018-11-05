#!/bin/sh
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/DC -m "4,90\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/PID/BL -m "30,40,30\r\n"