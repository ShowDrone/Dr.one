#!/bin/sh
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/DC -m "10,9\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/BL -m "10,9\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/DC -m "10,9\r\n"
mosquitto_pub -h 168.188.40.28 -t pidrone/CMD/DC -m "10,9\r\n"