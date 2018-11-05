#ifndef __RPI_H__
#define __RPI_H__

#include <wiringPi.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


typedef struct Rpi {
	int setValue;
} RPI;

RPI rpi = { 0 };

int startCount = 0;
int endCount = 6;
char *user = "pi";      // Raspberry Pi ID
char *pw = "vkdlemfhs"; // Raspberry Pi Password

void lBeep() {
	if (endCount > 0) {
		if (startCount < endCount) {
			if (startCount % 2 == 0) {
				//setAngle(bz, 0);    // buz on                                                                                                                                                                                        
			}
			else {
				//setAngle(bz, 255); // buz off
			}
			startCount++;
		}
		else {
			startCount = 0;
			endCount = 0;
		}
	}
}

#endif
