#ifndef __SONAR_H__
#define __SONAR_H__

#include <exception>
#include <stdint.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>


class SONAR {
public:
	uint16_t id;
	int distance;
	SONAR(uint16_t _id);
	virtual bool RequestData();
	virtual ~SONAR();
	uint16_t GetValues();
	void SetID(uint16_t newid);
};

#endif
