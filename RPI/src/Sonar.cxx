/*
 * Sonar.cxx
 * 
 * Copyright 2017  <pi@raspberrypi>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include "Sonar.h"
SONAR::SONAR(uint16_t _id) {
	id=wiringPiI2CSetup(_id);
}

bool SONAR::RequestData() {
	return wiringPiI2CWriteReg8(id,0,81); //send 81(cm response) to 0(command)
}

uint16_t SONAR::GetValues() {
	return wiringPiI2CReadReg16(id,2)/256;// read 2 byte from 2(result hi)
}

SONAR::~SONAR() {

}

void SONAR::SetID(uint16_t newid) {
                    wiringPiI2CWriteReg8(id,0,0xA0);
                    delay(20);
                    wiringPiI2CWriteReg8(id,0,0xAA);
                    delay(20);
                    wiringPiI2CWriteReg8(id,0,0xA5);
                    delay(20);
                    if(newid>0x77)
                    {
                        newid=0x77;
                    }
                    wiringPiI2CWriteReg8(id,0,newid*2);
}
