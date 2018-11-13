/*
 * change sonarid.cxx
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


#include <iostream>
#include "Sonar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


int main(int argc, char **argv)
{
		if(wiringPiSetup()==-1)
		return 1;
		
		int id=strtol(argv[1]+2,NULL,16);
		int newid=strtol(argv[2]+2,NULL,16);
	SONAR sonar=SONAR(id);	
	sonar.SetID(newid);
	printf("%i to %i\r\n",id,newid);
	delay(100);
	printf("done\r\n");
	
	return 0;
}

