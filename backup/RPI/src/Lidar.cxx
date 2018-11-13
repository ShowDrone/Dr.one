
#include <Lidar.h>


LIDAR :: LIDAR () {}

void LIDAR::begin(int configuration, char lidarAddress) {
   lidar = wiringPiI2CSetup(lidarAddress);
   configure(configuration); // Configuration settings
}

void LIDAR::configure(int configuration) {
  switch (configuration) {
    case 0: // Default mode, balanced performance
        wiringPiI2CWriteReg8(lidar, 0x02, 0x80);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x08);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0x00);
    break;
    case 1: // Short range, high speed
        wiringPiI2CWriteReg8(lidar, 0x02, 0x1d);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x08);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0x00);
    break;
    case 2: // Default range, higher speed short range
        wiringPiI2CWriteReg8(lidar, 0x02, 0x80);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x00);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0x00);
    break;

    case 3: // Maximum range
        wiringPiI2CWriteReg8(lidar, 0x02, 0xff);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x08);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0x00);
    break;
    case 4: // High sensitivity detection, high erroneous measurements
        wiringPiI2CWriteReg8(lidar, 0x02, 0x80);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x08);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0x80);
    break;
    case 5: // Low sensitivity detection, low erroneous measurements
        wiringPiI2CWriteReg8(lidar, 0x02, 0x80);
        wiringPiI2CWriteReg8(lidar, 0x04, 0x08);
        wiringPiI2CWriteReg8(lidar, 0x1c, 0xb0);
    break;
  }
}

void LIDAR::reset(char lidarAddress) {
  wiringPiI2CWriteReg8(lidar, 0x00, 0x00);
}


void LIDAR::read(char myAddress, int numOfBytes, uint8_t arrayToSave[2], bool monitorBusyFlag) {
  int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
  if(monitorBusyFlag) {
    busyFlag = 1; // Begin read immediately if not monitoring busy flag
  }
  int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout

  while(busyFlag != 0) { 
    // Read status register to check busy flag
    busyFlag = bitRead(wiringPiI2CReadReg8(lidar, 0x01), 0); // Assign the LSB of the status register to busyFlag
    busyCounter++;
    if(busyCounter > 9999) {
      goto bailout;
    }
  }
  // Device is not busy, begin read
  if(busyFlag == 0) {
    switch(numOfBytes) {
        case 1:
            arrayToSave[0] = wiringPiI2CReadReg8(lidar, (int)myAddress);
            break;
        case 2:
            arrayToSave[0] = wiringPiI2CReadReg8(lidar, (int)myAddress);
            arrayToSave[1] = wiringPiI2CReadReg8(lidar, (int)myAddress+1);
            break;
    }
  }

  // bailout reports error over serial
  if(busyCounter > 9999) {
      bailout:
      busyCounter = 0;
      //Serial.println("> read failed");
  }
}


int LIDAR::distance(bool biasCorrection, char lidarAddress) {
  if(biasCorrection) {
     wiringPiI2CWriteReg8(lidar, 0x00, 0x04);
  }
  else {
     wiringPiI2CWriteReg8(lidar, 0x00, 0x03);
  }
  uint8_t distanceArray[2];  // 0x08에서 2바이트,  
  read(0x8f,2,distanceArray,1);
  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return(distance);
}


void LIDAR::correlationRecordToSerial(char separator, int numberOfReadings) {
  uint8_t correlationArray[2];
  int correlationValue = 0;
  wiringPiI2CWriteReg8(lidar, 0x5d, 0xc0);
  wiringPiI2CWriteReg8(lidar, 0x40, 0x07);
  for(int i = 0; i<numberOfReadings; i++) {
    // Select single byte
    read(0xd2,2,correlationArray,false);
    correlationValue = correlationArray[0];
    if(correlationArray[1] == 1){
      correlationValue |= 0xff00;
    }
  }
  // test mode disable
  wiringPiI2CWriteReg8(lidar,0x40,0x00);
} /* LIDARLite::correlationRecordToSerial */

