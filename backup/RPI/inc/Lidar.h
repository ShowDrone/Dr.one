
#ifndef __LIDAR_H__
#define __LIDAT_H__

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>

#define LIDAR_ADDR 0x62
#define bitRead(value,bit)(((value)>>(bit))&0x01)

class LIDAR {
    public:
        LIDAR();
        void begin(int = 0, char = LIDAR_ADDR);
        void configure(int = 0);
        void reset(char = LIDAR_ADDR);
        int distance(bool = true, char = LIDAR_ADDR);
        void read(char, int, uint8_t*, bool);
        void correlationRecordToSerial(char = '\n', int = 256);
    private:
        int lidar;
};


#endif
 

