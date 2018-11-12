
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <inttypes.h>

#ifndef PORTNAME
#define PORTNAME " /dev/ttyAMA0"
#endif

void serial_init(void);
void serial_config(void);
void serial_println(const char *, int);
void serial_readln(char *, int);
extern void GPS_close(void);

#endif
