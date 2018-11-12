#include "SendToArm.h"

#define FLYING_BYTE 18
#define READY_TO_FLY_BYTE 39
uint8_t checkSum(unsigned char *data, uint8_t len );

RPI rpi = { 0 };

int slave_id;
uint8_t checkSum_Byte = 0;     
unsigned char ReadyToFlyBuf[READY_TO_FLY_BYTE] = {0,};
unsigned char FlyingBuf[READY_TO_FLY_BYTE];
char txBuf[READY_TO_FLY_BYTE] = "";
bool startFlying = 0;
bool takeoff = 0;
int wlen;

int set_interface_attribs(int slave_id, int speed)
{
	struct termios tty;
	
	if (tcgetattr(slave_id, &tty) < 0) {
	        printf("Error from tcgetattr: %s\n", strerror(errno));
		 	 return -1;
	}
	
	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);
	
	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */
	
	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;
	
	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;
	
	if (tcsetattr(slave_id, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int ARM_Init(int _id) {
	
	char *portname = "/dev/ttyUSB0";
	
	slave_id = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (slave_id < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	
    	set_interface_attribs(slave_id, B115200);
	printf("ARM_Init Succeed\r\n");
}



 
void send_Arm_ReadyToFly() {

		
	ReadyToFlyBuf[0]  = dc0.pGain.integerL;
	ReadyToFlyBuf[1]  = dc0.pGain.decimalL;
	ReadyToFlyBuf[2]  = dc0.pGain.decimalH;
	ReadyToFlyBuf[3]  = dc0.iGain.integerL;
	ReadyToFlyBuf[4]  = dc0.iGain.decimalL;
	ReadyToFlyBuf[5]  = dc0.iGain.decimalH;
	ReadyToFlyBuf[6]  = dc0.dGain.integerL;
	ReadyToFlyBuf[7]  = dc0.dGain.decimalL;
	ReadyToFlyBuf[8]  = dc0.dGain.decimalH;
	ReadyToFlyBuf[9]  = dc1.pGain.integerL;
	ReadyToFlyBuf[10] = dc1.pGain.decimalL;
	ReadyToFlyBuf[11] = dc1.pGain.decimalH;
	ReadyToFlyBuf[12] = dc1.iGain.integerL;
	ReadyToFlyBuf[13] = dc1.iGain.decimalL;
	ReadyToFlyBuf[14] = dc1.iGain.decimalH;
	ReadyToFlyBuf[15] = dc1.dGain.integerL;
	ReadyToFlyBuf[16] = dc1.dGain.decimalL;
	ReadyToFlyBuf[17] = dc1.dGain.decimalH;
	
	ReadyToFlyBuf[18] = dc2.pGain.integerL;
	ReadyToFlyBuf[19] = dc2.pGain.decimalL;
	ReadyToFlyBuf[20] = dc2.pGain.decimalH;
	ReadyToFlyBuf[21] = dc2.iGain.integerL;
	ReadyToFlyBuf[22] = dc2.iGain.decimalL;
	ReadyToFlyBuf[23] = dc2.iGain.decimalH;
	ReadyToFlyBuf[24] = dc2.dGain.integerL;
	ReadyToFlyBuf[25] = dc2.dGain.decimalL;
	ReadyToFlyBuf[26] = dc2.dGain.decimalH;
	
	ReadyToFlyBuf[27] = dc3.pGain.integerL;
	ReadyToFlyBuf[28] = dc3.pGain.decimalL;
	ReadyToFlyBuf[29] = dc3.pGain.decimalH;
	ReadyToFlyBuf[30] = dc3.iGain.integerL;
	ReadyToFlyBuf[31] = dc3.iGain.decimalL;
	ReadyToFlyBuf[32] = dc3.iGain.decimalH;
	ReadyToFlyBuf[33] = dc3.dGain.integerL;
	ReadyToFlyBuf[34] = dc3.dGain.decimalL;
	ReadyToFlyBuf[35] = dc3.dGain.decimalH;
	
	ReadyToFlyBuf[37] = startFlying;
	checkSum_Byte = checkSum(ReadyToFlyBuf, READY_TO_FLY_BYTE-1);
	ReadyToFlyBuf[38] = checkSum_Byte;

	//memcpy(txBuf, ReadyToFlyBuf, READY_TO_FLY_BYTE);
	//printf("%d %d %d\r\n", ReadyToFlyBuf[0],ReadyToFlyBuf[2],ReadyToFlyBuf[3]);

	printf("%d %d %d\r\n", ReadyToFlyBuf[0],ReadyToFlyBuf[1],ReadyToFlyBuf[2]);
	wlen = write(slave_id, ReadyToFlyBuf, READY_TO_FLY_BYTE);
	if (wlen != READY_TO_FLY_BYTE) {
	  printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(slave_id);    /* delay for output */


}
void send_Arm_Flying() {
	/*
	FlyingBuf[0]  = roll.data.integerL;
	FlyingBuf[1]  = roll.data.integerH;
	FlyingBuf[2]  = roll.data.decimalL;
	FlyingBuf[3]  = roll.data.decimalH;   
	FlyingBuf[4]  = pitch.data.integerL;
	FlyingBuf[5]  = pitch.data.integerH;
	FlyingBuf[6]  = pitch.data.decimalL;
	FlyingBuf[7]  = pitch.data.decimalH;   
	FlyingBuf[8]  = yaw.data.integerL;
	FlyingBuf[9]  = yaw.data.integerH;
	FlyingBuf[10] = yaw.data.decimalL;
	FlyingBuf[11] = yaw.data.decimalH;   
	FlyingBuf[12] = roll.server;
	FlyingBuf[13] = pitch.server;
	FlyingBuf[14] = yaw.server;
	FlyingBuf[15] = bldcSpeed;
	FlyingBuf[16] = servo.mode;
	checkSum_Byte   = checkSum(FlyingBuf, FLYING_BYTE-1);
	FlyingBuf[17] = checkSum_Byte;
	for(int i=0; i<FLYING_BYTE;i++) {
	wiringPiI2CWriteReg(slave_id, FlyingBuf[i]);
	}*/
}

uint8_t checkSum(unsigned char *data, uint8_t len ) {
	uint16_t sum = 0;
	uint8_t nibble = 0;
	for(int i=0;i<len;i++) {
		sum += data[i];
	}
	nibble = sum >> 8;
	sum = (sum & 0xff) + nibble;
	return checkSum_Byte = ~sum;
} 