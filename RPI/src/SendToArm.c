
#include "SendToArm.h"

RPI rpi = { 0 };

#define STX 'A'
#define ETX 'B'
int slave_id;
uint8_t checkSum_Byte = 0;     
uint8_t ReadyToFly_Data[READY_TO_FLY_BYTE];
uint8_t Flying_Data[READY_TO_FLY_BYTE];
uint8_t checkSum(uint8_t *data, uint8_t len );
bool takeoff = 0;
char txbuf[1024];
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
}




 
void send_Arm_ReadyToFly() {
	 sprintf((char*)txbuf, "%c%c", STX, ETX);
	 wlen = write(slave_id, txbuf, strlen(txbuf));
	    if (wlen != strlen(txbuf)) {
	        printf("Error from write: %d, %d\n", wlen, errno);
	    }
    	 tcdrain(slave_id);    /* delay for output */

/*
	ReadyToFly_Data[0]  = dc0.pGain.integerL;
	ReadyToFly_Data[1]  = dc0.pGain.integerH;
	ReadyToFly_Data[2]  = dc0.pGain.decimalL;
	ReadyToFly_Data[3]  = dc0.pGain.decimalH;
	ReadyToFly_Data[4]  = dc0.iGain.integerL;
	ReadyToFly_Data[5]  = dc0.iGain.integerH;
	ReadyToFly_Data[6]  = dc0.iGain.decimalL;
	ReadyToFly_Data[7]  = dc0.iGain.decimalH;
	ReadyToFly_Data[8]  = dc0.dGain.integerL;
	ReadyToFly_Data[9]  = dc0.dGain.integerH;
	ReadyToFly_Data[10] = dc0.dGain.decimalL;
	ReadyToFly_Data[11] = dc0.dGain.decimalH;

	ReadyToFly_Data[12] = dc1.pGain.integerL;
	ReadyToFly_Data[13] = dc1.pGain.integerH;
	ReadyToFly_Data[14] = dc1.pGain.decimalL;
	ReadyToFly_Data[15] = dc1.pGain.decimalH;
	ReadyToFly_Data[16] = dc1.iGain.integerL;
	ReadyToFly_Data[17] = dc1.iGain.integerH;
	ReadyToFly_Data[18] = dc1.iGain.decimalL;
	ReadyToFly_Data[19] = dc1.iGain.decimalH;
	ReadyToFly_Data[20] = dc1.dGain.integerL;
	ReadyToFly_Data[21] = dc1.dGain.integerH;
	ReadyToFly_Data[22] = dc1.dGain.decimalL;
	ReadyToFly_Data[23] = dc1.dGain.decimalH;
	
	ReadyToFly_Data[24] = dc2.pGain.integerL;
	ReadyToFly_Data[25] = dc2.pGain.integerH;
	ReadyToFly_Data[26] = dc2.pGain.decimalL;
	ReadyToFly_Data[27] = dc2.pGain.decimalH;
	ReadyToFly_Data[28] = dc2.iGain.integerL;
	ReadyToFly_Data[29] = dc2.iGain.integerH;
	ReadyToFly_Data[30] = dc2.iGain.decimalL;
	ReadyToFly_Data[31] = dc2.iGain.decimalH;
	ReadyToFly_Data[32] = dc2.dGain.integerL;
	ReadyToFly_Data[33] = dc2.dGain.integerH;
	ReadyToFly_Data[34] = dc2.dGain.decimalL;
	ReadyToFly_Data[35] = dc2.dGain.decimalH;

	ReadyToFly_Data[36] = dc3.pGain.integerL;
	ReadyToFly_Data[37] = dc3.pGain.integerH;
	ReadyToFly_Data[38] = dc3.pGain.decimalL;
	ReadyToFly_Data[39] = dc3.pGain.decimalH;
	ReadyToFly_Data[40] = dc3.iGain.integerL;
	ReadyToFly_Data[41] = dc3.iGain.integerH;
	ReadyToFly_Data[42] = dc3.iGain.decimalL;
	ReadyToFly_Data[43] = dc3.iGain.decimalH;
	ReadyToFly_Data[44] = dc3.dGain.integerL;
	ReadyToFly_Data[45] = dc3.dGain.integerH;
	ReadyToFly_Data[46] = dc3.dGain.decimalL;
	ReadyToFly_Data[47] = dc3.dGain.decimalH;

	ReadyToFly_Data[48] = dc4.pGain.integerL;
	ReadyToFly_Data[49] = dc4.pGain.integerH;
	ReadyToFly_Data[50] = dc4.pGain.decimalL;
	ReadyToFly_Data[51] = dc4.pGain.decimalH;
	ReadyToFly_Data[52] = dc4.iGain.integerL;
	ReadyToFly_Data[53] = dc4.iGain.integerH;
	ReadyToFly_Data[54] = dc4.iGain.decimalL;
	ReadyToFly_Data[55] = dc4.iGain.decimalH;
	ReadyToFly_Data[56] = dc4.dGain.integerL;
	ReadyToFly_Data[57] = dc4.dGain.integerH;
	ReadyToFly_Data[58] = dc4.dGain.decimalL;
	ReadyToFly_Data[59] = dc4.dGain.decimalH;

	checkSum_Byte = checkSum(ReadyToFly_Data, READY_TO_FLY_BYTE-1);
	ReadyToFly_Data[60] = checkSum_Byte;

	for(int i=0; i<READY_TO_FLY_BYTE;i++) {
		wiringPiI2CWriteReg(slave_id, ReadyToFly_Data[i]);
	}
*/
}
void send_Arm_Flying() {
/*
	Flying_Data[0]  = roll.data.integerL;
	Flying_Data[1]  = roll.data.integerH;
	Flying_Data[2]  = roll.data.decimalL;
	Flying_Data[3]  = roll.data.decimalH;   
	Flying_Data[4]  = pitch.data.integerL;
	Flying_Data[5]  = pitch.data.integerH;
	Flying_Data[6]  = pitch.data.decimalL;
	Flying_Data[7]  = pitch.data.decimalH;   
	Flying_Data[8]  = yaw.data.integerL;
	Flying_Data[9]  = yaw.data.integerH;
	Flying_Data[10] = yaw.data.decimalL;
	Flying_Data[11] = yaw.data.decimalH;   
	Flying_Data[12] = roll.server;
	Flying_Data[13] = pitch.server;
	Flying_Data[14] = yaw.server;
	Flying_Data[15] = bldcSpeed;
	Flying_Data[16] = servo.mode;
	checkSum_Byte   = checkSum(Flying_Data, FLYING_BYTE-1);
	Flying_Data[17] = checkSum_Byte;
	for(int i=0; i<FLYING_BYTE;i++) {
		wiringPiI2CWriteReg(slave_id, Flying_Data[i]);
	}*/
}

uint8_t checkSum(uint8_t *data, uint8_t len ) {
	uint16_t sum = 0;
	uint8_t nibble = 0;
	for(int i=0;i<len;i++) {
		sum += data[i];
	}
	nibble = sum >> 8;
	sum = (sum & 0xff) + nibble;
	return checkSum_Byte = ~sum;
}
