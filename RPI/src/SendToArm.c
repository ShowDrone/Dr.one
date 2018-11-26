#include "SendToArm.h"

#define SEND_TO_ARM_BYTE 20
//#define SEND_TO_ARM_BYTE 56
uint8_t checkSum(unsigned char *data, uint8_t len );
void ARM_close();

RPI rpi = { 0 };

int slave_id;
uint8_t checkSum_Byte = 0;    
unsigned char SendToArmBuf[SEND_TO_ARM_BYTE];
bool takeoff = 0;
bool ArmState = 1;
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
	
	char *portname = "/dev/ttyUSB1";
	
	slave_id = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (slave_id < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	}
	
    	set_interface_attribs(slave_id, B115200);
	printf("ARM_Init Succeed\r\n");
}

void ARM_close() {
	close(slave_id);
}

 
void sendToArm() {
/*
		
	SendToArmBuf[0]  = dc0.pGain.integerL;
	SendToArmBuf[1]  = dc0.pGain.decimalL;
	SendToArmBuf[2]  = dc0.pGain.decimalH;
	SendToArmBuf[3]  = dc0.iGain.integerL;
	SendToArmBuf[4]  = dc0.iGain.decimalL;
	SendToArmBuf[5]  = dc0.iGain.decimalH;
	SendToArmBuf[6]  = dc0.dGain.integerL;
	SendToArmBuf[7]  = dc0.dGain.decimalL;
	SendToArmBuf[8]  = dc0.dGain.decimalH;
	SendToArmBuf[9]  = dc1.pGain.integerL;
	SendToArmBuf[10] = dc1.pGain.decimalL;
	SendToArmBuf[11] = dc1.pGain.decimalH;
	SendToArmBuf[12] = dc1.iGain.integerL;
	SendToArmBuf[13] = dc1.iGain.decimalL;
	SendToArmBuf[14] = dc1.iGain.decimalH;
	SendToArmBuf[15] = dc1.dGain.integerL;
	SendToArmBuf[16] = dc1.dGain.decimalL;
	SendToArmBuf[17] = dc1.dGain.decimalH;
	
	SendToArmBuf[18] = dc2.pGain.integerL;
	SendToArmBuf[19] = dc2.pGain.decimalL;
	SendToArmBuf[20] = dc2.pGain.decimalH;
	SendToArmBuf[21] = dc2.iGain.integerL;
	SendToArmBuf[22] = dc2.iGain.decimalL;
	SendToArmBuf[23] = dc2.iGain.decimalH;
	SendToArmBuf[24] = dc2.dGain.integerL;
	SendToArmBuf[25] = dc2.dGain.decimalL;
	SendToArmBuf[26] = dc2.dGain.decimalH;
	
	SendToArmBuf[27] = dc3.pGain.integerL;
	SendToArmBuf[28] = dc3.pGain.decimalL;
	SendToArmBuf[29] = dc3.pGain.decimalH;
	SendToArmBuf[30] = dc3.iGain.integerL;
	SendToArmBuf[31] = dc3.iGain.decimalL;
	SendToArmBuf[32] = dc3.iGain.decimalH;
	SendToArmBuf[33] = dc3.dGain.integerL;
	SendToArmBuf[34] = dc3.dGain.decimalL;
	SendToArmBuf[35] = dc3.dGain.decimalH;


	SendToArmBuf[36] = bl.pGain.integerL;
	SendToArmBuf[37] = bl.pGain.decimalL;
	SendToArmBuf[38] = bl.pGain.decimalH;
	SendToArmBuf[39] = bl.iGain.integerL;
	SendToArmBuf[40] = bl.iGain.decimalL;
	SendToArmBuf[41] = bl.iGain.decimalH;
	SendToArmBuf[42] = bl.dGain.integerL;
	SendToArmBuf[43] = bl.dGain.decimalL;
	SendToArmBuf[44] = bl.dGain.decimalH;


	SendToArmBuf[45] = roll.server;
	SendToArmBuf[46] = pitch.server;
	SendToArmBuf[47] = yaw.server;
	SendToArmBuf[48] = bldcSpeed;
	SendToArmBuf[49] = yaw.data.integerL;
	SendToArmBuf[50] = yaw.data.integerH;
	SendToArmBuf[51] = yaw.data.decimalL;
	SendToArmBuf[52] = yaw.data.decimalH;
	SendToArmBuf[53] = servo.y;
	SendToArmBuf[54] = ArmState;  
	checkSum_Byte = checkSum(SendToArmBuf, SEND_TO_ARM_BYTE-1);
	SendToArmBuf[55] = checkSum_Byte;
*/	
	SendToArmBuf[0] = bl.pGain.integerL;
	SendToArmBuf[1] = bl.pGain.decimalL;
	SendToArmBuf[2] = bl.pGain.decimalH;
	SendToArmBuf[3] = bl.iGain.integerL;
	SendToArmBuf[4] = bl.iGain.decimalL;
	SendToArmBuf[5] = bl.iGain.decimalH;
	SendToArmBuf[6] = bl.dGain.integerL;
	SendToArmBuf[7] = bl.dGain.decimalL;
	SendToArmBuf[8] = bl.dGain.decimalH;

	SendToArmBuf[9] = roll.server;
	SendToArmBuf[10] = pitch.server;
	SendToArmBuf[11] = yaw.server;
	SendToArmBuf[12] = bldcSpeed;
	SendToArmBuf[13] = yaw.data.integerL;
	SendToArmBuf[14] = yaw.data.integerH;
	SendToArmBuf[15] = yaw.data.decimalL;
	SendToArmBuf[16] = yaw.data.decimalH;
	SendToArmBuf[17] = servo.y;
	SendToArmBuf[18] = ArmState;  
	checkSum_Byte = checkSum(SendToArmBuf, SEND_TO_ARM_BYTE-1);
	SendToArmBuf[19] = checkSum_Byte;


	printf("%f %d %d %d\r\n", bl.p, SendToArmBuf[0],SendToArmBuf[1],SendToArmBuf[2]);
	wlen = write(slave_id, SendToArmBuf, SEND_TO_ARM_BYTE);
	if (wlen != SEND_TO_ARM_BYTE) {
	  printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(slave_id);    /* delay for output */

}

void setZeroProp() {
	ArmState = 0; 

	SendToArmBuf[9] = 45;
	SendToArmBuf[10] = 45;
	SendToArmBuf[11] = 45;
	SendToArmBuf[12] = 0;;
	SendToArmBuf[17] = 0;
	SendToArmBuf[18] = ArmState;

	/*SendToArmBuf[45] = 45;
	SendToArmBuf[46] = 45;
	SendToArmBuf[47] = 45;
	SendToArmBuf[48] = 0;
	SendToArmBuf[48] = 0;
	SendToArmBuf[53] = ArmState;*/
	wlen = write(slave_id, SendToArmBuf, SEND_TO_ARM_BYTE);
	if (wlen != SEND_TO_ARM_BYTE) {
	  printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(slave_id);    /* delay for output */

	
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