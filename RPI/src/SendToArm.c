
#include "SendToArm.h"

RPI rpi = { 0 };

uint8_t slave_id;
uint8_t checkSum_Byte = 0;     
uint8_t ReadyToFly_Data[READY_TO_FLY_BYTE];
uint8_t Flying_Data[READY_TO_FLY_BYTE];

void setSlaveMCU (int _id) {
	do {                                                                                                                                      
  		slave_id = wiringPiI2CSetup(_id);                                                                                                                       
		delay(500);                                                                                                                 
	} while(slave_id == -1);
	printf("Slave Check\n");
}

 
void send_Arm_ReadyToFly() {
	ReadyToFly_Data[0]  = rollX.pGain.integerL;
	ReadyToFly_Data[1]  = rollX.pGain.integerH;
	ReadyToFly_Data[2]  = rollX.pGain.decimalL;
	ReadyToFly_Data[3]  = rollX.pGain.decimalH;
	ReadyToFly_Data[4]  = rollX.iGain.integerL;
	ReadyToFly_Data[5]  = rollX.iGain.integerH;
	ReadyToFly_Data[6]  = rollX.iGain.decimalL;
	ReadyToFly_Data[7]  = rollX.iGain.decimalH;
	ReadyToFly_Data[8]  = rollX.dGain.integerL;
	ReadyToFly_Data[9]  = rollX.dGain.integerH;
	ReadyToFly_Data[10] = rollX.dGain.decimalL;
	ReadyToFly_Data[11] = rollX.dGain.decimalH;

	ReadyToFly_Data[12] = pitchY.pGain.integerL;
	ReadyToFly_Data[13] = pitchY.pGain.integerH;
	ReadyToFly_Data[14] = pitchY.pGain.decimalL;
	ReadyToFly_Data[15] = pitchY.pGain.decimalH;
	ReadyToFly_Data[16] = pitchY.iGain.integerL;
	ReadyToFly_Data[17] = pitchY.iGain.integerH;
	ReadyToFly_Data[18] = pitchY.iGain.decimalL;
	ReadyToFly_Data[19] = pitchY.iGain.decimalH;
	ReadyToFly_Data[20] = pitchY.dGain.integerL;
	ReadyToFly_Data[21] = pitchY.dGain.integerH;
	ReadyToFly_Data[22] = pitchY.dGain.decimalL;
	ReadyToFly_Data[23] = pitchY.dGain.decimalH;
	
	ReadyToFly_Data[24] = yawZ.pGain.integerL;
	ReadyToFly_Data[25] = yawZ.pGain.integerH;
	ReadyToFly_Data[26] = yawZ.pGain.decimalL;
	ReadyToFly_Data[27] = yawZ.pGain.decimalH;
	ReadyToFly_Data[28] = yawZ.iGain.integerL;
	ReadyToFly_Data[29] = yawZ.iGain.integerH;
	ReadyToFly_Data[30] = yawZ.iGain.decimalL;
	ReadyToFly_Data[31] = yawZ.iGain.decimalH;
	ReadyToFly_Data[32] = yawZ.dGain.integerL;
	ReadyToFly_Data[33] = yawZ.dGain.integerH;
	ReadyToFly_Data[34] = yawZ.dGain.decimalL;
	ReadyToFly_Data[35] = yawZ.dGain.decimalH;

	ReadyToFly_Data[36] = roll.server;
	ReadyToFly_Data[37] = pitch.server;
	ReadyToFly_Data[38] = yaw.server;
	checkSum_Byte = checkSum(ReadyToFly_Data, READY_TO_FLY_BYTE-1);
	ReadyToFly_Data[39] = checkSum_Byte;

	for(int i=0; i<READY_TO_FLY_BYTE;i++) {
		wiringPiI2CWriteReg(slave_id, ReadyToFly_Data[i]);
	}
}

void send_Arm_Flying() {
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
	}
}

uint8_t checkSum(uint8_t *data, uin8_t len ) {
	uint16_t sum = 0;
	uint8_t nibble = 0;
	for(int i=0;i<len;i++) {
		sum += data[i];
	}
	nibble = sum >> 8;
	sum = (sum & 0xff) + nibble;
	return checkSum_Byte = ~sum + 1;
}