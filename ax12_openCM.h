/*
 * Dynamixel.h
 *
 *  Created on: 2013. 11. 8.
 *      Author: in2storm
 *      brief : 2013-11-12 [ROBOTIS] revised for OpenCM board
 */

#ifndef AX12_OPENCM_H_
#define AX12_OPENCM_H_

//#include "dxl.h"
#include <stdio.h>
#include <sys/types.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "debug.h"



typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef signed char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

#define word			uint16
#define byte			uint8
#define dword			uint32

#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

#define DXL_PACKET_TYPE1	1 //2014-04-08 sm6787@robotis.com ->  DXL protocol 1.0 packet type
#define DXL_PACKET_TYPE2	2

#define BROADCAST_ID		(254) /* 0xFE */

/*
 * defines error message for protocol 1.0
 * */
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

/*
 * defines error message for protocol 2.0
 * */
#define ERRBIT_RESULT_FAIL	(1)
#define ERRBIT_INST_ERROR	(2)
#define ERRBIT_CRC			(4)
#define ERRBIT_DATA_RANGE	(8)
#define ERRBIT_DATA_LENGTH	(16)
#define ERRBIT_DATA_LIMIT	(32)
#define ERRBIT_ACCESS		(64)

/*
 * defines message of communication
 * */
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)

/* timing defines */
#define RX_TIMEOUT_COUNT2		(1600L) //(1000L) //porting
#define NANO_TIME_DELAY			(12000) //ydh added 20111228 -> 20120210 edited ydh
//#define RX_TIMEOUT_COUNT1  	(RX_TIMEOUT_COUNT2*90L)// -> 110msec  before Ver 1.11e
#define RX_TIMEOUT_COUNT1  		(RX_TIMEOUT_COUNT2*128L)//  -> 156msec for ID 110 safe access after Ver 1.11f //porting ydh


/*
 * Dynamixel common control table -> see ROBOTIS support page(support.robotis.com)
 * 
 * 
 * */
#define P_MODEL_NUMBER_L      0
#define P_MODOEL_NUMBER_H     1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)

#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)

#define P_P_GAIN  			(26)
#define P_I_GAIN 			(27)
#define P_D_GAIN   			(28)
#define P_RESERVED  		(29)


#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
//#define P_FINAL_GOAL_POSITION_L (46)
#define P_MOVING (46)

#define DXL_RX_BUF_SIZE 0x3FF
#define DXL_PARAMETER_BUF_SIZE 128

enum DXL_INSTRUCTION{  //2014-04-02 ROBOTIS DXL Protocol 2.0
	INST_PING           = 1,
	INST_READ           = 2,
	INST_WRITE          = 3,
	INST_REG_WRITE      = 4,
	INST_ACTION         = 5,
	INST_FACTORY_RESET  = 6,
	INST_REBOOT         = 8,
	INST_SYSTEM_WRITE   = 13,   // 0x0D
	INST_STATUS         = 85,   // 0x55
	INST_SYNC_READ      = 130,  // 0x82
	INST_SYNC_WRITE     = 131,  // 0x83
	INST_BULK_READ      = 146,  // 0x92
	INST_BULK_WRITE     = 147   // 0x93
};

#define mDirPin 1

/*
typedef struct data {
    int             iID;
    int				iAddr;
    int             iLength;
    int             iError;
    byte  			iData[8];
} BulkData, *PBulkData;

*/

class Dynamixel {
public:
	Dynamixel(void);
	virtual ~Dynamixel();

	/////////// Device control methods /////////////
	int begin(int buad);
	//uint8 readRaw(void);
	uint8 available(void);
	//void writeRaw(uint8);

	byte getResult(void); // use getTxRxStatus() instead of getResult() method
	byte getTxRxStatus(void);// made by NaN (Robotsource.org)
	byte getError(byte errbit);
	byte setLibStatusReturnLevel(byte); // made by NaN (Robotsource.org)
	byte setLibNumberTxRxAttempts(byte);// made by NaN (Robotsource.org)

	byte txRxPacket(byte bID, byte bInst, int bTxParaLen);
	byte txPacket(byte bID, byte bInstruction, int bParameterLength);
	byte rxPacket(int bRxLength);

	void setPacketType(byte ver);
	byte getPacketType(void);

	word  ping(byte  bID);
	uint32  ping(void); //Broadcast ping in DXL 2.0 protocol

	//// High communication methods ////////
	byte readByte(byte bID, word bAddress);
	byte writeByte(byte bID, word bAddress, byte bData);
	word readWord(byte bID, word bAddress);
	byte writeWord(byte bID, word bAddress, word wData);

	byte writeDword( byte bID, word wAddress, uint32 value );
	uint32 readDword( byte bID, word wAddress );

	byte setPosition(byte ServoID, int Position, int Speed);
	byte syncWrite(int start_addr, byte data_length, int *param, int param_length);// DWORD(32bit) syncwrite() for DXL PRO
	byte syncWrite(int start_addr, int data_length, word *param, int param_length); // WORD(16bit) syncwrite() for DXL


	/////// Methods for making a packet ////////
	void setTxPacketId( byte id );
	void setTxPacketInstruction( byte instruction );
	void setTxPacketParameter( byte index, byte value );
	void setTxPacketLength( byte length );
	byte txrxPacket(void);
	int getRxPacketParameter( int index );
	int getRxPacketLength(void);

	 /*
	     * Dynamixel Pro Bulk Read & utility functions
	     * */
	/*int bulkRead(byte *param, int param_length);
	int bulkRead(word *param, int param_length); //new
	int bulkWrite(byte *param, int param_length);

    byte getBulkByte(int id, int addr);
	uint16 getBulkWord(int id, int addr);
	int getBulkDword(int id,int addr);
*/

	//Easy Functions for DXL
	word getModelNumber(byte bID);

	void setID(byte current_ID, byte new_ID);
	void setBaud(byte bID, byte baud_num);

	void returnLevel(byte bID, byte level);
	byte returnLevel(byte bID);

	void returnDelayTime(byte bID, byte time);
	byte returnDelayTime(byte bID);

	void alarmShutdown(byte bID,byte option);
	byte alarmShutdown(byte bID);

	void controlMode(byte bID, byte mode); // change wheel, joint
	byte controlMode(byte bID); // return current mode

	void jointMode(byte bID);
	void wheelMode(byte bID);

	void maxTorque(byte bID, word value);
	word maxTorque(byte bID);

	void maxVolt(byte bID, byte value);
	byte maxVolt(byte bID);

	void minVolt(byte bID, byte value);
	byte minVolt(byte bID);

	void maxTemperature(byte bID, byte temp);
	byte maxTemperature(byte bID);

	void torqueEnable(byte bID);
	void torqueDisable(byte bID);

	void cwAngleLimit(byte bID, word angle);
	word cwAngleLimit(byte bID);
	void ccwAngleLimit(byte bID, word angle);
	word ccwAngleLimit(byte bID);

	void goalPosition(byte bID, int position);
	void goalSpeed(byte bID, int speed);
	void goalTorque(byte bID, int torque);

	int getPosition(byte bID);
	int getSpeed(byte bID);
	int getLoad(byte bID);
	int getVolt(byte bID);
	byte getTemperature(byte bID);
	byte isMoving(byte bID);

	void ledOn(byte bID);
	void ledOn(byte bID, byte option);  //for XL-320 , DXL PRO
	void ledOff(byte bID);

	void setPID(byte bID, byte propotional, byte integral, byte derivative);

	void complianceMargin(byte bID, byte CW, byte CCW);
	void complianceSlope(byte bID, byte CW, byte CCW);



	void cwTurn(byte bID, word speed); //cwTurn()으로 변경
	void ccwTurn(byte bID, word speed);//ccwTurn()
	//int getRxPacketError( byte errbit );
	/*
	 * New Methods for making a packet
	 * you can make sync write packet and reg/action packet by using these methods
	 */
	//byte syncWrite(byte start_addr, byte num_of_data, byte *param, int array_length);
	//

	/*
	 * New Methods for making a packet
	 *
	 */
	void initPacket(byte bID, byte bInst);
	void pushByte(byte value);
	void pushParam(int value);
	void pushParam(byte value);
	byte flushPacket(void);
	byte getPacketLength(void);

	/*
	 * Utility methods for Dynamixel
	 */

	/*byte getLowByte( word wData ); //can be replaced by DXL_LOBYTE(w)
	byte getHighByte( word wData );//can be replaced by DXL_HIBYTE(w)
	word makeWord( byte lowbyte, byte highbyte ); //can be replaced by DXL_MAKEWORD(w)*/

private:
	void printBuffer(byte *bpPrintBuffer, byte bLength);
	uint32 Dummy(uint32 tmp);
	void uDelay(uint32 uTime);
	void nDelay(uint32 nTime);
	void dxlTxEnable(void);
	void dxlTxDisable(void);
	void clearBuffer(void);
	byte checkPacketType(void);

	uint8 setDxlLibStatRtnLvl(uint8); // inspired by NaN (robotsource.org)
	uint8 setDxlLibNumTries(uint8); // inspired by NaN (robotsource.org)

	// dxl_dev *mDxlDevice;
	// usart_dev *mDxlUsart;  /*< USART Device*/
	// gpio_dev *mTxPort;
	// gpio_dev *mRxPort;
	// gpio_dev *mDirPort;
	uint8 mTxPin;
	uint8 mRxPin;
    int fd;
	//int mDirPin;
	//uint8 mTxDEnablePin;  //CM-900 ES only
	//uint8 mRxDEnablePin;  //CM-900 ES only
	uint8 mRxBuffer[DXL_RX_BUF_SIZE];
	uint8 mTxBuffer[DXL_RX_BUF_SIZE];
	uint8 mParamBuffer[DXL_PARAMETER_BUF_SIZE];
	uint8 mBusUsed;
	uint8 mRxLength;  // the length of the received data from dynamixel bus

	// additions to return proper COMM_* status
	uint8 mDXLtxrxStatus;  // inspired by NaN (robotsource.org)
	// additions to permit non-default Status Return Level settings without returning errors
	uint8 gbDXLStatusReturnLevel;
	// additions to adjust number of txrx attempts
	uint8 gbDXLNumberTxRxAttempts;

	uint8 mPacketType;  //2014-04-02

	byte mPktIdIndex;
	byte mPktLengthIndex;
	byte mPktInstIndex;
	byte mPktErrorIndex;
	//byte mRxLengthOffset;

	byte mbLengthForPacketMaking;
	byte mbIDForPacketMaking;
	byte mbInstructionForPacketMaking;
	byte mCommStatus;

	byte SmartDelayFlag;
	//BulkData mBulkData[32]; //Maximum dxl pro number is 32
};


#endif /* DYNAMIXEL_H_ */
