/*
 * Dynamixel.cpp
 *
 *  Created on: 2013. 11. 8.
 *      Author: in2storm
 */

#include "ax12_openCM.h"
//#include "Arduino-compatibles.h"
//#include "dxl.h"

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {0x0000,
	                                0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	                                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	                                0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	                                0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	                                0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	                                0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	                                0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	                                0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	                                0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	                                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	                                0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	                                0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	                                0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	                                0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	                                0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	                                0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	                                0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	                                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	                                0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	                                0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	                                0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	                                0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	                                0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	                                0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	                                0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	                                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	                                0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	                                0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	                                0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	                                0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	                                0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	                                0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	                                0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	                                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	                                0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	                                0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	                                0x820D, 0x8207, 0x0202 };

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}


Dynamixel::Dynamixel(void) {
	// TODO Auto-generated constructor stub
    wiringPiSetup();
}

Dynamixel::~Dynamixel() {
	// TODO Auto-generated destructor stub
}
int Dynamixel::begin(int baud){

	uint32 Baudrate = 0;
	mPacketType = DXL_PACKET_TYPE1; //2014-04-02 default packet type is 1.0 ->  set as 1

	//Initialize USART 1 device
	 //usart_init(mDxlUsart);
	 //Calculate baudrate, refer to ROBOTIS support page.
	 //Baudrate = dxl_get_baudrate(baud);  //Dxl 2.0
	 //int fd;
	
	if((fd = serialOpen ("/dev/ttyAMA0",baud))<0)
    {
        printf("serial err\n");
        return -1;
    }
	pinMode(mDirPin, OUTPUT);
    digitalWrite(mDirPin, LOW);
	delay(100);
	mDXLtxrxStatus = 0;
	mBusUsed = 0;// only 1 when tx/rx is operated
	//gbIsDynmixelUsed = 1;  //[ROBOTIS]2012-12-13 to notify end of using dynamixel SDK to uart.c

	this->setLibStatusReturnLevel(2);
	this->setLibNumberTxRxAttempts(2);

	this->clearBuffer();
	if(this->checkPacketType()){ // Dxl 2.0
		this->setPacketType(DXL_PACKET_TYPE2);
	}else{           // Dxl 1.0
		this->setPacketType(DXL_PACKET_TYPE1);
	}
	this->setLibNumberTxRxAttempts(1);
	this->clearBuffer();
    
    return fd;
}
/*
 *  [ROBOTIS][ADD][START] 2013-04-09 support read and write on dxl bus
 * */

// byte Dynamixel::readRaw(void){
	// return mDxlDevice->data_buffer[(mDxlDevice->read_pointer)++ & DXL_RX_BUF_SIZE];
// }
// void Dynamixel::writeRaw(uint8 value){

	// dxlTxEnable();

	// TxByteToDXL(value);
	// mDxlUsart->regs->DR = (value & (u16)0x01FF);
	// while( (mDxlUsart->regs->SR & ((u16)0x0040)) == RESET );

	// dxlTxDisable();
	// DXL_RXD
// }

/*
 * @brief : if data coming from dxl bus, returns 1, or if not, returns 0.
 *
 */
byte Dynamixel::available(void){
		return serialDataAvail(fd) ;
}
void Dynamixel::dxlTxEnable(void){
	if(mDirPin == 0)
		return;
    digitalWrite(mDirPin, HIGH);
    usleep(1000);
	// TX Enable

}
void Dynamixel::dxlTxDisable(void){
	if(mDirPin == 0)
		return;
    digitalWrite(mDirPin, LOW);
    usleep(1000);
	// RX Enable
}
void Dynamixel::clearBuffer(void){
	//mDxlDevice->read_pointer = 0;
	//mDxlDevice->write_pointer = 0;

}
void Dynamixel::setPacketType(byte type){
	mPacketType = type;
	if(mPacketType == DXL_PACKET_TYPE2){ // Dxl 2.0
		mPktIdIndex = 4;
		mPktLengthIndex = 5;
		mPktInstIndex = 7;
		mPktErrorIndex = 8;
		//mRxLengthOffset = 7;
	}else{           // Dxl 1.0
		mPktIdIndex = 2;
		mPktLengthIndex = 3;
		mPktInstIndex = 4;
		mPktErrorIndex = 4;
		//mRxLengthOffset = 4;
	}
}
byte Dynamixel::getPacketType(void){
	return mPacketType;
}
byte Dynamixel::checkPacketType(void){
	this->setPacketType(DXL_PACKET_TYPE2);
	//TxDStringC("Check DXL Ver\r\n");
	if(this->txRxPacket(0xFE, INST_PING, 0)){
		if(mRxBuffer[0] == 0xff && mRxBuffer[1] == 0xff && mRxBuffer[2] == 0xfd){
			//TxDStringC("DXL 2.0 Detected!\r\n");
			return 1; //Dxl 2.0
		}else{
			//TxDStringC("DXL 1.0 Detected!\r\n");
			return 0;  //Dxl 1.0
		}
	}
	return 0;//default is 1.0 protocol
}


byte Dynamixel::setLibStatusReturnLevel(byte num)
{
	gbDXLStatusReturnLevel = num;
	return gbDXLStatusReturnLevel;
}

byte Dynamixel::setLibNumberTxRxAttempts(byte num)
{
	gbDXLNumberTxRxAttempts = num;
	return gbDXLNumberTxRxAttempts;
}
/*
 * return value for getTxRxStatus(), getResult();
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)
*/
byte Dynamixel::getTxRxStatus(void) // made by NaN (Robotsource.org)
{
	return mDXLtxrxStatus;
}
/*
 * Use getTxRxStatus() instead of getResult()
 * */
byte  Dynamixel::getResult(void){
	//	return mCommStatus;
	return this->getTxRxStatus();
}
/*
 *  ERROR Bit table is below.

//DXL 1.0 protocol
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

//DXL 2.0 protocol
#define ERRBIT_RESULT_FAIL	(1)
#define ERRBIT_INST_ERROR	(2)
#define ERRBIT_CRC			(4)
#define ERRBIT_DATA_RANGE	(8)
#define ERRBIT_DATA_LENGTH	(16)
#define ERRBIT_DATA_LIMIT	(32)
#define ERRBIT_ACCESS		(64)
 * */
byte Dynamixel::getError( byte errbit ){
	//return dxl_get_rxpacket_error( errbit );
	if(mPacketType == DXL_PACKET_TYPE1){
		if( mRxBuffer[4] & errbit ) return 1;
		else return 0;

	}else{
		if( mRxBuffer[8] & errbit ) return 1;
		else return 0;


	}

}

byte Dynamixel::txPacket(byte bID, byte bInstruction, int bParameterLength){

    word bCount,bCheckSum,bPacketLength;
	
	//byte bCount,bCheckSum,bPacketLength; // modified 20151127 jason ver 1.03

    byte offsetParamIndex;
    if(mPacketType == DXL_PACKET_TYPE1){//dxl protocol 1.0
		mTxBuffer[0] = 0xff;
		mTxBuffer[1] = 0xff;
		mTxBuffer[2] = bID;
		mTxBuffer[3] = bParameterLength+2; //2(byte) <- instruction(1byte) + checksum(1byte)
		mTxBuffer[4] = bInstruction;

		offsetParamIndex = 5;
		bPacketLength = bParameterLength+2+4;

    }else{ //dxl protocol 2.0
    	mTxBuffer[0] = 0xff;
    	mTxBuffer[1] = 0xff;
    	mTxBuffer[2] = 0xfd;
    	mTxBuffer[3] = 0x00;
    	mTxBuffer[4] = bID;
    	//get parameter length
    	mTxBuffer[5] = DXL_LOBYTE(bParameterLength+3);// 3(byte) <- instruction(1byte) + checksum(2byte)
    	mTxBuffer[6] = DXL_HIBYTE(bParameterLength+3);
    	mTxBuffer[7] = bInstruction;

    	offsetParamIndex = 8;
    	bPacketLength = bParameterLength+3+7; //parameter length 3bytes, 7bytes =  packet header 4bytes, ID 1byte,  length 2bytes
    }

    //copy parameters from mParamBuffer to mTxBuffer
    for(bCount = 0; bCount < bParameterLength; bCount++)
    {
    	mTxBuffer[bCount+offsetParamIndex] = mParamBuffer[bCount];
    }

    if(mPacketType == DXL_PACKET_TYPE1){
        bCheckSum = 0;
    	for(bCount = 2; bCount < bPacketLength-1; bCount++){ //except 0xff,checksum
			bCheckSum += mTxBuffer[bCount];
		}
    	mTxBuffer[bCount] = ~bCheckSum; //Writing Checksum with Bit Inversion
    }else{
    	bCheckSum = update_crc(0, mTxBuffer, bPacketLength-2);  // -2 : except CRC16
    	mTxBuffer[bPacketLength-2] = DXL_LOBYTE(bCheckSum);     // last - 2
   	    mTxBuffer[bPacketLength-1] = DXL_HIBYTE(bCheckSum);     // last - 1
    }
    this->dxlTxEnable(); // this define is declared in dxl.h


    for(bCount = 0; bCount < bPacketLength; bCount++)
    {
        //writeRaw(mTxBuffer[bCount]);
        serialPutchar(fd,mTxBuffer[bCount]);
    }
    usleep(20);
    this->dxlTxDisable();// this define is declared in dxl.h
    //usleep(20);

    int nbytes;
    nbytes = serialDataAvail(fd);
    if (nbytes > 0)
    {
        memset(mRxBuffer,0,DXL_RX_BUF_SIZE);
        for (int i = 0;i<nbytes;i++)
        {
            mRxBuffer[i] = serialGetchar(fd);
            DEBUG(8,"serialDataAvail,current [%03d] byte is %02x\n",i,mRxBuffer[i]);
        }
    }
    else
        DEBUG(8,"no data recevied!\n");
    
    return(bPacketLength); // return packet length
}
byte Dynamixel::rxPacket(int bRxLength){
	unsigned long ulCounter, ulTimeLimit;
	word bCount, bLength, bChecksum;
	//byte bCount, bLength, bChecksum; // modified 20151217 jason ver 1.03

	byte bTimeout;

	bTimeout = 0;
	if(bRxLength == 255 || bRxLength == 0xffff) //2014-04-03
		ulTimeLimit = RX_TIMEOUT_COUNT1;
	else
		ulTimeLimit = RX_TIMEOUT_COUNT2;
    #if 0
    DEBUG(1,"rx %d delay begain !\n",bRxLength);
	for(bCount = 0; bCount < bRxLength; bCount++)
	{
		ulCounter = 0;
		//while(mDxlDevice->read_pointer == mDxlDevice->write_pointer)
		while(1)
		{
			nDelay(NANO_TIME_DELAY); //[ROBOTIS] porting ydh
			if(ulCounter++ > ulTimeLimit)
			{
				bTimeout = 1;
				break;
			}
			uDelay(0); //[ROBOTIS] porting ydh added  //if exist DXL 1.0 -> ok DXL 2.0 -> ok, if not exist 1.0 not ok, 2.0 ok
		}
		if(bTimeout) break;
		//mRxBuffer[bCount] = mDxlDevice->data_buffer[mDxlDevice->read_pointer++ & DXL_RX_BUF_SIZE]; // get packet data from USART device
		//TxDStringC("mRxBuffer = ");TxDHex8C(mRxBuffer[bCount]);TxDStringC("\r\n");
	}
    DEBUG(1,"rx delay end !\n");
    #endif

	bLength = bCount;
	bChecksum = 0;
	if( mTxBuffer[mPktIdIndex] != BROADCAST_ID )
	{
		if(bTimeout && bRxLength != 255)
		{
			mDXLtxrxStatus |= (1<<COMM_RXTIMEOUT);
			clearBuffer();
			return 0;
		}
		if(bLength > 3) //checking available length.
		{
			if(mPacketType == 1){  //Dxl 1.0 header check
				if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff ){
					mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
					clearBuffer();
					return 0;
				}
			}else{// Dxl 2.0 header check
				if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xfd)
				{
					mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
					clearBuffer();
					return 0;
				}
			}

			if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] )  //id check
			{
				mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
				clearBuffer();
				return 0;
			}

			if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) // status packet length check
			{
				mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
				clearBuffer();
				return 0;
			}

			if(mPacketType == 1 && mRxBuffer[mPktErrorIndex] != 0){										//140512 shin
				
			}else{																					//140512 shin

			}

			if(mPacketType == 1){ // Dxl 1.0 checksum
				for(bCount = 2; bCount < bLength; bCount++){
					bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
				}

				bChecksum &= 0xff; // added 20151217 jason ver 1.04

				if(bChecksum != 0xff)
				{
					mDXLtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
					clearBuffer();
					return 0;
				}
			}else{ // Dxl 2.0 checksum
				bChecksum = DXL_MAKEWORD(mRxBuffer[bRxLength-2], mRxBuffer[bRxLength-1]);
				if(update_crc(0, mRxBuffer, bRxLength-2) == bChecksum){ // -2 : except CRC16
					return bLength;
				}
				else{
					return 0;
				}
			}//end of checksum
		}//(bLength > 3)
	}//end of Rx status packet check

	return bLength;
}
void Dynamixel::printBuffer(byte *bpPrintBuffer, byte bLength)
{

}

byte Dynamixel::txRxPacket(byte bID, byte bInst, int bTxParaLen){

	mDXLtxrxStatus = 0;

	word bTxLen, bRxLenEx, bTryCount;

	mBusUsed = 1;
	mRxLength = bRxLenEx = bTxLen = 0;

	for(bTryCount = 0; bTryCount < gbDXLNumberTxRxAttempts; bTryCount++)//for(bTryCount = 0; bTryCount < TRY_NUM; bTryCount++)
	{
		//gbDXLReadPointer = gbDXLWritePointer;
		//mDxlDevice->read_pointer = mDxlDevice->write_pointer;//[ROBOTIS]BufferClear050728
		/**************************************   Transfer packet  ***************************************************/
		bTxLen = this->txPacket(bID, bInst, bTxParaLen);

		if(mPacketType == DXL_PACKET_TYPE1){ //Dxl 1.0 Tx success ?
			if (bTxLen == (bTxParaLen+4+2))	mDXLtxrxStatus = (1<<COMM_TXSUCCESS);
		}else{ //Dxl 2.0 Tx success?
			if (bTxLen == (bTxParaLen+3+7))	mDXLtxrxStatus = (1<<COMM_TXSUCCESS);
		}
		if(bInst == INST_PING){
			if(mPacketType == DXL_PACKET_TYPE1){ //Dxl 1.0
				if(bID == BROADCAST_ID)	mRxLength = bRxLenEx = 0xff;
				else mRxLength = bRxLenEx = 6; // basic response packet length
			}else{  //Dxl 2.0
				if(bID == BROADCAST_ID)	mRxLength = bRxLenEx = 0xffff;
				else mRxLength = bRxLenEx = 14;
			}

		}
		else if(bInst == INST_READ){
			if (gbDXLStatusReturnLevel > 0){
				if(mPacketType == DXL_PACKET_TYPE1) mRxLength = bRxLenEx = 6+mParamBuffer[1];
				else mRxLength = bRxLenEx = 11+DXL_MAKEWORD(mParamBuffer[2], mParamBuffer[3]);
			}
			else{
				mRxLength = bRxLenEx = 0;
			}

		}
		else if( bID == BROADCAST_ID ){
			if(bInst == INST_SYNC_READ || bInst == INST_BULK_READ) mRxLength = bRxLenEx = 0xffff; //only 2.0 case
			else mRxLength = bRxLenEx = 0; // no response packet
		}
		else{
			if (gbDXLStatusReturnLevel>1){
				if(mPacketType == DXL_PACKET_TYPE1) mRxLength = bRxLenEx = 6;//+mParamBuffer[1];
				else mRxLength = bRxLenEx = 11;
			}
			else{
				mRxLength = bRxLenEx = 0;
			}
		}


		if(bRxLenEx){
			if(SmartDelayFlag == 1)
            {
                DEBUG(1,"delay begain\n");
				delay(150);
                DEBUG(1,"delay end\n");
            }
			/**************************************   Receive packet  ***************************************************/
			mRxLength = this->rxPacket(bRxLenEx);

		}//bRxLenEx is exist
	} //for() gbDXLNumberTxRxAttempts

	//TxDStringC("\r\n TEST POINT 2");//TxDString("\r\n Err ID:0x");
	mBusUsed = 0;

	if((mRxLength != bRxLenEx) && (mTxBuffer[mPktIdIndex] != BROADCAST_ID))
	{
		return 0;
	}else if((mRxLength == 0) && (mTxBuffer[mPktInstIndex] == INST_PING)){  //[ROBOTIS] 2013-11-22 correct response for ping instruction
		//return 0;
	}
	mDXLtxrxStatus = (1<<COMM_RXSUCCESS);

	//gbLengthForPacketMaking =0;
	return 1;
}
uint32 Dynamixel::Dummy(uint32 tmp){
	return tmp;
}
void Dynamixel::uDelay(uint32 uTime){
	uint32 cnt, max;
		static uint32 tmp = 0;

		for( max=0; max < uTime; max++)
		{
			for( cnt=0; cnt < 10 ; cnt++ )
			{
				tmp +=Dummy(cnt);
			}
		}
		//tmpdly = tmp;
}
void Dynamixel::nDelay(uint32 nTime){
	uint32 cnt, max;
		cnt=0;
		static uint32 tmp = 0;

		for( max=0; max < nTime; max++)
		{
			//for( cnt=0; cnt < 10 ; cnt++ )
			//{
				tmp +=Dummy(cnt);
			//}
		}
		//tmpdly = tmp;
}


word  Dynamixel::ping(byte  bID ){

	if(this->txRxPacket(bID, INST_PING, 0)){
		if(mPacketType == DXL_PACKET_TYPE1) return (mRxBuffer[2]); //1.0
		else return DXL_MAKEWORD(mRxBuffer[9],mRxBuffer[10]); //return product code when 2.0
	}else{
		return 0xffff;  //no dxl in bus.
	}

}
/*
 * Broadcast ping for DXL 2.0 protocol
 * return : bit set each dynaxmel on bus. but it is limit to 32 DXLs
 * */
uint32 Dynamixel::ping(void){
	int i=0;
	uint32 result=0;
	if(mPacketType == DXL_PACKET_TYPE1) return 0xFFFFFFFF;  //cannot be used in 1.0 protocol
	if(this->txRxPacket(BROADCAST_ID, INST_PING, 0)){
		for(i=0; i < 32*14; i+=14){
			if(mRxBuffer[i] == 0xFF && mRxBuffer[i+1] == 0xFF && mRxBuffer[i+2] == 0xFD){
				result |= 1UL << mRxBuffer[i+4];
			    //TxDStringC("result = ");TxDHex32C(result);TxDStringC("\r\n");
			}
		}
		return result;
	}else{
		return 0xFFFFFFFF;  //no dxl in bus.
	}
}
byte  Dynamixel::writeByte(byte bID, word bAddress, byte bData){
	byte param_length = 0;
	if(mPacketType == DXL_PACKET_TYPE1){
		mParamBuffer[0] = bAddress;
		mParamBuffer[1] = bData;
		param_length = 2;
	}else{
		//insert wAddress to parameter bucket
		mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
		mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
		//insert data to parameter bucket
		mParamBuffer[2]	= bData;
		param_length = 3;
	}
	return this->txRxPacket(bID, INST_WRITE, param_length);
}

byte Dynamixel::readByte(byte bID, word bAddress){
	this->clearBuffer();
	if(mPacketType == DXL_PACKET_TYPE1){
		mParamBuffer[0] = bAddress;
		mParamBuffer[1] = 1;
		if( this->txRxPacket(bID, INST_READ, 2 )){
			//mCommStatus = 1;
			return(mRxBuffer[5]); //refer to 1.0 packet structure
		}
		else{
			//mCommStatus = 0;
			return 0xff;
		}
	}else{
		mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
		mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
		mParamBuffer[2]	= 1; //1byte
		mParamBuffer[3]	= 0;
		if( this->txRxPacket(bID, INST_READ, 4 )){
			return(mRxBuffer[9]);//refer to 2.0 packet structure
		}
		else{
			return 0xff;
		}
	}
}



byte Dynamixel::writeWord(byte bID, word bAddress, word wData){
    byte param_length = 0;
    this->clearBuffer();
	if(mPacketType == DXL_PACKET_TYPE1){
		mParamBuffer[0] = bAddress;
	    mParamBuffer[1] = DXL_LOBYTE(wData);//(byte)(wData&0xff);
	    mParamBuffer[2] = DXL_HIBYTE(wData);//(byte)((wData>>8)&0xff);
	    param_length = 3;
	}else{
		mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
		mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
		//insert data to parameter bucket
		mParamBuffer[2]	= DXL_LOBYTE(wData);
		mParamBuffer[3]	= DXL_HIBYTE(wData);
		param_length = 4;

	}
	return this->txRxPacket(bID, INST_WRITE, param_length);

}



word Dynamixel::readWord(byte bID, word bAddress){
	this->clearBuffer();
	if(mPacketType == DXL_PACKET_TYPE1){
		mParamBuffer[0] = bAddress;
		mParamBuffer[1] = 2;
		if(this->txRxPacket(bID, INST_READ, 2)){
			DEBUG(8,"rxbuf[5]:%02X,rxbuffer[6]:%02X\n",mRxBuffer[5],mRxBuffer[6]);
			return DXL_MAKEWORD(mRxBuffer[5],mRxBuffer[6]);//( (((word)mRxBuffer[6])<<8)+ mRxBuffer[5] );
		}
		else{
			return 0xffff;
		}

	}else{
		mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(bAddress);
		mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(bAddress);
		mParamBuffer[2]	= 2; //2byte
		mParamBuffer[3]	= 0;
		if(this->txRxPacket(bID, INST_READ, 4)){
			return(DXL_MAKEWORD(mRxBuffer[9], mRxBuffer[10]));
		}else{
			return 0xffff;
		}
	}
}


byte Dynamixel::writeDword( byte bID, word wAddress, uint32 value ){

	this->clearBuffer();
	if(mPacketType == DXL_PACKET_TYPE1)	return 0;
	//insert wAddress to parameter bucket
	mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(wAddress);
	mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(wAddress);
	//insert data to parameter bucket
	mParamBuffer[2]	= DXL_LOBYTE(DXL_LOWORD(value));
	mParamBuffer[3]	= DXL_HIBYTE(DXL_LOWORD(value));
	mParamBuffer[4]	= DXL_LOBYTE(DXL_HIWORD(value));
	mParamBuffer[5]	= DXL_HIBYTE(DXL_HIWORD(value));

	return this->txRxPacket(bID, INST_WRITE, 6); //// parameter length 4 = 2(address)+2(data)
}
uint32 Dynamixel::readDword( byte bID, word wAddress ){

	if(mPacketType == DXL_PACKET_TYPE1)	return 0xFFFFFFFF;

	mParamBuffer[0]	= (unsigned char)DXL_LOBYTE(wAddress);
	mParamBuffer[1]	= (unsigned char)DXL_HIBYTE(wAddress);
	mParamBuffer[2]	= 4; //4byte
	mParamBuffer[3]	= 0;
	if(this->txRxPacket(bID, INST_READ, 4)){
		return DXL_MAKEDWORD( DXL_MAKEWORD( mRxBuffer[9], mRxBuffer[10]),
							  DXL_MAKEWORD( mRxBuffer[11], mRxBuffer[12]));
	}else{
		return 0xFFFFFFFF;
	}
}

/*
 * @brief Sets the target position and speed of the specified servo
 * @author Made by Martin S. Mason(Professor @Mt. San Antonio College)
 * @change 2013-04-17 changed by ROBOTIS,.LTD.
 * */
byte Dynamixel::setPosition(byte ServoID, int Position, int Speed){

    byte param_length = 0;
	if(mPacketType == DXL_PACKET_TYPE1){
		mParamBuffer[0] = (unsigned char)30;
		mParamBuffer[1] = (unsigned char)DXL_LOBYTE(Position);
		mParamBuffer[2] = (unsigned char)DXL_HIBYTE(Position);
		mParamBuffer[3] = (unsigned char)DXL_LOBYTE(Speed);
		mParamBuffer[4] = (unsigned char)DXL_HIBYTE(Speed);
		param_length = 5;

	}else{
		mParamBuffer[0]	= 30;
		mParamBuffer[1]	= 0;
		//insert data to parameter bucket
		mParamBuffer[2] = (unsigned char)DXL_LOBYTE(Position);
		mParamBuffer[3] = (unsigned char)DXL_HIBYTE(Position);
		mParamBuffer[4] = (unsigned char)DXL_LOBYTE(Speed);
		mParamBuffer[5] = (unsigned char)DXL_HIBYTE(Speed);
		param_length = 6;
	}
	return (this->txRxPacket(ServoID, INST_WRITE, param_length));

}
byte Dynamixel::syncWrite(int start_addr, int data_length, word *param, int param_length){

	int i=0, j=0, k=0, num=0;
	this->clearBuffer();
	num = param_length/(data_length + 1); //ID+DATA1+DATA2..
	if(mPacketType == DXL_PACKET_TYPE2){

		mParamBuffer[0]   = DXL_LOBYTE(start_addr);
		mParamBuffer[1]   = DXL_HIBYTE(start_addr);
		mParamBuffer[2]   = DXL_LOBYTE(data_length*2);
		mParamBuffer[3]   = DXL_HIBYTE(data_length*2);

		for(i=4; i < (4+num*(1+data_length*2)); i+=(1+data_length*2) ){
			mParamBuffer[i]   = (byte)param[k++]; //ID
			for(j=0; j < (data_length*2); j+=2){
				mParamBuffer[i+j+1] = DXL_LOBYTE(param[k]); //low byte
				mParamBuffer[i+j+2] = DXL_HIBYTE(param[k]); //high byte
				k++;
			}
		}

		return this->txRxPacket(BROADCAST_ID, INST_SYNC_WRITE, i);

	}else{

		mbLengthForPacketMaking = 0;
		mbIDForPacketMaking = BROADCAST_ID;
		mbInstructionForPacketMaking = INST_SYNC_WRITE;
		mCommStatus = 0;
		mParamBuffer[mbLengthForPacketMaking++] = start_addr;
		mParamBuffer[mbLengthForPacketMaking++] = data_length*2;
		for(i=mbLengthForPacketMaking; i < num*(1+data_length*2); i+=(1+data_length*2)){
			mParamBuffer[i] = param[k++]; //ID
			for(j=0; j < (data_length*2); j+=2){
				mParamBuffer[i+j+1] = DXL_LOBYTE(param[k]); //low byte
				mParamBuffer[i+j+2] = DXL_HIBYTE(param[k]);; //high byte
				k++;
			}
		}
		mbLengthForPacketMaking= i;
		return this->txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	}

}

byte Dynamixel::syncWrite(int start_addr, byte data_length, int *param, int param_length){
	int i=0, j=0, k=0, num=0;
	if(mPacketType == DXL_PACKET_TYPE1) return 0;

	this->clearBuffer();

	num = param_length / (data_length + 1);

	mParamBuffer[0]   = DXL_LOBYTE(start_addr);
	mParamBuffer[1]   = DXL_HIBYTE(start_addr);
	mParamBuffer[2]   = DXL_LOBYTE(data_length*4);
	mParamBuffer[3]   = DXL_HIBYTE(data_length*4);

	for(i=4; i < (4+num*(1+data_length*4)); i+=(1+data_length*4) ){
		mParamBuffer[i]   = (byte)param[k++]; //ID
		for(j=0; j < (data_length*4); j+=4){
			mParamBuffer[i+j+1] = DXL_LOBYTE(DXL_LOWORD(param[k])); //data
			mParamBuffer[i+j+2] = DXL_HIBYTE(DXL_LOWORD(param[k]));
			mParamBuffer[i+j+3] = DXL_LOBYTE(DXL_HIWORD(param[k]));
			mParamBuffer[i+j+4] = DXL_HIBYTE(DXL_HIWORD(param[k]));
			k++;
		}

	}
	return this->txRxPacket(BROADCAST_ID, INST_SYNC_WRITE, 4+i);
}

#if 0
int Dynamixel::bulkRead(byte *param, int param_length){
	//mResult = 0;
	uint32 bulkReadlength=0;

	int n, i, k=0;
	int num = param_length / 5; // each length : 5 (ID ADDR_L ADDR_H LEN_L LEN_H)
	// int pkt_length = param_length + 3;  // 3 : INST CHKSUM_L CHKSUM_H
	//   unsigned char txpacket[MAXNUM_TXPACKET] = {0};
	//   unsigned char rxpacket[MAXNUM_RXPACKET] = {0};

	for(n=0; n < param_length; n++){
		mParamBuffer[n] = param[n];
	}

	/************ TxRxPacket *************/
	// Wait for Bus Idle
	/*    while(comm->iBusUsing == 1)
	{
		//Sleep(0);
	}*/

	//mResult = txrx_PacketEx(BROADCAST_ID, INST_BULK_READ_EX, param_length);;
	this->txRxPacket(BROADCAST_ID, INST_BULK_READ, param_length);


	for(n = 0; n < num; n++){
	   // int id = param[n*5+0];

		bulkReadlength = this->rxPacket(param_length+11);
		/*result =  DXL_MAKEDWORD(	DXL_MAKEWORD(gbpRxBufferEx[9],gbpRxBufferEx[10]),
						DXL_MAKEWORD(gbpRxBufferEx[11],gbpRxBufferEx[12])
					  );*/
		if(mRxBuffer[7] == 0x55){ //packet instruction index
			mBulkData[n].iID = mRxBuffer[4]; //packet ID index
			mBulkData[n].iAddr = DXL_MAKEWORD(mParamBuffer[5*n+1],mParamBuffer[5*n+2]); //get address
			mBulkData[n].iLength = DXL_MAKEWORD(mParamBuffer[5*n+3],mParamBuffer[5*n+4]);//DXL_MAKEWORD(gbpRxBufferEx[PKT_LENGTH_L],gbpRxBufferEx[PKT_LENGTH_H]);
			//TxDStringC("iLength = ");TxDHex8C(mBulkData[n].iLength);TxDStringC("\r\n");
			mBulkData[n].iError = mRxBuffer[7+1]; //Error code
			for(i=0; i < mBulkData[n].iLength ; i++){
				mBulkData[n].iData[i] = mRxBuffer[7+2+i]; //DATA1
			}
		}
		for(k=0;k < DXL_RX_BUF_SIZE ; k++){
			mRxBuffer[k] = 0; //buffer clear
		}
		this->clearBuffer();
	}
	return bulkReadlength;

}
#endif


void Dynamixel::setTxPacketId(byte id){
	mbIDForPacketMaking = id;

}
void Dynamixel::setTxPacketInstruction(byte instruction){
	mbInstructionForPacketMaking = instruction;

}
void Dynamixel::setTxPacketParameter( byte index, byte value ){
	mParamBuffer[index] = value;

}
void Dynamixel::setTxPacketLength( byte length ){
	mbLengthForPacketMaking = length;

}
byte Dynamixel::txrxPacket(void){
	mCommStatus = this->txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}

int Dynamixel::getRxPacketParameter( int index ){
	//return dxl_get_rxpacket_parameter( index );
	return mRxBuffer[5 + index];
}
int Dynamixel::getRxPacketLength(void){
	//return dxl_get_rxpacket_length();
	return mRxBuffer[3]; //length index is 3 in status packet
}

word Dynamixel::getModelNumber(byte bID){
	return this->readWord(bID, 0);
}


void Dynamixel::setID(byte current_ID, byte new_ID){
	this->writeByte(current_ID, 3, new_ID);
}
void Dynamixel::setBaud(byte bID, byte baud_num){
	this->writeByte(bID, 4, baud_num);
}

void Dynamixel::returnLevel(byte bID, byte level){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 16, level);
	}else{
		this->writeByte(bID, 17, level);
	}
}
byte Dynamixel::returnLevel(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 16);
	}else{
		return this->readByte(bID, 17);
	}
}

void Dynamixel::returnDelayTime(byte bID, byte time){
	this->writeByte(bID, 5, time);

}
byte Dynamixel::returnDelayTime(byte bID){
	return this->readByte(bID, 5);
}

void Dynamixel::alarmShutdown(byte bID,byte option){
	this->writeByte(bID, 18, option);
}
byte Dynamixel::alarmShutdown(byte bID){
	return this->readByte(bID, 18);
}

void Dynamixel::controlMode(byte bID, byte mode){ // change wheel, joint
	word model=0;
	if(mPacketType == DXL_PACKET_TYPE1){
		if(mode == 1) this->writeWord(bID,8,0);
		else{
			model = this->getModelNumber(bID);
			if( model == 12 || model == 18 || model == 144) this->writeWord(bID,8,0x3FF);
			else this->writeWord(bID,8,0xFFF);
		}
	}else{
		this->writeByte(bID, 24, 0);
		this->writeByte(bID, 11, mode);
		this->writeByte(bID, 24, 1);
	}
}
byte Dynamixel::controlMode(byte bID){ // return current mode
	if(mPacketType == DXL_PACKET_TYPE1){
		//return this->readByte(bID, 16);
		if(this->readWord(1, 8) == 0 ) return 1;  //wheel mode
		else return 2; //joint mode
	}else{
		return this->readByte(bID, 11);
	}
}

void Dynamixel::wheelMode(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeWord(bID,8,0);
	}else{
		this->writeByte(bID, 24, 0);
		this->writeByte(bID, 11, 1);
		this->writeByte(bID, 24, 1);
	}
}
void Dynamixel::jointMode(byte bID){
	word model=0;
	if(mPacketType == DXL_PACKET_TYPE1){
		model = this->getModelNumber(bID);
		if( model == 12 || model == 18 || model == 144 || model == 300) this->writeWord(bID,8,1023);
		else this->writeWord(bID,8,4095);
	}else{
		this->writeByte(bID, 24, 0);
		this->writeByte(bID, 11, 2);
		this->writeByte(bID, 24, 1);
	}
}


void Dynamixel::maxTorque(byte bID, word value){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeWord(bID, 14, value);
	}else{
		this->writeWord(bID, 15, value);
	}

}
word Dynamixel::maxTorque(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readWord(bID, 14);
	}else{
		return this->readWord(bID, 15);
	}

}

void Dynamixel::maxVolt(byte bID, byte value){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 13, value);
	}else{
		this->writeByte(bID, 14, value);
	}
}
byte Dynamixel::maxVolt(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 13);
	}else{
		return this->readByte(bID, 14);
	}
}

void Dynamixel::minVolt(byte bID, byte value){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 12, value);
	}else{
		this->writeByte(bID, 13, value);
	}
}
byte Dynamixel::minVolt(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 12);
	}else{
		return this->readByte(bID, 13);
	}
}

void Dynamixel::maxTemperature(byte bID, byte temp){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 11, temp);
	}else{
		this->writeByte(bID, 12, temp);
	}
}
byte Dynamixel::maxTemperature(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 11);
	}else{
		return this->readByte(bID, 12);
	}
}

void Dynamixel::torqueEnable(byte bID){
	this->writeByte(bID, 24, 1);
}
void Dynamixel::torqueDisable(byte bID){
	this->writeByte(bID, 24, 0);
}

void Dynamixel::cwAngleLimit(byte bID, word angle){
	this->writeWord(bID, 6, angle);
}
word Dynamixel::cwAngleLimit(byte bID){
	return this->readWord(bID, 6);
}

void Dynamixel::ccwAngleLimit(byte bID, word angle){
	this->writeWord(bID, 8, angle);
}
word Dynamixel::ccwAngleLimit(byte bID){
	return this->readWord(bID, 8);
}

void Dynamixel::goalPosition(byte bID, int position){
	this->writeWord(bID, 30, position);
}
void Dynamixel::goalSpeed(byte bID, int speed){
	this->writeWord(bID, 32, speed);
}
void Dynamixel::goalTorque(byte bID, int torque){
	if(mPacketType == DXL_PACKET_TYPE2)
		this->writeWord(bID, 35, torque);
}

int Dynamixel::getPosition(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readWord(bID, 36);
	}else{
		return this->readWord(bID, 37);
	}
}
int Dynamixel::getSpeed(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readWord(bID, 38);
	}else{
		return this->readWord(bID, 39);
	}
}
int Dynamixel::getLoad(byte bID){
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readWord(bID, 40);
	}else{
		return this->readWord(bID, 41);
	}
}

int Dynamixel::getVolt(byte bID){  //Current Voltage
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 42);
	}else{
		return this->readByte(bID, 45);
	}
}
byte Dynamixel::getTemperature(byte bID){ //Current Temperature
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 43);
	}else{
		return this->readByte(bID, 46);
	}
}
byte Dynamixel::isMoving(byte bID){ // is Moving?? Means if there is any movement
	if(mPacketType == DXL_PACKET_TYPE1){
		return this->readByte(bID, 46);
	}else{
		return this->readByte(bID, 49);
	}
}

void Dynamixel::ledOn(byte bID){
	this->writeByte(bID, 25, 1);
}
void Dynamixel::ledOn(byte bID, byte option){  //for XL-320 , DXL PRO
	this->writeByte(bID, 25, option);
}
void Dynamixel::ledOff(byte bID){
	this->writeByte(bID, 25, 0);
}

void Dynamixel::setPID(byte bID, byte propotional, byte integral, byte derivative){
	if(mPacketType == DXL_PACKET_TYPE2){
		this->writeByte(bID, 27, derivative);
		this->writeByte(bID, 28, integral);
		this->writeByte(bID, 29, propotional);
	}
}

void Dynamixel::complianceMargin(byte bID, byte CW, byte CCW){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 26, CW);
		this->writeByte(bID, 27, CCW);
	}
}
void Dynamixel::complianceSlope(byte bID, byte CW, byte CCW){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeByte(bID, 28, CW);
		this->writeByte(bID, 29, CCW);
	}
}

void Dynamixel::cwTurn(byte bID, word speed){
	word mode=0;
	if(mPacketType == DXL_PACKET_TYPE1){
		//return this->readByte(bID, 16);
		if(this->readWord(bID, 8) == 0 ) mode = 1;  //wheel mode
		else mode = 2; //joint mode
	}else{
		mode = this->readByte(bID, 11);
	}

	if(mode != 1){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeWord(bID,8,0);

	}else{
		this->writeByte(bID, 11, 1);
	}
   }
   this->writeWord(bID, 32, speed+1023);
}

void Dynamixel::ccwTurn(byte bID, word speed){
	word mode=0;
	if(mPacketType == DXL_PACKET_TYPE1){
		//return this->readByte(bID, 16);
		if(this->readWord(bID, 8) == 0 ) mode = 1;  //wheel mode
		else mode = 2; //joint mode
	}else{
		mode = this->readByte(bID, 11);
	}

	if(mode != 1){
	if(mPacketType == DXL_PACKET_TYPE1){
		this->writeWord(bID,8,0);

	}else{
		this->writeByte(bID, 11, 1);
	}
   }
   this->writeWord(bID, 32, speed);
}

/*
 * @brief initialize parameter and get ID, instruction for making packet
 * */
void Dynamixel::initPacket(byte bID, byte bInst){
	mbLengthForPacketMaking = 0;
	mbIDForPacketMaking = bID;
	mbInstructionForPacketMaking = bInst;
	mCommStatus = 0;
}
/*
 * @brief just push parameters, individual ID or moving data, individual data length
 * */
void Dynamixel::pushByte(byte value){
	//packet length is not above the maximum 143 bytes because size of buffer receiver has only 143 bytes capacity.
	//please refer to ROBOTIS e-manual (support.robotis.com)
	if(mbLengthForPacketMaking > 140)//prevent violation of memory access
		return;
	mParamBuffer[mbLengthForPacketMaking++] = value;
}
void Dynamixel::pushParam(byte value){
	if(mbLengthForPacketMaking > 140)//prevent violation of memory access
			return;
	mParamBuffer[mbLengthForPacketMaking++] = value;
}
void Dynamixel::pushParam(int value){

	if(mbLengthForPacketMaking > 140)//prevent violation of memory access
			return;
	mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)DXL_LOBYTE(value);
	mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)DXL_HIBYTE(value);
}
/*
 * @brief transfers packets to dynamixel bus
 * */
byte Dynamixel::flushPacket(void){

	//TxDString("\r\n");
	//TxD_Dec_U8(gbLengthForPacketMaking);
	mCommStatus = this->txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}
/*
 * @brief return current the total packet length
 * */

