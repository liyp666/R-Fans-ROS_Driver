/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _FRAME_LIB_H_
#define _FRAME_LIB_H_
#include <iostream>
#include <string>
#include "rfans_driver/RfansPacket.h"

using namespace std;

const static int REG_DEVICE_CTRL = (0x40);
const static int REG_DATA_TRANS = (0x70);

const static int CMD_SCAN_SPEED_5HZ = 0;
const static int CMD_SCAN_SPEED_10HZ = 0x50;
const static int CMD_SCAN_SPEED_20HZ = 0xF0;
const static int CMD_SCAN_ENABLE = 0x1;
const static int CMD_LASER_ENABLE = 0x2;
const static int CMD_CALC_DATA = 0x2;
const static int CMD_DEBUG_DATA = 0x5;
const static int CMD_RCV_CLOSE = 0x0;

const static int ANGLE_SPEED_5HZ = 5;
const static int ANGLE_SPEED_10HZ = 10;
const static int ANGLE_SPEED_20HZ = 20;

static unsigned short DEVICE_PORT_NUMBER = 2014;
static unsigned short PC_PORT_NUMBER = 2014;
static std::string DEVICE_IP_STRING = "192.168.0.3";

static unsigned short UDP_FRAME_MIN =(18);

#define DEB_FRAME_WRITE (0xA5)  //write head sync
#define DEB_FRAME_READ  (0x5a)  //read head sync
#define DEB_FRAME_ERROR  (0xE7) //err  haad sync

#define FRAME_MSG_LENGTH (1024) 

const int UDPREG_MAX_COUNT = 256;
const int ROMREG_MAX_COUNT = 0x7FF;

typedef enum{
  eCmdWrite,  //write command
  eCmdRead,   //read command
  eCmdQuery,  //Query command
} SCD_FRAME_TYPE_E ;

typedef struct _frams_buffer{
  char msgStream[FRAME_MSG_LENGTH];
  int writeIdx;
  int readIdx;
  int length;
} FRAMS_BUFFER_S;

#pragma pack(1)
typedef struct {                 //! rfans udp command package
  unsigned char msgHead;         //!< head sync
  unsigned char msgCheckSum;     //!< check sum
  unsigned short regAddress;     //!< register add
  unsigned int regData;          //!< register data
} DEB_FRAME_S;

typedef enum {
  eDevCmdIdle = 0,
  eDevCmdWork,
  eDevCmdSimu,
  eDevCmdBreak,
  eDevCmdReset,
  eDevCmdAsk,
} DEB_CMD_E;

typedef enum {
  eFormatCalcData = 0x2,
  eFormatDebugData = 0x5,
}DEB_DFORMAT_E;


static int DEVICE_MOTOR_HZ = 5;
typedef struct {
  DEB_CMD_E cmdstat;
  DEB_DFORMAT_E dataFormat;
  int scnSpeed;
  int lsrFreq;
  float rangeMin, rangeMax;
}DEB_PROGRM_S;

static const size_t upd_packet_size = 0x8000;  //32KB
const unsigned char ID_RFANSBLOCKV2_SYNC = 0x96;
typedef struct {
  unsigned short angle  : 16 ;          //scan angle, 0.01бу
  unsigned short rangeOne  : 16 ;       //echo 1,  cm
  unsigned short rangeTwo  : 16 ;       //echo 2,  cm
  unsigned char  intentOne : 8 ;        //0~255
  unsigned char  intentTwo : 8 ;        //0~255
}SCDRFANS_POINT_S;

const int RFANS_LASER_COUNT = 16 ;
typedef struct {                                        //138 byte
  unsigned char           dataID : 8;                   //Sync Number
  unsigned char           chksum : 8;                   //chksum  Bytes[3,138]
  unsigned int            t0stampH  : 32;               //T0 Bloceek
  unsigned int            t0stampL  : 32;               //T0 tag
  SCDRFANS_POINT_S        laserData[RFANS_LASER_COUNT] ;//
}SCDRFANS_BLOCK_S;

typedef struct {
  double time ;
  float x,y,z ;
}RFANS_XYZ_S;

#pragma pack()

#ifdef __cplusplus
extern "C"
{
#endif
int swapchar( unsigned char * _data, int size_ ) ;
int checkSum(unsigned char * _dataBuf, int count_ ) ;

DEB_FRAME_S packDEBV3Frame(SCD_FRAME_TYPE_E flag, int regAddress_, int regData_);

void writeFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, char * _mt_frame, int mt_size);

void readDEBFrameBuffer(FRAMS_BUFFER_S *mtFrameMsgBuf, DEB_FRAME_S *mtRegMap);

int searchBlock(unsigned char key,unsigned char *data, int size,int flag,
                SCDRFANS_BLOCK_S &outBlock) ;

int ssDepacket(rfans_driver::RfansPacket &inPack, std::vector<SCDRFANS_BLOCK_S> &outBlocks) ;

int ssConvertXyz(std::vector<SCDRFANS_BLOCK_S> &inBlocks, std::vector<RFANS_XYZ_S> &outXyzBlocks) ;

#ifdef __cplusplus
}
#endif


#endif
