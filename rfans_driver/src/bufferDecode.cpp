#include "bufferDecode.h"
#include <string.h>
#include <ros/ros.h>
#include "ioapi.h"

const float ANGLE_CIRCLE_CONDITION = 270; //角度抖动处理值
const float UINTCONVERT = 0.01;
static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);

SSBufferDec::SSBufferDec()
{
  ROS_INFO_STREAM( "sizeof( SCDRFANS_BLOCK_S) "<<sizeof( SCDRFANS_BLOCK_S));
  reset();
}

SSBufferDec::~SSBufferDec()
{

}

int SSBufferDec::writeBuffer(unsigned char *data, int size)
{
  if(m_decBuf.bufSize+size > DECODE_BUFFER_SIZE) return 0 ;

  memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
  m_decBuf.bufSize += size;
  m_decBuf.wrHead += size ;

  //  ROS_INFO_STREAM( "writeBuffer"
  //                  << " bufSize " << m_decBuf.bufSize
  //                  << " wrHead "<< m_decBuf.wrHead
  //                  << " rdTail " <<m_decBuf.rdTail);
  return size ;
}

int SSBufferDec::findPacketPro() {
  int rtn = 0 ;
  int tmpIndex = 0 ;
  SCDRFANS_BLOCK_S tmpBlock;
  tmpIndex = searchBlock(ID_RFANSBLOCKV2_SYNC,
                         m_decBuf.buffer+m_decBuf.rdTail, m_decBuf.bufSize,
                         0, tmpBlock ) ;

  if(tmpBlock.dataID == ID_RFANSBLOCKV2_SYNC ) {
    rtn = 1;
    m_decBuf.bufSize = m_decBuf.wrHead-tmpIndex;
    m_decBuf.rdTail = tmpIndex+m_decBuf.rdTail;

  } else {
    m_decBuf.bufSize = tmpIndex ;
    m_decBuf.rdTail =  m_decBuf.wrHead-tmpIndex ;
  }
  return rtn;
}

int SSBufferDec::findZeroPro() {
  int rtn = 0 ;
//  int tmpIndex = 0 ;
//  float tmpCurAng = 0 ;
//  SCDRFANS_BLOCK_S tmpBlock;

//  tmpIndex = searchBlock(ID_RFANSBLOCKV2_SYNC,
//                         m_decBuf.buffer+m_decBuf.rdTail, m_decBuf.bufSize,
//                         0,tmpBlock ) ;

//  if(tmpBlock.dataID == ID_RFANSBLOCKV2_SYNC ) {
//    tmpCurAng = tmpBlock.laserData[0].angle*UINTCONVERT;

//    if( s_preAngle-tmpCurAng>ANGLE_CIRCLE_CONDITION) {
//      rtn = 1;
//      m_decBuf.bufSize = m_decBuf.wrHead-tmpIndex ;
//      m_decBuf.rdTail = tmpIndex+m_decBuf.rdTail ;
//    } else {
//      s_preAngle = tmpCurAng;
//      //go to the next block
//      m_decBuf.bufSize = m_decBuf.wrHead-tmpIndex;
//      m_decBuf.rdTail = tmpIndex++m_decBuf.rdTail;
//    }
//  } else {
//    m_decBuf.bufSize = tmpIndex ;
//    m_decBuf.rdTail =  m_decBuf.wrHead-tmpIndex ;
//  }
  return rtn;
}

int SSBufferDec::readBuffer() {
  int rtn = 0 ;
  SCDRFANS_BLOCK_S tmpBlock;
  static const int BLOCK_SIZE = sizeof(tmpBlock);
  int tmpIndex = 0 ;
  while ( m_decBuf.bufSize> BLOCK_SIZE && !rtn ) {

      memset(&tmpBlock,0,sizeof(tmpBlock) ) ;
      tmpIndex = searchBlock(ID_RFANSBLOCKV2_SYNC,
                             m_decBuf.buffer+m_decBuf.rdTail, m_decBuf.bufSize,
                             0, tmpBlock ) ;

      if(tmpBlock.dataID == ID_RFANSBLOCKV2_SYNC ) {

      m_decBuf.rdTail = tmpIndex + m_decBuf.rdTail;
      m_decBuf.bufSize = m_decBuf.wrHead-m_decBuf.rdTail;


      //ouput buffer
      memcpy(&m_packet.data[0]+m_packetSize,
          &tmpBlock, BLOCK_SIZE );

      m_packetSize += BLOCK_SIZE;
      m_decBuf.rdTail +=  BLOCK_SIZE;
      m_decBuf.bufSize -= BLOCK_SIZE;

      if( m_decBuf.rdTail > m_decBuf.wrHead) {
        ROS_INFO("m_decBuf.rdTail > m_decBuf.wrHead") ;
        m_decBuf.rdTail= m_decBuf.wrHead;
        m_decBuf.bufSize = 0 ;
      }

      if(m_decBuf.bufSize <0 ) {
        ROS_INFO("m_decBuf.bufSize <0") ;
        m_decBuf.bufSize = 0 ;
      }

//      ROS_INFO("tmpBlock.t0stampL B %u  index %d wrHead %d rdTail %d bufSize %d m_packetSize %d\n",
//             tmpBlock.t0stampL,tmpIndex, m_decBuf.wrHead,
//             m_decBuf.rdTail, m_decBuf.bufSize ,m_packetSize);



    } else {
      m_decBuf.rdTail =  m_decBuf.wrHead-tmpIndex ;
      m_decBuf.bufSize = tmpIndex ;

//      ROS_INFO(tmpMsg,"tmpBlock  tmpBlock.dataID != ID_RFANSBLOCKV2_SYNC rdTail %d wrHead %d bufSize %d \n",
//              m_decBuf.rdTail, m_decBuf.wrHead, m_decBuf.bufSize);
    }

    if(m_packetSize >= packet_size) { //buffer full
       rtn = 1;
    }

  }
  return rtn;
}

int SSBufferDec::ouputPacket(rfans_driver::RfansPacket &pkt) {

  pkt = m_packet;   //ouput pkt;
  m_packet.packNum++;
  m_packetSize = 0;
  memset(&m_packet.data[0], 0, sizeof(m_packet.data) ) ;
  return 1;
}

int SSBufferDec::readPacket(rfans_driver::RfansPacket &pkt)
{
  int rtn = 0;
  if(m_status == eReady) {
      m_status = eReadPackData;
  }

  if(m_status == eReadPackData) {
    if( readBuffer() ) {
      m_status = eReadFinish;
    }
  }

  if(m_status == eReadFinish) {
    rtn = ouputPacket(pkt) ;
    m_status = eReadPackData;
  }
  bufferReverse();

  //  ROS_INFO_STREAM( "readPacket"
  //                  << " bufSize " << m_decBuf.bufSize
  //                  << " wrHead "<< m_decBuf.wrHead
  //                  << " rdTail " <<m_decBuf.rdTail);

  return rtn ;
}

void SSBufferDec::bufferReverse()
{

  char tmpBuffer[DECODE_BUFFER_SIZE] = { 0 };

  if (m_decBuf.bufSize > 0) {
    memcpy(tmpBuffer, m_decBuf.buffer + m_decBuf.rdTail, m_decBuf.bufSize);
    memcpy(m_decBuf.buffer, tmpBuffer, m_decBuf.bufSize);
    m_decBuf.rdTail = 0;
    m_decBuf.wrHead = m_decBuf.bufSize;
  } else {
    m_decBuf.wrHead = m_decBuf.bufSize = m_decBuf.rdTail = 0;
  }
  return;
}

void SSBufferDec::reset()
{
  memset(&m_decBuf,0,sizeof(m_decBuf)) ;
  memset(&m_packet,0,sizeof(m_packet)) ;
  m_status = eReady;
  s_preAngle =0 ;
  m_packetSize = 0;
}

int SSBufferDec::Depacket(rfans_driver::RfansPacket &inPack, std::vector<SCDRFANS_BLOCK_S> &outBlocks)
{
  int rtn =0;
  static const size_t packetCount = sizeof(rfans_driver::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S );
  outBlocks.clear();
  outBlocks.resize( packetCount );

  SCDRFANS_BLOCK_S *tmpBlocks =  (SCDRFANS_BLOCK_S *)(&inPack.data[0]) ;
  for(int i=0;i<packetCount;i++) {
    outBlocks[i] = tmpBlocks[i]  ;
  }

  return rtn ;
}

int SSBufferDec::ConvertXyz(std::vector<SCDRFANS_BLOCK_S> &inBlocks, std::vector<RFANS_XYZ_S> &outXyzBlocks)
{
  float UINTCONVERT =0.01 ;
  float x,y,z ,ot;

  static double VAngle[] = {
    -15.0, -13.0, -11.0,  -9.0, -7.0,
    -5.0,  -4.0,   -3.0,  -2.0, -1.0,
     0,   1.0,      3.0,   5.0,  7.0, 9.0 };

  double tmptheta=0, tmpRange = 0 ;
  outXyzBlocks.clear();
  outXyzBlocks.resize( inBlocks.size()*RFANS_LASER_COUNT);

  RFANS_XYZ_S tmpXyz ;
  int tmpXyzIndex = 0 ;

  for(int i=0;i <inBlocks.size();i++) {

    for(int j=0;j <RFANS_LASER_COUNT;j++) {


      tmpXyzIndex = i*RFANS_LASER_COUNT+j;
      tmpRange = inBlocks[i].laserData[j].rangeOne*UINTCONVERT;
      ot = inBlocks[i].laserData[j].angle *UINTCONVERT*M_PI / 180.0;
      tmptheta = VAngle[j] * M_PI / 180.0;

      tmpXyz.x = tmpRange*cos(tmptheta) *cos(-ot );
      tmpXyz.y = tmpRange*cos(tmptheta) *sin(-ot );
      tmpXyz.z = tmpRange*sin(tmptheta) ;
      tmpXyz.time = inBlocks[i].t0stampH + t0stampL*0.0000001*31.25*j;

      outXyzBlocks[tmpXyzIndex] = tmpXyz ;
    }
  }
  return 0;
}

