#include "bufferDecode.h"
#include <string.h>
#include <ros/ros.h>
#include "ioapi.h"

static const int LINE_POINT_MINI_COUNT = 500; //一条扫描线最少点的个数
static const float ANGLE_CIRCLE_CONDITION = 270; //角度抖动处理值
static const float UINTCONVERT = 0.01;
static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static float s_lastAngle = 0 ;

static int LINE_POINT_COUNT = 128*1024;
static std::vector<RFANS_XYZ_S> s_lineData;
static int s_lineCount = 0 ;

SSBufferDec::SSBufferDec()
{
  reset();
}

SSBufferDec::~SSBufferDec()
{
}

int SSBufferDec::moveWriteIndex(int setpIndex)
{
  m_decBuf.bufSize += setpIndex;
  m_decBuf.wrHead = (m_decBuf.wrHead+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

int SSBufferDec::moveReadIndex(int setpIndex)
{
  m_decBuf.bufSize -= setpIndex;
  m_decBuf.rdTail = (m_decBuf.rdTail+setpIndex)%DECODE_BUFFER_SIZE;
  return m_decBuf.bufSize ;
}

unsigned char *SSBufferDec::getWriteIndex()
{
  return m_decBuf.buffer+m_decBuf.wrHead ;
}

unsigned char *SSBufferDec::getReadIndex()
{
  return m_decBuf.buffer+m_decBuf.rdTail ;
}


int SSBufferDec::writeBuffer(unsigned char *data, int size)
{
  if(m_decBuf.bufSize+size >= DECODE_BUFFER_SIZE) return 0 ;

  memcpy(m_decBuf.buffer+m_decBuf.wrHead,data,size);
  m_decBuf.bufSize += size;
  m_decBuf.wrHead += size ;

  //  ROS_INFO_STREAM( "writeBuffer"
  //                  << " bufSize " << m_decBuf.bufSize
  //                  << " wrHead "<< m_decBuf.wrHead
  //                  << " rdTail " <<m_decBuf.rdTail);
  return size ;
}

int SSBufferDec::readPacket(rfans_driver::RfansPacket &pkt)
{
  int rtn = 0;
//  if(m_status == eReady) {
//    m_status = eReadPackData;
//  }

//  if(m_status == eReadPackData) {
//    if( readBuffer() ) {
//      m_status = eReadFinish;
//    }
//  }

//  if(m_status == eReadFinish) {
//    rtn = ouputPacket(pkt) ;
//    m_status = eReadPackData;
//  }

  if( readBuffer() ) {
    rtn = ouputPacket(pkt) ;
  }
  bufferReverse();

  return rtn ;
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
      //ouput buffer
      memcpy(&m_packet.data[0]+m_packetSize,
          &tmpBlock, BLOCK_SIZE );
      moveReadIndex(BLOCK_SIZE+tmpIndex) ;
      m_packetSize += BLOCK_SIZE;

//      if(tmpIndex>0)
//        ROS_INFO("tmpBlock.t0stampL B %u  index %d wrHead %d rdTail %d bufSize %d m_packetSize %d\n",
//                 tmpBlock.t0stampL,tmpIndex, m_decBuf.wrHead,
//                 m_decBuf.rdTail, m_decBuf.bufSize ,m_packetSize);

    } else {
      m_decBuf.rdTail =  m_decBuf.wrHead-tmpIndex ;
      m_decBuf.bufSize = tmpIndex ;
    }

    if(m_packetSize >= packet_size) { //buffer full
      rtn = 1;
    }

  }
  return rtn;
}

int SSBufferDec::readBufferSteam()
{
  int rtn = 0 ;
  SCDRFANS_BLOCK_S tmpBlock;
  static const int BLOCK_SIZE = sizeof(tmpBlock);
  int tmpIndex = 0 ;

  if( m_decBuf.bufSize >= m_packet.data.size() && !rtn ) {

    //ouput buffer
    memcpy(&m_packet.data[0], getReadIndex(), m_packet.data.size() );

    moveReadIndex(m_packet.data.size());
    rtn = 1;
  }
  return rtn;

}

int SSBufferDec::ouputPacket(rfans_driver::RfansPacket &pkt) {

  pkt = m_packet;   //ouput pkt;
  m_packet.packNum++;
  m_packetSize = 0;
  //memset(&m_packet.data[0], 0, sizeof(m_packet.data) ) ;
  return 1;
}

int SSBufferDec::readPacket( std::vector<SCDRFANS_BLOCK_S> &outBlocks)
{
  int rtn = 0 ;
  SCDRFANS_BLOCK_S tmpBlock;

  static const int BLOCK_SIZE = sizeof(tmpBlock);
  int tmpIndex = 0 ;

  while ( m_decBuf.bufSize> BLOCK_SIZE && !rtn ) {

    tmpIndex = searchBlock(ID_RFANSBLOCKV2_SYNC,
                           getReadIndex(), m_decBuf.bufSize,
                           0, tmpBlock ) ;

    if(tmpBlock.dataID == ID_RFANSBLOCKV2_SYNC ) {
      outBlocks[m_blockCout++] =tmpBlock;
      moveReadIndex(BLOCK_SIZE+tmpIndex) ;
    } else {
      m_decBuf.rdTail =  m_decBuf.wrHead-tmpIndex ;
      m_decBuf.bufSize = tmpIndex ;
    }

    if(outBlocks.size() <= m_blockCout) { //buffer full
      m_blockCout = 0 ;
      rtn = 1;
    }

  }
  bufferReverse();
  return rtn ;
}

int SSBufferDec::readPacketStream(rfans_driver::RfansPacket &pkt)
{

  int rtn = 0;
  if(m_status == eReady) {
    m_status = eReadPackData;
  }

  if(m_status == eReadPackData) {
    if( readBufferSteam() ) {
      m_status = eReadFinish;
    }
  }

  if(m_status == eReadFinish) {
    rtn = ouputPacket(pkt) ;
    m_status = eReadPackData;
  }
  return rtn ;
}

int SSBufferDec::size()
{
  return m_decBuf.bufSize;
}

int SSBufferDec::freeSize()
{
  return DECODE_BUFFER_SIZE-m_decBuf.bufSize;
}


static char s_tmpBuffer[DECODE_BUFFER_SIZE];
void SSBufferDec::bufferReverse()
{
  if (m_decBuf.bufSize > 0) {
    memcpy(s_tmpBuffer, m_decBuf.buffer + m_decBuf.rdTail, m_decBuf.bufSize);
    memcpy(m_decBuf.buffer, s_tmpBuffer, m_decBuf.bufSize);
    m_decBuf.rdTail = 0;
    m_decBuf.wrHead = m_decBuf.bufSize;
  } else {
    m_decBuf.wrHead = m_decBuf.bufSize = m_decBuf.rdTail = 0;
  }
  return;
}

void SSBufferDec::reset()
{
  //memset(&m_decBuf,0,sizeof(m_decBuf)) ;
  m_decBuf.wrHead = m_decBuf.rdTail = 0 ;
  memset(&m_packet,0,sizeof(m_packet)) ;
  m_status = eReady;
  s_preAngle =0 ;
  m_packetSize = 0;
  m_blockCout = 0 ;
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
      tmpXyz.intent = inBlocks[i].laserData[j].intentOne;

      outXyzBlocks[tmpXyzIndex] = tmpXyz ;
    }
  }
  return 0;
}


int SSBufferDec::ConvertPointCloud(std::vector<SCDRFANS_BLOCK_S> &inBlock, std::vector<RFANS_XYZ_S> &inXyzBlock,
                                   sensor_msgs::PointCloud2 &outCloud)
{
  int rtn = 0 ;

  //  for(int i = 0; i < inXyzBlock.size() && outCloud.width <= inXyzBlock.size() ;i++) {
  //    memcpy(&outCloud.data[0] + outCloud.width*outCloud.point_step, &inXyzBlock[i], outCloud.point_step);
  //    outCloud.width++;
  //    rtn = 1 ;
  //  }
  double tmpTime = 0 ;
  float timeOffset = 31.25;
  float timeScale = 0.0000001;
  float tmpRange = 0 ;

  float tmpAngle = 0 ;
  float UINTCONVERT = 0.01 ;
  int tmpXyzIndex = 0 ;

  for(int i=0;i <inBlock.size();i++){
    for(int j=0 ; j < RFANS_LASER_COUNT && tmpXyzIndex < inXyzBlock.size() ;j++) {
      tmpXyzIndex = i*RFANS_LASER_COUNT+j;
      tmpTime = inBlock[i].t0stampH + inBlock[i].t0stampL *timeScale + timeOffset * j*timeScale; //calc UTC time
      //tmpRange = outBlocks[i].laserData[j].rangeOne*UINTCONVERT;

      tmpAngle = inBlock[i].laserData[j].angle*UINTCONVERT;

      if( 0 == j) {
        if( (s_lastAngle - tmpAngle) > ANGLE_CIRCLE_CONDITION
            && s_lineCount > LINE_POINT_MINI_COUNT ) {
          outCloud.data.resize( outCloud.point_step*s_lineCount  );
          outCloud.row_step = outCloud.data.size();
          memcpy(&outCloud.data[0] , &s_lineData[0], outCloud.data.size() );

          outCloud.width = s_lineCount;
          s_lineCount = 0 ;
          rtn = 1;
        }

        s_lastAngle = tmpAngle;
      }

      s_lineData[s_lineCount++] = inXyzBlock[tmpXyzIndex];
      if(s_lineCount>=LINE_POINT_COUNT) {
        s_lineCount = LINE_POINT_COUNT-1;
      }


    } //end laser number
  } //end packet number


  return rtn;
}

void SSBufferDec::InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
  static const size_t DataSize = sizeof(rfans_driver::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S ) * sizeof(RFANS_XYZ_S) *RFANS_LASER_COUNT;
  initCloud.data.clear();
  initCloud.data.resize( DataSize); //point data

  initCloud.is_bigendian = false ;//false;      //stream foramt
  initCloud.fields.resize(5);          //line format
  initCloud.is_dense = false;

  int tmpOffset = 0 ;
  for(int i=0; i < initCloud.fields.size() ;i++) {
    switch(i) { //value type
    case 0:
      initCloud.fields[i].name = "x" ;
      initCloud.fields[i].datatype = 7u;
      break;
    case 1:
      initCloud.fields[i].name = "y" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 2:
      initCloud.fields[i].name = "z" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    case 3:
      initCloud.fields[i].name = "i" ;
      initCloud.fields[i].datatype = 2u;
      tmpOffset += 4;
      break;
    case 4:
      initCloud.fields[i].name = "laserid" ;
      initCloud.fields[i].datatype = 2u;
      tmpOffset += 1;
      break;
    case 5:
      initCloud.fields[i].name = "angle" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 1;
      break;
    case 6:
      initCloud.fields[i].name = "range" ;
      initCloud.fields[i].datatype = 7u;
      tmpOffset += 4;
      break;
    }
    initCloud.fields[i].offset = tmpOffset ;      //value offset
    initCloud.fields[i].count = 1 ;
  }
  initCloud.height = 1;
  initCloud.point_step = sizeof(RFANS_XYZ_S);
  initCloud.row_step = DataSize ;
  initCloud.width = 0 ;
  initCloud.header.frame_id = "/world";

  s_lastAngle = 0 ;
  s_lineData.resize(LINE_POINT_COUNT);
  s_lineCount = 0 ;
}

void SSBufferDec::ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) {
  static unsigned int s_count = 2;
  char tmpMSg[64] ;
  memset(tmpMSg,0,sizeof(tmpMSg));
  sprintf(tmpMSg,"%u",s_count++);
  //initCloud.header.frame_id = tmpMSg;

  initCloud.width = 0;
  //memset(&initCloud.data[0], 0, initCloud.data.size() ) ;
}
