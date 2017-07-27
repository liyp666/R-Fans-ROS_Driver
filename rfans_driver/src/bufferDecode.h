#ifndef _BUFFER_DECODER_H_
#define _BUFFER_DECODER_H_
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "rfans_driver/RfansPacket.h"
#include "ssFrameLib.h"


static const int DECODE_BUFFER_SIZE = 0x40000; // 256KB
typedef struct {
  int wrHead, rdTail, bufSize;
  unsigned char buffer[DECODE_BUFFER_SIZE];
} UDP_DECBUFFER_S;

typedef enum {
  eReady,
  eSearch,
  eReadPackData,
  eReadFinish
}DEC_STSTUS_E ;

class SSBufferDec {
public:
  SSBufferDec();
  ~SSBufferDec();
  int moveWriteIndex(int setpIndex);
  int moveReadIndex(int setpIndex);
  unsigned char * getWriteIndex();
  unsigned char * getReadIndex();

  int writeBuffer(unsigned char *data, int size);

  int readPacket(rfans_driver::RfansPacket &pkt);
  int readPacket(std::vector<SCDRFANS_BLOCK_S> &outBlocks);

  int readPacketStream(rfans_driver::RfansPacket &pkt);
  int size();
  int freeSize();
  void reset();

  static int Depacket(rfans_driver::RfansPacket &inPack, std::vector<SCDRFANS_BLOCK_S> &outBlocks);
  static int ConvertXyz(std::vector<SCDRFANS_BLOCK_S> &inBlocks, std::vector<RFANS_XYZ_S> &outXyzBlocks);

  static int ConvertPointCloud(std::vector<SCDRFANS_BLOCK_S> &inBlocks, std::vector<RFANS_XYZ_S> &inXyzBlock,
                               sensor_msgs::PointCloud2 &outCloud) ;

  static void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) ;
  static void ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) ;
private:
  int findPacketPro();
  int findZeroPro();
  int readBuffer() ;
  int readBufferSteam() ;
  int ouputPacket(rfans_driver::RfansPacket &pkt);
  void bufferReverse();
private:
  UDP_DECBUFFER_S m_decBuf;
  DEC_STSTUS_E m_status ;
  rfans_driver::RfansPacket m_packet ;
  int m_packetSize ;
  int m_blockCout ;
  float s_preAngle ;
};


#endif
