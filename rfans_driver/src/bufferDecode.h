#ifndef _BUFFER_DECODER_H_
#define _BUFFER_DECODER_H_
#include "rfans_driver/RfansPacket.h"
#include "ssFrameLib.h"

static const int DECODE_BUFFER_SIZE = 0x20000; // 128KB
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
  int writeBuffer(unsigned char *data, int size);
  int readPacket(rfans_driver::RfansPacket &pkt);
  void reset();

  static int Depacket(rfans_driver::RfansPacket &inPack, std::vector<SCDRFANS_BLOCK_S> &outBlocks);
  static int ConvertXyz(std::vector<SCDRFANS_BLOCK_S> &inBlocks, std::vector<RFANS_XYZ_S> &outXyzBlocks);
private:
  int findPacketPro();
  int findZeroPro();
  int readBuffer() ;
  int ouputPacket(rfans_driver::RfansPacket &pkt);
  void bufferReverse();
private:
  UDP_DECBUFFER_S m_decBuf;
  DEC_STSTUS_E m_status ;
  rfans_driver::RfansPacket m_packet ;
  int m_packetSize ;
  float s_preAngle ;
};


#endif
