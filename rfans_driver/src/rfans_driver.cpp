/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "rfans_driver.h"
#include "rfans_driver/RfansCommand.h"

namespace rfans_driver {

static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const int RFANS_PACKET_NUM = 1024 ;

static Rfans_Driver *s_this = NULL;

static char s_simuFileName[] = "test.imp";

/** @brief Rfans Command Handle */
bool CommandHandle(rfans_driver::RfansCommand::Request  &req,
                   rfans_driver::RfansCommand::Response &res)
{
  res.status = 1;

  ROS_INFO("request: cmd= %d , speed = %d Hz", (int)req.cmd, (int)req.speed);
  ROS_INFO("sending back response: [%d]", (int)res.status);

  DEB_PROGRM_S tmpProg ;
  tmpProg.cmdstat = (DEB_CMD_E)req.cmd;
  tmpProg.dataFormat = eFormatCalcData;
  tmpProg.scnSpeed = req.speed;
  if(s_this) {
    s_this->prog_Set(tmpProg);
  }
  return true;
}

Rfans_Driver::Rfans_Driver(ros::NodeHandle mtNode, int type)
{
  m_svr = mtNode.advertiseService("rfans_driver/rfans_control", CommandHandle);
  m_output = mtNode.advertise<rfans_driver::RfansPacket>("rfans_driver/rfans_packets", RFANS_PACKET_NUM);

  if(type) {
    m_devapi = new rfans_driver::SSFileAPI(s_simuFileName);
  } else {
    m_devapi = new rfans_driver::IOSocketAPI();
  }


  s_this = this ;
}

Rfans_Driver::~Rfans_Driver()
{
  if(m_devapi) delete m_devapi;
}

/** @brief Rfnas Driver Core */
int Rfans_Driver::spinOnce()
{
  int rtn = 0 ;

  m_devapi->revPacket();

  do{
    rtn = m_devapi->getPacket(tmpPacket);
    if(rtn > 0) {
      m_output.publish(tmpPacket) ;
    }
  }while (rtn);

  return rtn ;
}

/** @brief control the device
         *  @param .parameters
         */
int Rfans_Driver::prog_Set(DEB_PROGRM_S &program)
{
  unsigned int tmpData = 0;

  switch (program.dataFormat) {
  case eFormatCalcData:
    tmpData |= CMD_CALC_DATA;
    break;
  case eFormatDebugData:
    tmpData |= CMD_DEBUG_DATA;
    break;
  }
  m_devapi->HW_WRREG(0, REG_DATA_TRANS, tmpData);
  //===============================================================
  tmpData = 0;
  switch (program.scnSpeed) {
  case ANGLE_SPEED_10HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_10HZ;
    break;
  case ANGLE_SPEED_20HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_20HZ;
    break;
  case ANGLE_SPEED_5HZ:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  default:
    tmpData |= CMD_SCAN_ENABLE;
    tmpData |= CMD_SCAN_SPEED_5HZ;
    break;
  }

  tmpData |= CMD_LASER_ENABLE;
  switch (program.cmdstat) {
  case eDevCmdWork:
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdIdle:
    tmpData = CMD_RCV_CLOSE;
    m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
    break;
  case eDevCmdAsk:
    break;
  default:
    break;
  }

  return 0;

}//end prog_Set

} //rfans_driver namespace
