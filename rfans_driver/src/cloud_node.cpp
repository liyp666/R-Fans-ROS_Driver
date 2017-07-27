/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "rfans_driver/RfansCommand.h"
#include "bufferDecode.h"

static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::PointCloud2 outCloud ;

static ros::Publisher  s_output;
static ros::Subscriber s_sub ;

static void RFansPacketReceived(rfans_driver::RfansPacket pkt) {
  int rtn = 0 ;
  if (s_output.getNumSubscribers() == 0)  return;


  SSBufferDec::Depacket(pkt, outBlocks) ;
  SSBufferDec::ConvertXyz(outBlocks, outXyzBlocks) ;
  rtn = SSBufferDec::ConvertPointCloud(outBlocks, outXyzBlocks, outCloud);

  if(rtn) {
    s_output.publish(outCloud);
    SSBufferDec::ResetPointCloud2(outCloud);
  }
  return ;
}


int main ( int argc , char ** argv )
{

  // Initialize the ROS system
  ros::init ( argc , argv , "rfans_cloud") ;
  ros::NodeHandle nh ;
  SSBufferDec::InitPointcloud2(outCloud) ;

  s_sub= nh.subscribe ("rfans_driver/rfans_packets" ,
                       RFANS_POINT_CLOUD_NUM, &RFansPacketReceived ) ;

  s_output = nh.advertise<sensor_msgs::PointCloud2>("rfans_driver/rfans_points", RFANS_POINT_CLOUD_NUM);
  ros::spin () ;
}
