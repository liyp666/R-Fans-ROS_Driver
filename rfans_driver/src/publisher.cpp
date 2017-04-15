/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include "rfans_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rfans_driver");
  ros::NodeHandle node;
  // start the driver
  rfans_driver::Rfans_Driver dvr(node);
  //loop until shut down
  while( ros::ok() ) {
    dvr.spinOnce();
    ros::spinOnce();
  }
  return 0;
}
