/*
 * bag_to_ctp.cpp
 *
 *  Created on: Feb 17, 2011
 *      Author: kalakris
 */

#include <policy_learning_tools/bag_to_ctp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_to_ctp");

  ros::NodeHandle nh("~");
  policy_learning_tools::BagToCTP bag_to_ctp(nh);
  return bag_to_ctp.run(argc, argv);
}
