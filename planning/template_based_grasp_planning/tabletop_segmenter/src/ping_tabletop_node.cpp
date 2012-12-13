/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>

#include "tabletop_segmenter/TabletopSegmentation.h"

namespace enc = sensor_msgs::image_encodings;

bool dumpCVImage(const sensor_msgs::Image & img, 
		 const char *name)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(img);
    }
  catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

  cv::imwrite(name, cv_ptr->image);
  
  return true;
}

void dumpImages( const std::vector<sensor_msgs::Image> &masks,
		 const sensor_msgs::Image &depth,
		 const sensor_msgs::Image &rgb )
{

  // dump rectified depth image
  if(! dumpCVImage(depth, "depth.png"))
    ROS_WARN("Problems while writing depth image");
  
  // dump rgb image
  if(! dumpCVImage(rgb, "rgb.png"))
    ROS_ERROR("Problems while writing depth image");
  
  // dump segmentation masks
  for(size_t i=0; i<masks.size(); ++i)
    {
      char filename[512];
      sprintf(filename, "mask_%d.png", (int)i);
      
      if(! dumpCVImage( masks[i], filename))
	ROS_ERROR("Problems while writing depth image");
    }
  
}

void dumpImages( const std::vector<sensor_msgs::Image> &masks,
		 const sensor_msgs::Image &depth )
{
  
  // dump rectified depth image
  if(! dumpCVImage(depth, "depth.png"))
    ROS_WARN("Problems while writing depth image");
  
  // dump segmentation masks
  for(size_t i=0; i<masks.size(); ++i)
    {
      char filename[512];
      sprintf(filename, "mask_%d.png", (int)i);
      
      if(! dumpCVImage( masks[i], filename))
	ROS_ERROR("Problems while writing depth image");
    }
  
}

void projectCloud2Image( const sensor_msgs::PointCloud2 &cloud,
			 const sensor_msgs::CameraInfo &cam_info,
			 sensor_msgs::Image &mask )
{
  sensor_msgs::PointCloud2 cloud_proj;

  mask.height = cam_info.height;
  mask.width = cam_info.width;
  //mask.encoding = enc::TYPE_32FC1;
  mask.encoding = enc::MONO8;
  mask.is_bigendian = false;
  mask.step = mask.width;
  size_t size = mask.step * mask.height;
  mask.data.resize(size);

  Eigen::Matrix4f P;
  int rows=3, cols=4;
  for(int r=0; r<rows; ++r) 
    for(int c=0; c<cols; ++c) 
      P(r,c) = cam_info.P[r*cols+c];
    
  P(3,0) = 0;
  P(3,1) = 0;
  P(3,2) = 0;
  P(3,3) = 1;

  std::cout << "Transformation Matrix " << std::endl << P << std::endl;

  pcl_ros::transformPointCloud( P, cloud, cloud_proj);
  
  ROS_INFO("Projected Point cloud has parameters %d x %d", cloud_proj.width, cloud_proj.height);

  for(unsigned int j=0; j < cloud_proj.width; j++){
    
    float x, y, z;
	
    memcpy (&x,
	    &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[0].offset], 
	    sizeof (float));
    memcpy (&y,
	    &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[1].offset], 
	    sizeof (float));
    memcpy (&z,
	    &cloud_proj.data[j * cloud_proj.point_step + cloud_proj.fields[2].offset], 
	    sizeof (float));
    
    if( round(y/z) >= 0 && round(y/z)<mask.height && round(x/z) >= 0 && round(x/z)<mask.width) {
      int i = round(y/z) * mask.step + round(x/z);
      mask.data[i] = 255;
    } 
    
  }
}

/*! Simply pings the tabletop segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_node");
  ros::NodeHandle nh;
  ros::Publisher image_pub = nh.advertise <sensor_msgs::Image> ("output", 1);

  std::string service_name("/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  tabletop_segmenter::TabletopSegmentation segmentation_srv;
  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS && 
      segmentation_srv.response.result != segmentation_srv.response.SUCCESS_NO_RGB)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    exit(0);
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", 
	   (int)segmentation_srv.response.clusters.size());
  if (segmentation_srv.response.clusters.empty()) exit(0);
  
  
  // DEBUG of projection
  if(segmentation_srv.response.result == segmentation_srv.response.SUCCESS)
    dumpImages(segmentation_srv.response.masks, 
	       segmentation_srv.response.depth,
	       segmentation_srv.response.rgb);
  else if(segmentation_srv.response.result == segmentation_srv.response.SUCCESS_NO_RGB)
    dumpImages(segmentation_srv.response.masks, 
	       segmentation_srv.response.depth);
  
  return true;
}
