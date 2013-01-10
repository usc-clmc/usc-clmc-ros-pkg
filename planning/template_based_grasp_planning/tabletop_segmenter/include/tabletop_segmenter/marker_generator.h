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

// Author(s): Marius Muja and Matei Ciocarlie

#ifndef _MARKER_GENERATOR_H_
#define _MARKER_GENERATOR_H_

#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

namespace enc = sensor_msgs::image_encodings;

namespace tabletop_segmenter {

  //! A convenience class for generating markers based on various clustering and fitting data
  /*! Just a place to group all the different debug marker generators
    so they don't polute other classes.
  */
  class MarkerGenerator {
  public:
    //! Create a line strip marker that goes around a detected table
    static visualization_msgs::Marker getTableMarker(float xmin, float xmax, float ymin, float ymax);
    //! A marker with all the points in a cloud in a random color
    static visualization_msgs::Marker getCloudMarker(const sensor_msgs::PointCloud2 &cloud);
    //! A marker drawing points at each grid point indicating height
    static visualization_msgs::Marker getHeightMarker(const sensor_msgs::Image &heightmap,
						      const sensor_msgs::Image &costmap, 
						      const geometry_msgs::Point &origin);
    
  };

  /*!
    It is the responsibility of the caller to set the appropriate pose for the marker so that
    it shows up in the right reference frame.
  */
  visualization_msgs::Marker MarkerGenerator::getCloudMarker(const sensor_msgs::PointCloud2 &cloud)
    {
      static bool first_time = true;
      if (first_time) {
	srand ( time(NULL) );
	first_time = false;
      }

      //create the marker
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration();

      marker.type = visualization_msgs::Marker::POINTS;
      marker.scale.x = 0.002;
      marker.scale.y = 0.002;
      marker.scale.z = 1.0;

      marker.color.r = ((double)rand())/RAND_MAX;
      marker.color.g = ((double)rand())/RAND_MAX;
      marker.color.b = ((double)rand())/RAND_MAX;
      marker.color.a = 1.0;
      
      std::cout << "Width of point cloud " << cloud.width << std::endl;
      std::cout << "point step " << cloud.point_step << std::endl;
      std::cout << "offset x " << cloud.fields[0].offset << std::endl;
      std::cout << "offset y " << cloud.fields[1].offset << std::endl;
      std::cout << "offset z " << cloud.fields[2].offset << std::endl;

      for(size_t i=0; i<cloud.width; i++) {
	float x, y, z;
	memcpy(&x, &cloud.data[i * cloud.point_step + cloud.fields[0].offset],sizeof(float));
	memcpy(&y, &cloud.data[i * cloud.point_step + cloud.fields[1].offset],sizeof(float));
	memcpy(&z, &cloud.data[i * cloud.point_step + cloud.fields[2].offset],sizeof(float));

	geometry_msgs::Point p;

	p.x = x;
	p.y = y;
	p.z = z;
	marker.points.push_back(p);
      }

      //the caller must decide the header; we are done here
      return marker;
    }

  /*!
    It is the responsibility of the caller to set the appropriate pose for the marker so that
    it shows up in the right reference frame.
  */
  visualization_msgs::Marker MarkerGenerator::getHeightMarker(const sensor_msgs::Image &heightmap, 
							      const sensor_msgs::Image &costmap, 
							      const geometry_msgs::Point &origin)
    {
      float mm_2_m = 1.0f/1000.0f;

      static bool first_time = true;
      if (first_time) {
	srand ( time(NULL) );
	first_time = false;
      }

      //create the marker
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration();

      marker.type = visualization_msgs::Marker::POINTS;
      marker.scale.x = 0.002;
      marker.scale.y = 0.002;
      marker.scale.z = 1.0;

      
      marker.color.r = ((double)rand())/RAND_MAX;
      marker.color.g = ((double)rand())/RAND_MAX;
      marker.color.b = ((double)rand())/RAND_MAX;
      marker.color.a = 1.0;
      
      // copy heightmap image
      cv::Mat cv_height;
      cv_bridge::CvImagePtr cv_height_ptr;
      sensor_msgs::Image::ConstPtr height_ptr
	= boost::make_shared<sensor_msgs::Image>(heightmap);
      
      try {
	cv_height_ptr 
	  = cv_bridge::toCvCopy(height_ptr,
				enc::TYPE_32FC1);
	
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("Shit cv_bridge exception: %s", e.what());
	return marker;
      }
      cv_height = cv_height_ptr->image;

      // copy costmap image
      cv::Mat cv_cost;
      cv_bridge::CvImagePtr cv_cost_ptr;
      sensor_msgs::Image::ConstPtr cost_ptr
	= boost::make_shared<sensor_msgs::Image>(costmap);
      
      try {
	cv_cost_ptr 
	  = cv_bridge::toCvCopy(cost_ptr,
				enc::TYPE_32FC1);
	
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("Shit cv_bridge exception: %s", e.what());
	return marker;
      }
      cv_cost = cv_cost_ptr->image;

      std::cout << "Width and height of height map " 
		<< cv_height.rows << " " 
		<< cv_height.cols << std::endl;

      assert(cv_height.rows == cv_cost.rows && cv_height.cols == cv_cost.cols);
      
      for(int i=0; i<cv_height.rows; i++) {
	float* height_i = cv_height.ptr<float>(i);
	float* cost_i = cv_cost.ptr<float>(i);
	for(int j=0; j<cv_height.cols; j++) {
	  if(cost_i[j]>=0) {
	    
	    geometry_msgs::Point p;
	    p.x = (j-origin.x)*mm_2_m;
	    p.y = (i-origin.y)*mm_2_m;
	    p.z = height_i[j]*mm_2_m;
	    marker.points.push_back(p);
	    
	    std_msgs::ColorRGBA c;
	    c.r = cost_i[j];
	    c.g = 0.0;
	    c.b = 1.0-cost_i[j];
	    c.a = 1.0;
	    marker.colors.push_back(c);
	  }
	}
      }


      assert(marker.points.size()==marker.colors.size());
      std::cout << "Number of points " << marker.points.size() 
		<< " and number of RGB values " << marker.colors.size() << std::endl;

      //the caller must decide the header; we are done here
      return marker;
    }


}//namespace

#endif
