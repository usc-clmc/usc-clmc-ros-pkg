/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         image_listener.cpp

 \author       Alexander Herzog
 \date         July 14, 2012

 *********************************************************************/

#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <grasp_template_planning/image_listener.h>
#include <grasp_template_planning/grasp_planning_params.h>

#include <boost/bind.hpp>

namespace grasp_template_planning
{


ImageListener::ImageListener(ros::NodeHandle& n, rosbag::Bag& bag) : bag_(bag),
		image_received_(false), nh_(n)
{
}

ImageListener::~ImageListener() {
	// TODO Auto-generated destructor stub
	stop();
}

void ImageListener::makeSnapshot(bool wait)
{
  stop_requested_ = false;
  if (publish_thread_ != NULL)
  {
    ROS_DEBUG("grasp_template_planning::ImageListener: Thread pointer is not NULL.");
  }
  if(wait)
  {
	  processImageRequest();
  }
  else
  {
	  publish_thread_ = boost::shared_ptr<boost::thread>(
			  new boost::thread(boost::bind(&ImageListener::processImageRequest, this)));
  }
}

void ImageListener::stop()
{
  stop_requested_ = true;
  if (publish_thread_ != NULL)
    publish_thread_->join();
}

void ImageListener::processImageRequest()
{
//	std::cout << "LOCK" << std::endl;
  boost::mutex::scoped_lock lock(mutex_);
  image_received_ = false;

	GraspPlanningParams params;
//	boost::bind(&ImageListener::receiveImage,
//					  this, _1);
  ros::Subscriber img_req_sub = nh_.subscribe(
		  params.rostopicColoredImage(), 1, &ImageListener::receiveImage,
				  this);
  while (ros::ok() && !stop_requested_ && !image_received_)
  {
    ros::spinOnce();
  }

  if(!image_received_)
  {
	  ROS_DEBUG("ImageListener: Could not process image request");
  }
  else
  {
		ROS_DEBUG_STREAM("Writing image to bag: " << bag_.getFileName().c_str());
		try {
			bag_.write(params.topicObjectImage(), ros::Time::now(), image_);
		} catch (rosbag::BagIOException ex) {
			ROS_DEBUG_STREAM("Problem when writing log file " <<
					bag_.getFileName().c_str() << " : " << ex.what());

			  lock.unlock();
			return;
		}
  }

  lock.unlock();
}

void ImageListener::receiveImage(const sensor_msgs::Image& img)
{
	image_ = img;
	image_received_ = true;
}

}  //namespace
