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
#include <polled_camera/GetPolledImage.h>
#include <rosbag/bag.h>
#include <grasp_template_planning/image_listener.h>
#include <grasp_template_planning/grasp_planning_params.h>

#include <boost/bind.hpp>

namespace grasp_template_planning
{


ImageListener::ImageListener(ros::NodeHandle& n, rosbag::Bag& bag) : bag_(bag), nh_(n)
{
	grasp_in_progress_ = false;
	image_received_ = false;
}

ImageListener::~ImageListener() {
	// TODO Auto-generated destructor stub
	stop();
}


void ImageListener::startRecording()
{
  stop_requested_ = false;
  if (publish_thread_ != NULL)
  {
	ROS_DEBUG("grasp_template_planning::ImageListener: Thread pointer is not NULL.");
  }

	  publish_thread_ = boost::shared_ptr<boost::thread>(
			  new boost::thread(boost::bind(&ImageListener::processRecordingRequest, this)));
}

void ImageListener::processRecordingRequest()
{
	  boost::mutex::scoped_lock lock(mutex_);

		GraspPlanningParams params;
	//	boost::bind(&ImageListener::receiveImage,
	//					  this, _1);

		  ros::Subscriber img_req_sub = nh_.subscribe(
				  params.rostopicColoredImage(), 1, &ImageListener::receiveImage,
						  this);
//		  ros::Subscriber pickup_stat_sub = nh_.subscribe(
//		  				  params.rostopicPickupStatus(), 1, &ImageListener::receivePickupStatus,
//		  						  this);

	while(ros::ok() && !stop_requested_)
	{
//		ROS_INFO("DEBUG 1");
		  image_received_ = false;
		  while (ros::ok() && !stop_requested_ && !image_received_)
		  {

//				ROS_INFO("DEBUG 2");
//			ros::spinOnce();
			boost::xtime xt;

			boost::xtime_get(&xt, boost::TIME_UTC);

			xt.sec += 0.04;

			boost::thread::sleep(xt);

//			ROS_INFO("DEBUG 3");
		  }

//			ROS_INFO("DEBUG 4");

	  if(stop_requested_)
	  {
//			ROS_INFO("DEBUG 5");
		break;
	  }
	  else if(image_received_ && grasp_in_progress_)
	  {
			ROS_INFO("DEBUG 6");
			ROS_DEBUG_STREAM("Writing image to bag: " << bag_.getFileName().c_str());
			try {
				bag_.write(params.topicObjectImage(), ros::Time::now(), image_);
			} catch (rosbag::BagIOException ex) {
				ROS_DEBUG_STREAM("Problem when writing log file " <<
						bag_.getFileName().c_str() << " : " << ex.what());
			}

//			ROS_INFO("DEBUG 7");
	  }

//		ROS_INFO("DEBUG 8");
	}

//	ROS_INFO("DEBUG 9");


//	ROS_INFO("DEBUG 10");
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
	  processSnapshotRequest();
  }
  else
  {
	  publish_thread_ = boost::shared_ptr<boost::thread>(
			  new boost::thread(boost::bind(&ImageListener::processSnapshotRequest, this)));
  }
}

void ImageListener::stop()
{
  stop_requested_ = true;
  if (publish_thread_ != NULL)
    publish_thread_->join();
  ROS_DEBUG("Stopped Image Listener");
}

void ImageListener::processSnapshotRequest()
{
//	std::cout << "LOCK" << std::endl;
  boost::mutex::scoped_lock lock(mutex_);
  image_received_ = false;

	GraspPlanningParams params;
//	boost::bind(&ImageListener::receiveImage,
//					  this, _1);
  {
//	  ros::Subscriber img_req_sub = nh_.subscribe(
//			  params.rostopicProsilicaImage().append("/image_raw"), 1, &ImageListener::receiveImage,
//					  this);
	  ros::Subscriber img_req_sub = nh_.subscribe(
			  params.rostopicColoredImage(), 1, &ImageListener::receiveImage,
					  this);
//	  ros::ServiceClient prosilica_poll = nh_.serviceClient<actionlib_msgs::GoalStatus>(
//			  params.serviceNameImagePoll());
//	  polled_camera::GetPolledImage poll_srv;
//	  poll_srv.request.response_namespace = params.rostopicProsilicaImage();
//	  poll_srv.request.binning_x = 0;
//	  poll_srv.request.binning_y = 0;
//	  poll_srv.request.roi = sensor_msgs::RegionOfInterest();
//	  poll_srv.request.roi.x_offset = 0;
//	  poll_srv.request.roi.y_offset = 0;
//	  poll_srv.request.roi.height = 2048;
//	  poll_srv.request.roi.width = 1536;
//	  poll_srv.request.roi.do_rectify = false;
//	  poll_srv.request.timeout = ros::Duration(2.0);
//
//	  if (!prosilica_poll.call(poll_srv))
//	  {
//	    ROS_INFO("Waiting for prosilica poll service to come up tiemd out");
//
//	    return;
//	  }

//	  ROS_INFO_STREAM("camera poll response: " << poll_srv.response.success
//			  << poll_srv.response.status_message);
	  ros::Time t_start_while = ros::Time::now();
	  ros::Duration dur_while = ros::Duration(0.0);
	  ros::Duration dur_while_max = ros::Duration(2.0);
	  while (
//			  poll_srv.response.success == true &&
			  ros::ok() && !stop_requested_
			  && !image_received_ && dur_while < dur_while_max)
	  {
		ros::spinOnce();
		dur_while = ros::Time::now() - t_start_while;
	  }
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


			return;
		}
  }


}

void ImageListener::receivePickupStatus(const actionlib_msgs::GoalStatus& state)
{
	grasp_in_progress_ = state.status == 1;

}

void ImageListener::receiveImage(const sensor_msgs::Image& img)
{
	image_ = img;
	image_received_ = true;
}

}  //namespace
