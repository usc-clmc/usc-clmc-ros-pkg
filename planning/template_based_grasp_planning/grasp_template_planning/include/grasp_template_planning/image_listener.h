/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         image_listener.h

 \author       Alexander Herzog
 \date         July 14, 2012

 *********************************************************************/

#ifndef IMAGELISTENER_H_
#define IMAGELISTENER_H_

#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <actionlib_msgs/GoalStatus.h>

namespace grasp_template_planning
{

class ImageListener {
public:
	ImageListener(ros::NodeHandle& n, rosbag::Bag& bag);
	virtual ~ImageListener();
	void makeSnapshot(bool wait=false);

	void startRecording();
	void stop();

private:
	  boost::mutex mutex_;
	  boost::shared_ptr<boost::thread> publish_thread_;
	  volatile bool stop_requested_;

		ros::NodeHandle& nh_;
		sensor_msgs::Image image_;
		rosbag::Bag& bag_;
		  bool image_received_, grasp_in_progress_;

	  void processSnapshotRequest();
	  void processRecordingRequest();

	  void receiveImage(const sensor_msgs::Image& img);
	  void receivePickupStatus(const actionlib_msgs::GoalStatus& state);
};

}  //namespace
#endif /* IMAGELISTENER_H_ */
