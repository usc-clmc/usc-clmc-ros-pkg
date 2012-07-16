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

#include <rosbag/bag.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace grasp_template_planning
{

class ImageListener {
public:
	ImageListener(ros::NodeHandle& n, rosbag::Bag& bag);
	virtual ~ImageListener();
	void makeSnapshot(bool wait=false);

private:
	  boost::mutex mutex_;
	  bool image_received_;
	  boost::shared_ptr<boost::thread> publish_thread_;
	  volatile bool stop_requested_;

		ros::NodeHandle& nh_;
		sensor_msgs::Image image_;
		rosbag::Bag& bag_;

	  void stop();
	  void processImageRequest();

	  void receiveImage(const sensor_msgs::Image& img);
};

}  //namespace
#endif /* IMAGELISTENER_H_ */
