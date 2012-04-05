/*
 * object_detection_listener.h
 *
 *  Created on: Feb 11, 2011
 *      Author: herzog
 */

#ifndef OBJECT_DETECTION_LISTENER_H_
#define OBJECT_DETECTION_LISTENER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tabletop_object_detector/TabletopDetection.h>


namespace grasp_test
{

class ObjectDetectionListener
{
public:
	void connectToObjectDetector(ros::NodeHandle& n);
	bool fetchClusterFromObjectDetector();
	boost::shared_ptr<const sensor_msgs::PointCloud> getCluster() const;
private:
	ros::ServiceClient cluster_client_;
	tabletop_object_detector::TabletopDetection tod_communication_;
	boost::shared_ptr<const sensor_msgs::PointCloud> object_cluster_;  //pointer to first detected object

};

bool ObjectDetectionListener::fetchClusterFromObjectDetector()
{

	tod_communication_.request.return_clusters = true;
	tod_communication_.request.return_models = false;

	if(!cluster_client_.call(tod_communication_))
	{
		ROS_INFO("Could not get object cluster from tabletop_object_detector.");
		return false;
	}

	if(tod_communication_.response.detection.clusters.size() <= 0)
	{
		ROS_INFO("tabletop_object_detector could not recognize any object.");
		return false;
	}else
	{
		ROS_INFO("got %d cluster(s) from object-detector", static_cast<int>(tod_communication_.
				response.detection.clusters.size()) );
	}

	object_cluster_.reset(&tod_communication_.response.detection.clusters[0]);

	return true;
}

void ObjectDetectionListener::connectToObjectDetector(ros::NodeHandle& n)
{
	cluster_client_ = n.serviceClient<tabletop_object_detector::TabletopDetection>(
			"/object_detection");
}


boost::shared_ptr<const sensor_msgs::PointCloud> ObjectDetectionListener::getCluster() const
{
	return object_cluster_;
}

}//  namespace

#endif /* OBJECT_DETECTION_LISTENER_H_ */
