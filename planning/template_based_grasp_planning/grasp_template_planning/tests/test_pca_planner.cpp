/*
 * test_pca_planner.cpp
 *
 *  Created on: Jul 29, 2012
 *      Author: herzog
 */

#include <Eigen/Eigen>
#include <boost/smart_ptr/intrusive_ptr.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/common/pca.h>

#include <grasp_template_planning/object_detection_listener.h>
#include <grasp_template_planning/pca_planning_pipe.h>
#include <grasp_template_planning/pca_grasp_container.h>
#include <grasp_template_planning/pca_planning_pipe.h>

using namespace std;
using namespace pcl;
using namespace grasp_template_planning;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_pca_planner");
	ros::NodeHandle nh;

	boost::shared_ptr<PCAGraspContainer> grasp_pool;
	boost::shared_ptr<TemplateMatching> grasp_pool_super;
	ObjectDetectionListener obj_listener;

	obj_listener.connectToObjectDetector(nh);
	obj_listener.fetchClusterFromObjectDetector();
	sensor_msgs::PointCloud2 object;
	obj_listener.getClusterPC2(object);

	  string demos_path, demo_lib_path, failures_path, successes_path, log_path;
	  ros::param::get("~grasp_library_file", demo_lib_path);
	  ros::param::get("~grasp_demonstrations_path", demos_path);
	  ros::param::get("~library_negatives_path", failures_path);
	  ros::param::get("~library_positives_path", successes_path);
	  ros::param::get("~log_data_path", log_path);


	boost::shared_ptr<PCAPlanningPipe> planner;
	planner.reset(new PCAPlanningPipe(demos_path, demo_lib_path, failures_path,
			successes_path, log_path));

	planner->initialize(object, obj_listener.getTableFrame().pose);
	planner->planGrasps(grasp_pool_super);
	grasp_pool = boost::dynamic_pointer_cast<PCAGraspContainer,
			TemplateMatching>(grasp_pool_super);


	  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped> ("pca_transform", 100);

	  for(unsigned int i = 0; i < grasp_pool->size() && ros::ok(); ++i)
	  {
		  pub.publish(grasp_pool->getGrasp(i).gripper_pose);
		  ROS_INFO_STREAM("publishing grasp: " << grasp_pool->getGrasp(i).gripper_pose);
		  ros::Rate(3).sleep();
	  }

	return 0;
}
