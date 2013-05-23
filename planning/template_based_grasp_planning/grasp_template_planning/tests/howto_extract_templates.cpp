/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         howto_extract_templates.cpp

 \author       Alexander Herzog
 \date         April 15, 2013

 *********************************************************************/

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <usc_utilities/file_io.h>

#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

using namespace std;
using namespace ros;
using namespace usc_utilities;
using namespace grasp_template_planning;
using namespace grasp_template;

int main(int argc, char** argv)
{
	init(argc, argv, "howto_extract_templates");
	NodeHandle nh;

	// define path to the bagfile and topic names of messages
	// that we want to pull out from the bag file
	const std::string BAGFILE_NAME = "/tmp/example_grasp_log.bag";
	const std::string SNAPSHOT_TOPIC = "/grasp_planning_image_object";
	const std::string LOG_TOPIC = "/grasp_planning_log";

	// read snapshot of the object from the bagfile
	sensor_msgs::Image table_snapshot;
	FileIO<sensor_msgs::Image>::readFromBagFile(table_snapshot, SNAPSHOT_TOPIC, BAGFILE_NAME);

	// read execution log from the bagfile
	vector<GraspLog> grasp_trial_log;
	FileIO<GraspLog>::readFromBagFile(grasp_trial_log, LOG_TOPIC, BAGFILE_NAME);

	// this is disgusting, but the content of GraspLog is spread over 3 messages
	// in the following we sort the messages
	vector<const GraspLog*> grasp_trial_log_sorted;
	grasp_trial_log_sorted.resize(3);
	for(int i=0; i<grasp_trial_log.size();i++)
	{
		cout << "read log message with seq nr " << grasp_trial_log[i].seq << endl;
		const int seq_nr = grasp_trial_log[i].seq;
		grasp_trial_log_sorted[seq_nr] = &(grasp_trial_log[i]);

	}

	// here we pull out what is required for heightmap computation
	sensor_msgs::PointCloud2 object_cloud = grasp_trial_log_sorted[0]->target_object;
	geometry_msgs::Pose table_frame = grasp_trial_log_sorted[1]->table_frame;
	geometry_msgs::Point vp = grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.position;
	geometry_msgs::Quaternion vo = grasp_trial_log_sorted[1]->applied_grasp.viewpoint_transform.pose.orientation;

	// convert some data structures
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(object_cloud, pcl_cloud);
	Eigen::Vector3d viewpoint_pos(vp.x, vp.y, vp.z);
	Eigen::Quaterniond viewpoint_rot(vo.w, vo.x, vo.y, vo.z);

	// this guy generates heightmaps from point clouds
	HeightmapSampling heightmap_computation(viewpoint_pos, viewpoint_rot);
	heightmap_computation.initialize(pcl_cloud, table_frame);

	// unfortunately you have to pass an eigen allocator if you use STL containers
	vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > template_container;

	// HsIterator is implemented in the same header where I implemented HeightmapSampling
	for (HsIterator it = heightmap_computation.getIterator();
		 !it.passedLast(); it.inc())
	{
	  GraspTemplate t;

	  // check the overloaded functions of generateTemplateOnHull and generateTemplate
	  // they provide more options to extract templates
	  heightmap_computation.generateTemplateOnHull(t, it);
	  template_container.push_back(t);
	}

	return 0;
}




