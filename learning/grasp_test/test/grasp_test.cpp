#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/grasp_template.h>
#include <grasp_template/template_generator.h>
#include <grasp_test/object_detection_listener.h>

using namespace std;
using namespace grasp_template;
using namespace grasp_test;
using namespace visualization_msgs;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasp_test");
	ros::NodeHandle n;
	ros::Publisher templt_marker_pub = n.advertise<Marker>("templates_viz", 100);
	ros::Rate viz_rate(1);
	ros::Publisher generator_rslt_pub = n.advertise<Marker>("templt_cluster_viz", 10);

	/* get object cluster */
	ObjectDetectionListener object_detection;
	object_detection.connectToObjectDetector(n);
	if(!object_detection.fetchClusterFromObjectDetector())
	{
		ROS_INFO("no results from object detector");

		return 0;
	}
	boost::shared_ptr<const sensor_msgs::PointCloud> cluster =
			object_detection.getCluster();
	if(cluster == NULL)
	{
		ROS_INFO("got no clusters from object detector");

		return 0;
	}

	/* transform point-cloud for template_generator */
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcl_cluster;
	pcl_cluster.reset(new pcl::PointCloud<pcl::PointXYZ>());

	pcl_cluster->points.clear();
	for(vector<geometry_msgs::Point32>::const_iterator it = cluster->points.begin();
			it != cluster->points.end(); ++it)
	{
		pcl_cluster->push_back(*(new pcl::PointXYZ(it->x, it->y, it->z)));
	}

	/* compute templates */
	TemplateGenerator generator;
	generator.setPointCloud(pcl_cluster);
	generator.generateTemplates();

	/* visualize */
	bool viz_templt = true;

	if(generator.getTemplates().empty())
	{
		ROS_INFO("no templates available");
		viz_templt = false;
	}else
	{
		ROS_INFO("publishing %d templates", static_cast<int>(generator.getTemplates().size()) );
		viz_templt = true;
	}

	vector<GraspTemplate>::const_iterator templt_it =
			generator.getTemplates().begin();
	while(ros::ok())
	{
		if(viz_templt)
		{
			/* visualize templates */
			templt_marker_pub.publish(generator.getTemplates().begin()->getVisualization());

			if(templt_it == generator.getTemplates().end())
			{
				templt_it = generator.getTemplates().begin();
			}else
			{
			  templt_it++;
			}

			/* visualize cluster */
			generator_rslt_pub.publish(generator.getVisualizationPointCloud());
//			generator_rslt_pub.publish(generator.getVisualizationNormals());

			viz_rate.sleep();
		}
	}


	return 0;
}
