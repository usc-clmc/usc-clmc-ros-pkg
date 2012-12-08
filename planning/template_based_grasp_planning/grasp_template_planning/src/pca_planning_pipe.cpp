/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         pca_planning_pipe.cpp

 \author       Alexander Herzog
 \date         July 29, 2012

 *********************************************************************/

#include <math.h>
#include <boost/smart_ptr/intrusive_ptr.hpp>
#include <Eigen/Eigen>
#include <pcl/common/pca.h>

#include <grasp_template_planning/pca_grasp_container.h>
#include <grasp_template_planning/pca_planning_pipe.h>

using namespace std;
using namespace Eigen;
using namespace boost;
using namespace pcl;
using namespace grasp_template;

namespace grasp_template_planning
{


PCAPlanningPipe::PCAPlanningPipe(const string& demo_path,
	    const string& library_path, const string& failures_path,
	    const string& successes_path, const string& log_data_path) :
	    PlanningPipeline(demo_path, library_path, failures_path,
	    	    successes_path,  log_data_path)
{

};
PCAPlanningPipe::~PCAPlanningPipe(){};


void PCAPlanningPipe::planGrasps(boost::shared_ptr<TemplateMatching>& pool) const
{
  boost::shared_ptr<PointCloud<PointXYZ> > cloud_pcl;
  cloud_pcl.reset(new PointCloud<PointXYZ>());
  fromROSMsg(target_object_, *cloud_pcl);
  pcl::PCA<pcl::PointXYZ> pca_handler_;
  pca_handler_.setInputCloud(cloud_pcl);

//  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
//  for(unsigned int i = 0; i < cloud_pcl->size(); ++i)
//  {
//	  mean.x() += cloud_pcl->points[i].x;
//	  mean.y() += cloud_pcl->points[i].y;
//	  mean.z() += cloud_pcl->points[i].z;
//  }
//
//  mean = (1 / cloud_pcl->size()) * mean;

  Eigen::Matrix4f trans_to_pc_comps, trans_to_pc_comps_inv;
  trans_to_pc_comps = trans_to_pc_comps_inv = Eigen::Matrix4f::Zero();
  trans_to_pc_comps(3,3) = trans_to_pc_comps_inv(3,3) = 1;
  trans_to_pc_comps.block<3, 3>(0, 0) = pca_handler_.getEigenVectors();
  if( trans_to_pc_comps.block<3, 3>(0, 0).determinant() < 0)
  {
	  trans_to_pc_comps.block<3,1>(0,0) = -1 * trans_to_pc_comps.block<3,1>(0,0);
  }
  trans_to_pc_comps.block<3, 1>(0, 3) = pca_handler_.getMean().block<3, 1>(0, 0);

  std::cout << "eigen values: " << pca_handler_.getEigenValues() << std::endl;

  trans_to_pc_comps_inv.block<3, 3>(0, 0) = trans_to_pc_comps.block<3, 3>(0, 0).transpose();
  trans_to_pc_comps_inv.block<3, 1>(0, 3) = -trans_to_pc_comps.block<3, 1>(0, 3);

  pool.reset(new PCAGraspContainer(this));
  boost::shared_ptr<PCAGraspContainer> pca_pool;
  pca_pool = dynamic_pointer_cast<PCAGraspContainer, TemplateMatching>(pool);
  pca_pool->grasps_.clear();

  Eigen::Matrix4f gripper_orientation_1;
  gripper_orientation_1 = Eigen::Matrix4f::Identity();
//  gripper_orientation_1.block<2, 2>(0, 0) << 0, 1, -1, 0;
  Eigen::Matrix4f gripper_orientation_2;
  gripper_orientation_2 = Eigen::Matrix4f::Identity();
  gripper_orientation_2.block<2, 2>(0, 0) << -1, 0, 0, -1;

  const unsigned int rot_steps = 32;
  for(unsigned int i = 0; i < rot_steps; ++i)
  {
	  double angle = -M_PI + (2 * M_PI / rot_steps * i);

	  Eigen::Matrix4f rot_about_main;
	  rot_about_main = Eigen::Matrix4f::Identity();
	  rot_about_main.block<2, 2>(1, 1) << cos(angle), -sin(angle), sin(angle), cos(angle);

	  geometry_msgs::PoseStamped pose1, pose2;
	  Eigen::Matrix4f preshape_1 = trans_to_pc_comps * gripper_orientation_1 * rot_about_main;
	  Eigen::Vector3f shift_dir = -(preshape_1).block<3, 1>(0, 2);
	  Eigen::Vector3f cluster_mean = trans_to_pc_comps.block<3, 1>(0, 3);
	  Eigen::Vector3f shift_back;
	  shiftOutOfConvexHull(cluster_mean, shift_dir, shift_back);
	  preshape_1.block<3, 1>(0, 3) += shift_back;
	  poseEigenToTf(preshape_1, pose1);

	  Eigen::Matrix4f preshape_2 = trans_to_pc_comps * gripper_orientation_2 * rot_about_main;
	  shift_dir = -(preshape_2).block<3, 1>(0, 2);
	  shiftOutOfConvexHull(cluster_mean, shift_dir, shift_back);
	  preshape_2.block<3, 1>(0, 3) += shift_back;
	  poseEigenToTf(preshape_2, pose2);

	  GraspAnalysis ana;
	  DoubleVector fingerpos;
	  fingerpos.vals = std::vector<double>(4);
	  fingerpos.vals[0] = 0.024544;
	  fingerpos.vals[1] = 0.620342;
	  fingerpos.vals[2] = 0.823146;
	  fingerpos.vals[3] = 0.660103;
	  ana.fingerpositions = fingerpos;
	  setIdAndTime(ana);
	  ana.grasp_success = 0.5;

	  ana.gripper_pose = pose1;
	  pca_pool->grasps_.push_back(ana);
	  ana.gripper_pose = pose2;
	  pca_pool->grasps_.push_back(ana);
  }




}

void PCAPlanningPipe::shiftOutOfConvexHull(const Eigen::Vector3f& mean,
		const Eigen::Vector3f& shift_dir, Eigen::Vector3f& shift_back) const
{
	shift_back = shift_dir;
	shift_back.normalize();

	double max_a = 0;

	for(unsigned int i = 0; i < templt_generator_->convex_hull_points_->points.size(); ++i)
	{
		Eigen::Vector3f c;
		c.x() = templt_generator_->convex_hull_points_->points[i].x;
		c.y() = templt_generator_->convex_hull_points_->points[i].y;
		c.z() = templt_generator_->convex_hull_points_->points[i].z;

		double a_i = shift_back.dot(c - mean);

		if(a_i > max_a)
			max_a = a_i;
	}

	shift_back = max_a*shift_back;
	cout << "shift_back = " << shift_back << " with a=" << max_a << endl;
}

void PCAPlanningPipe::poseEigenToTf(const Eigen::Matrix4f& transform, geometry_msgs::PoseStamped& pose) const
{
	pose.header.frame_id = templt_generator_->frameBase();
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = transform(0, 3);
	pose.pose.position.y = transform(1, 3);
	pose.pose.position.z = transform(2, 3);

	  Eigen::Quaternionf rot(transform.block<3, 3>(0, 0));
//	  std::cout << "rotation matrix diff: " << rot.toRotationMatrix() - transform.block<3, 3>(0, 0) << std::endl;
//	  std::cout << "rot_tmp.determinant: " << transform.block<3, 3>(0, 0).determinant() << std::endl;

	  pose.pose.orientation.w = rot.w();
	  pose.pose.orientation.x = rot.x();
	  pose.pose.orientation.y = rot.y();
	  pose.pose.orientation.z = rot.z();
}

//virtual bool PCAPlanningPipe::logPlannedGrasps(const TemplateMatching& pool, unsigned int max_num_grasps);
//virtual bool PCAPlanningPipe::logGraspResult(const GraspAnalysis& res_ana, const TemplateMatching& pool, int rank);
//virtual bool PCAPlanningPipe::writeLogToBag(){};

}  //namespace