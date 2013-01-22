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

#include <list>
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

  //sort according to angle between middle finger and table
  list<GraspAnaComparableWrapper, Eigen::aligned_allocator<GraspAnaComparableWrapper> > grasp_list;
  for(vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> >::const_iterator it=pca_pool->grasps_.begin();
		  it!=pca_pool->grasps_.end();it++)
  {
	  grasp_list.push_back(GraspAnaComparableWrapper(*it, table_frame_));
  }
  cout << grasp_list.size() << ", " << grasp_list.begin()->ana_.demo_filename;
  grasp_list.sort();
  cout << grasp_list.size() << ", " << grasp_list.begin()->ana_.demo_filename;

  ROS_ASSERT(grasp_list.size() == pca_pool->grasps_.size());
  pca_pool->grasps_.clear();
  for(list<GraspAnaComparableWrapper, Eigen::aligned_allocator<GraspAnaComparableWrapper> >::iterator it=grasp_list.begin();
		  it!=grasp_list.end();it++)
  {
//	  it->ana_;
	  GraspAnalysis tmp = it->ana_;
	  cout << tmp.demo_filename;
	  pca_pool->grasps_.push_back(tmp);
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


bool GraspAnaComparableWrapper::operator <(const GraspAnaComparableWrapper& b) const
{
	Eigen::Matrix4d table_transf = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond table_orient;
	table_orient.x() = table_pose_.orientation.x;
	table_orient.y() = table_pose_.orientation.y;
	table_orient.z() = table_pose_.orientation.z;
	table_orient.w() = table_pose_.orientation.w;
	table_transf.block<3,3>(0,0) = table_orient.toRotationMatrix();
//	table_transf(0,3) = table_pose_.position.x;
//	table_transf(1,3) = table_pose_.position.y;
//	table_transf(2,3) = table_pose_.position.z;

	Eigen::Matrix4d a_grasp_transf = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond a_grasp_orient;
	a_grasp_orient.x() = ana_.gripper_pose.pose.orientation.x;
	a_grasp_orient.y() = ana_.gripper_pose.pose.orientation.y;
	a_grasp_orient.z() = ana_.gripper_pose.pose.orientation.z;
	a_grasp_orient.w() = ana_.gripper_pose.pose.orientation.w;
	a_grasp_transf.block<3,3>(0,0) = a_grasp_orient.toRotationMatrix();
//	a_grasp_transf(0,3) = a.ana_.gripper_pose.pose.position.x;
//	a_grasp_transf(1,3) = a.ana_.gripper_pose.pose.position.y;
//	a_grasp_transf(2,3) = a.ana_.gripper_pose.pose.position.z;

	Eigen::Matrix4d b_grasp_transf = Eigen::Matrix4d::Identity();
	Eigen::Quaterniond b_grasp_orient;
	b_grasp_orient.x() = b.ana_.gripper_pose.pose.orientation.x;
	b_grasp_orient.y() = b.ana_.gripper_pose.pose.orientation.y;
	b_grasp_orient.z() = b.ana_.gripper_pose.pose.orientation.z;
	b_grasp_orient.w() = b.ana_.gripper_pose.pose.orientation.w;
	b_grasp_transf.block<3,3>(0,0) = b_grasp_orient.toRotationMatrix();
//	b_grasp_transf(0,3) = b.ana_.gripper_pose.pose.position.x;
//	b_grasp_transf(1,3) = b.ana_.gripper_pose.pose.position.y;
//	b_grasp_transf(2,3) = b.ana_.gripper_pose.pose.position.z;

	double cos_a = a_grasp_transf.block<3,1>(0,1).dot(table_transf.block<3,1>(0,2));
	double cos_b = b_grasp_transf.block<3,1>(0,1).dot(table_transf.block<3,1>(0,2));

	return std::abs(cos_a) < std::abs(cos_b);
};

//virtual bool PCAPlanningPipe::logPlannedGrasps(const TemplateMatching& pool, unsigned int max_num_grasps);
//virtual bool PCAPlanningPipe::logGraspResult(const GraspAnalysis& res_ana, const TemplateMatching& pool, int rank);
//virtual bool PCAPlanningPipe::writeLogToBag(){};

}  //namespace
