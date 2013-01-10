/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks	...

 \file		heightmap_sampling.h

 \author	Alexander Herzog
 \date		April 1 2012

 *********************************************************************/

#ifndef HEIGHTMAP_SAMPLING_H_
#define HEIGHTMAP_SAMPLING_H_

#include <vector>
#include <boost/shared_ptr.hpp>

#include <Eigen/Eigen>
#include <pcl/Vertices.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/grasp_template_params.h>
#include <grasp_template/height_value_extractor.h>
#include <grasp_template/template_heightmap.h>
#include <grasp_template/grasp_template.h>

namespace grasp_template
{

struct HsIterator
{

public:

  HsIterator(unsigned int elements, double angle_step_size = M_PI / 6);

  double current_angle_;
  unsigned int index_;
  unsigned int elements_;
  double angle_step_size_;

  void inc();
  void setIndex(unsigned int index);
  bool passedLast();
};

class HeightmapSampling : public GraspTemplateParams
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HeightmapSampling();
  HeightmapSampling(const Eigen::Vector3d& viewpoint_trans, const Eigen::Quaterniond& viewpoint_quat);

  Eigen::Quaterniond viewp_rot_;
  Eigen::Vector3d viewp_trans_;
//  std::string viewpoint_frame_id_;
  geometry_msgs::Pose table_pose_;

  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > convex_hull_points_;

  const pcl::PointCloud<pcl::PointXYZ>& getSearchPoints() const {return *search_points_;};
  const pcl::PointCloud<pcl::PointXYZ>& getPointCloud() const {return *point_cloud_;};
  const pcl::PointCloud<pcl::PointXYZ>& getConvexHullPoints() const {return *convex_hull_points_;};
  const std::vector<pcl::Vertices>& getConvexHullVertices() const {return convex_hull_vertices_;};
  const pcl::PointCloud<pcl::Normal>& getNormals() const {return normals_;};
  HsIterator getIterator() const {return HsIterator(search_points_->size());};
//  std::string getViewpointFrameId() const {return viewpoint_frame_id_;};
  std::string getTemplateFrameId() const {return point_cloud_->header.frame_id;};

  void initialize(const pcl::PointCloud<pcl::PointXYZ>& cluster, const geometry_msgs::Pose& table);
  bool generateTemplateOnHull(GraspTemplate& templt, const Eigen::Vector3d& ref, double z_angle = 0);
  bool generateTemplateOnHull(GraspTemplate& templt, const HsIterator& it);
  bool generateTemplate(GraspTemplate& templt, const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation);
  void addTable(grasp_template::GraspTemplate& t) const;
//  void pclToSensorMsg(const pcl::PointCloud<pcl::PointXYZ>& in, sensor_msgs::PointCloud2& out) const;
  visualization_msgs::Marker getVisualizationNormals(const std::string& ns, const
      std::string& frame_id, int id = 1) const;

private:

  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > point_cloud_;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > search_points_;
  std::vector<pcl::Vertices> convex_hull_vertices_;
  pcl::PointCloud<pcl::Normal> normals_;
//  std::string point_cloud_frame_id_;
  HeightValueExtractor img_ss_;

  bool calculateNormalsFromHullSurface();
  bool calculateConvexHull();
};

} //namespace
#endif /* HEIGHTMAP_SAMPLING_H_ */
