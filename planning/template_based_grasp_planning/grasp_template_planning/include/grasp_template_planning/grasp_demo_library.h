/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_demo_library.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef GRASP_DEMO_LIBRARY_H_
#define GRASP_DEMO_LIBRARY_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <grasp_template_planning/SimpleLabel.h>
#include <grasp_template_planning/GraspAnalysis.h>
#include <grasp_template_planning/DoubleVector.h>
#include <grasp_template_planning/grasp_planning_params.h>

namespace grasp_template_planning
{

typedef std::map<std::string, std::pair<ros::Time, SimpleLabel> > GraspDemoEventMap;
typedef std::map<ros::Time, sensor_msgs::PointCloud2> GraspDemoObjectMap;
typedef std::map<ros::Time, geometry_msgs::PoseStamped> GraspDemoPoseMap;
typedef std::map<ros::Time, DoubleVector> GraspDemoVecDoubleMap;

class GraspDemoLibrary : private GraspPlanningParams
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspDemoLibrary(const std::string& grasp_demonstrations_path,
      const std::string& grasp_library_file);

  boost::shared_ptr<const GraspDemoEventMap> getEvents() const{return static_cast<
          boost::shared_ptr<const GraspDemoEventMap> >(events_);};
  boost::shared_ptr<const GraspDemoObjectMap> getObjects() const{return static_cast<
          boost::shared_ptr<const GraspDemoObjectMap> >(objects_);};
  boost::shared_ptr<const GraspDemoPoseMap> getGripperPoses() const{return static_cast<
          boost::shared_ptr<const GraspDemoPoseMap> >(gripper_poses_);};
  boost::shared_ptr<const GraspDemoPoseMap> getViewpointTransforms() const{return static_cast<
          boost::shared_ptr<const GraspDemoPoseMap> >(viewpoint_transforms_);};
  boost::shared_ptr<const GraspDemoPoseMap> getTablePoses() const{return static_cast<
          boost::shared_ptr<const GraspDemoPoseMap> >(table_poses_);};
  boost::shared_ptr<const GraspDemoVecDoubleMap> getFingerpositions() const{return static_cast<
          boost::shared_ptr<const GraspDemoVecDoubleMap> >(fingerpositions_);};
  boost::shared_ptr<const std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > getAnalysisMsgs() const{return static_cast<
          boost::shared_ptr<const std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > >(analysis_msgs_);};
  const std::string& getDemoFilename() const{return demo_filename_;};
  bool getAllDemonstrationFilenames(std::vector<std::string>& container) const;

  void dropDemonstrationData();
  void dropLibraryData();
  bool isDemonstrationLoaded() const;
  bool removeLibraryFileFromFilesystem();
  bool loadDemonstration(const std::string& filename);
  bool isIgnored(GraspAnalysis::ConstPtr ana) const;
  bool loadLibrary();
  bool addAnalysisToLib(const GraspAnalysis& msg);
  void ignore(const std::string& filename) {ignored_.push_back(filename);};

private:

  std::string grasp_demonstrations_path_;
  std::string grasp_library_file_;
  std::string demo_filename_;
  std::vector<std::string> ignored_;

  boost::shared_ptr<GraspDemoEventMap> events_;
  boost::shared_ptr<GraspDemoObjectMap> objects_;
  boost::shared_ptr<GraspDemoPoseMap> gripper_poses_;
  boost::shared_ptr<GraspDemoPoseMap> table_poses_;
  boost::shared_ptr<GraspDemoPoseMap> viewpoint_transforms_;
  boost::shared_ptr<GraspDemoVecDoubleMap> fingerpositions_;
  boost::shared_ptr<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > analysis_msgs_;
};

} //namespace
#endif /* GRASP_DEMO_LIBRARY_H_ */
