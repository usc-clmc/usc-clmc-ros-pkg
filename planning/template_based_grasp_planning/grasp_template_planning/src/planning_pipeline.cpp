/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         planning_pipeline.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <string>
#include <omp.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template_planning/demonstration_parser.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <grasp_template_planning/planning_pipeline.h>
#include <grasp_template_planning/demo_writer.h>

using namespace std;
using namespace grasp_template;
using namespace Eigen;

namespace grasp_template_planning
{

PlanningPipeline::PlanningPipeline(const string& demo_path,
    const string& library_path, const string& failures_path)
{
  demonstrations_folder_ = demo_path;
  library_path_ = library_path;
  failures_path_ = failures_path;
}

bool PlanningPipeline::getRelatedObject(const GraspAnalysis& analysis,
    sensor_msgs::PointCloud& container) const
{
  GraspDemoLibrary lib(demonstrations_folder_, library_path_);
  if (!lib.loadDemonstration(analysis.demo_filename))
  {
    return false;
  }

  container = lib.getObjects()->begin()->second;

  return true;
}

bool PlanningPipeline::initialize(const string& object_filename)
{
  offline_ = true;

  /* get object offline */
  target_folder_ = object_filename;
  size_t sep_pos = target_folder_.find_last_of('/');
  target_folder_ = target_folder_.substr(0, sep_pos + 1);

  target_file_ = object_filename;
  target_file_ = target_file_.substr(sep_pos + 1);

  GraspDemoLibrary target_object_reader(target_folder_, "");
  if (!target_object_reader.loadDemonstration(target_file_))
    return false;
  target_object_ = target_object_reader.getObjects()->begin()->second;

  /* get table frame */
  geometry_msgs::Point32 min_z;
  min_z.z = numeric_limits<float>::max();
  for (unsigned int i = 0; i < target_object_.points.size(); i++)
  {
    if (target_object_.points[i].z < min_z.z)
    {
      min_z = target_object_.points[i];
    }
  }

  geometry_msgs::Point table_center;
  table_center.x = min_z.x;
  table_center.y = min_z.y;
  table_center.z = min_z.z;

  table_frame_ = target_object_reader.getTablePoses()->begin()->second.pose;

  /* find viewpoint and setup heightmap sampler*/
  Vector3d viewpoint_trans;
  string viewpoint_frame_id;
  Quaterniond viewpoint_rot;

  {
    GraspDemoLibrary vp_reader("", library_path_);
    vp_reader.loadLibrary();
    bool vp_found = false;
    for (vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> >::const_iterator it = vp_reader.getAnalysisMsgs()->begin(); it
        != vp_reader.getAnalysisMsgs()->end(); it++)
    {
      if (it->demo_filename.compare(target_file_) == 0)
      {
        viewpoint_trans.x() = it->viewpoint_transform.pose.position.x;
        viewpoint_trans.y() = it->viewpoint_transform.pose.position.y;
        viewpoint_trans.z() = it->viewpoint_transform.pose.position.z;

        viewpoint_rot.w() = it->viewpoint_transform.pose.orientation.w;
        viewpoint_rot.x() = it->viewpoint_transform.pose.orientation.x;
        viewpoint_rot.y() = it->viewpoint_transform.pose.orientation.y;
        viewpoint_rot.z() = it->viewpoint_transform.pose.orientation.z;

        viewpoint_frame_id = it->viewpoint_transform.header.frame_id;
        vp_found = true;
      }
    }

    if (!vp_found)
      return false;
  }

  templt_generator_.reset(new HeightmapSampling(viewpoint_trans, viewpoint_rot, viewpoint_frame_id));
  templt_generator_->initialize(target_object_, table_frame_);

  /* setup library */
  library_.reset(new GraspDemoLibrary(demonstrations_folder_, library_path_));
  library_->ignore(target_file_);

  library_->loadLibrary();
  if (library_->getAnalysisMsgs()->empty())
  {
    return false;
  }

  return true;
}

bool PlanningPipeline::initialize(const sensor_msgs::PointCloud& cluster,
    const geometry_msgs::Pose& table_pose)
{
  target_object_ = cluster;
  table_frame_ = table_pose;

  /* setup heightmap sampler */
  templt_generator_.reset(new HeightmapSampling(target_object_.header.frame_id));
  templt_generator_->initialize(target_object_, table_frame_);

  /* setup library */
  library_.reset(new GraspDemoLibrary(demonstrations_folder_, library_path_));
  library_->loadLibrary();
  if (library_->getAnalysisMsgs()->empty())
    return false;

  return true;
}

bool PlanningPipeline::addFailure(const GraspAnalysis& lib_grasp, const GraspAnalysis& failure)
{
  string filename = failures_path_;
  filename.append(getRelatedFailureLib(lib_grasp));

  GraspDemoLibrary failure_lib("", filename);
  bool success = failure_lib.addAnalysisToLib(failure);

  return success;
}

bool PlanningPipeline::addSuccess(const geometry_msgs::Pose& grasp_pose,
    const GraspAnalysis& succ_demo, const TemplateMatching& match_handl)
{
  geometry_msgs::PoseStamped table_pose;
  table_pose.pose = table_frame_;
  table_pose.header.frame_id = target_object_.header.frame_id;
  table_pose.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped grasp_pose_stamped;
  grasp_pose_stamped.pose = grasp_pose;
  grasp_pose_stamped.header = table_pose.header;

  geometry_msgs::PoseStamped viewpoint;
  const Vector3d& vp_trans = templt_generator_->viewp_trans_;
  const Quaterniond& vp_rot = templt_generator_->viewp_rot_;
  viewpoint.header = table_pose.header;
  viewpoint.pose.position.x = vp_trans.x();
  viewpoint.pose.position.y = vp_trans.y();
  viewpoint.pose.position.z = vp_trans.z();
  viewpoint.pose.orientation.w = vp_rot.w();
  viewpoint.pose.orientation.x = vp_rot.x();
  viewpoint.pose.orientation.y = vp_rot.y();
  viewpoint.pose.orientation.z = vp_rot.z();

  string demo_full_path = demonstrations_folder_;
  string demo_filename = createNewDemoFilename(succ_demo);
  demo_full_path.append(demo_filename);
  DemoWriter demo_writer(demo_full_path);
  if (!demo_writer.writeDemonstration(target_object_, grasp_pose_stamped, table_pose, viewpoint))
    return false;
  demo_writer.close();

  GraspDemoLibrary demo_lib(demonstrations_folder_, library_path_);
  if (!demo_lib.loadDemonstration(demo_filename))
    return false;
  DemonstrationParser analyzer(demo_lib);
  GraspAnalysis analysis;
  if (!analyzer.analyzeGrasp(analysis))
    return false;
  if (!demo_lib.addAnalysisToLib(analysis))
    return false;

  return true;
}

geometry_msgs::PoseStamped PlanningPipeline::projectedGripperPose(const GraspAnalysis& analysis,
    const GraspTemplate& templt) const
{
  /* we assume, that template and gripper-pose from library are given in same
   * frame
   */
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = analysis.gripper_pose.header.frame_id;

  /* get frames */
  Vector3d target_templt_trans = templt.object_to_template_translation_;
  Quaterniond target_templt_rot = templt.object_to_template_rotation_;

  Vector3d ref_gripper_trans;
  Quaterniond ref_gripper_rot;
  ref_gripper_trans.x() = analysis.gripper_pose.pose.position.x;
  ref_gripper_trans.y() = analysis.gripper_pose.pose.position.y;
  ref_gripper_trans.z() = analysis.gripper_pose.pose.position.z;

  ref_gripper_rot.w() = analysis.gripper_pose.pose.orientation.w;
  ref_gripper_rot.x() = analysis.gripper_pose.pose.orientation.x;
  ref_gripper_rot.y() = analysis.gripper_pose.pose.orientation.y;
  ref_gripper_rot.z() = analysis.gripper_pose.pose.orientation.z;

  GraspTemplate ref_templt(analysis.grasp_template, analysis.template_pose.pose);

  Vector3d ref_templt_trans = ref_templt.object_to_template_translation_;
  Quaterniond ref_templt_rot = ref_templt.object_to_template_rotation_;

  Transform<double, 3, Affine> trans1;
  trans1.fromPositionOrientationScale(ref_templt_trans, ref_templt_rot, Vector3d::Ones());

  Transform<double, 3, Affine> trans2;
  trans2.fromPositionOrientationScale(ref_gripper_trans, ref_gripper_rot, Vector3d::Ones());

  Transform<double, 3, Affine> trans3;
  trans3.fromPositionOrientationScale(target_templt_trans, target_templt_rot, Vector3d::Ones());

  Transform<double, 3, Affine> trans4;
  trans4 = trans3 * trans1.inverse() * trans2;

  Quaterniond target_gripper_rot = Quaterniond(trans4.rotation());
  Vector3d target_gripper_trans = trans4.translation();

  pose.pose.position.x = target_gripper_trans.x();
  pose.pose.position.y = target_gripper_trans.y();
  pose.pose.position.z = target_gripper_trans.z();

  pose.pose.orientation.w = target_gripper_rot.w();
  pose.pose.orientation.x = target_gripper_rot.x();
  pose.pose.orientation.y = target_gripper_rot.y();
  pose.pose.orientation.z = target_gripper_rot.z();

  return pose;
}

void PlanningPipeline::createGrasp(const GraspTemplate& templt,
    const GraspAnalysis& lib_grasp, GraspAnalysis& result) const
{
  DemonstrationParser::templtToAnalysis(templt, templt_generator_->getTemplateFrameId(), result);

  result.viewpoint_transform.header.stamp = ros::Time::now();
  result.viewpoint_transform.header.frame_id = templt_generator_-> getViewpointFrameId();

  result.viewpoint_transform.pose.position.x = templt_generator_->viewp_trans_.x();
  result.viewpoint_transform.pose.position.y = templt_generator_->viewp_trans_.y();
  result.viewpoint_transform.pose.position.z = templt_generator_->viewp_trans_.z();

  result.viewpoint_transform.pose.orientation.w = templt_generator_->viewp_rot_.w();
  result.viewpoint_transform.pose.orientation.x = templt_generator_->viewp_rot_.x();
  result.viewpoint_transform.pose.orientation.y = templt_generator_->viewp_rot_.y();
  result.viewpoint_transform.pose.orientation.z = templt_generator_->viewp_rot_.z();

  result.demo_filename = target_file_;
  result.demo_id = lib_grasp.demo_id;
  result.gripper_joint_state = lib_grasp.gripper_joint_state;
  result.gripper_pose = projectedGripperPose(lib_grasp, templt);

  result.grasp_success = 0.5;
}

void PlanningPipeline::planGrasps(boost::shared_ptr<TemplateMatching>& pool) const
{
  ros::Time t_start = ros::Time::now();

  boost::shared_ptr<const vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > > templts = extractTemplatesParallel();

  ros::Time t_extract = ros::Time::now();
  ros::Duration extract_duration = t_extract - t_start;

  boost::shared_ptr < vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > lib_grasps;
  lib_grasps.reset(new vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > ());
  boost::shared_ptr < vector<vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > > lib_failures;
  lib_failures.reset(new vector<vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > ());
  *lib_grasps = *(library_->getAnalysisMsgs());
  for (unsigned int i = 0; i < lib_grasps->size(); i++)
  {
    lib_failures->push_back(vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > ());
    string failures_file = failures_path_;
    failures_file.append(getRelatedFailureLib((*lib_grasps)[i]));
    GraspDemoLibrary failure_lib("", failures_file);
    failure_lib.loadLibrary();

    if (failure_lib.getAnalysisMsgs() != NULL)
      (*lib_failures)[i] = *(failure_lib.getAnalysisMsgs());
  }

  ros::Time t_failure_map_creation = ros::Time::now();
  ros::Duration failure_map_creation_duration = t_failure_map_creation - t_extract;

  pool.reset(new TemplateMatching(this, templts, lib_grasps, lib_failures));
  pool->create();

  ros::Time t_pool_creation = ros::Time::now();
  ros::Duration pool_creation_duration = t_pool_creation - t_failure_map_creation;

  ROS_DEBUG_STREAM("grasp_template_planning::PlanningPipeline: "
      "Extracting templates took: " << extract_duration);
  ROS_DEBUG_STREAM("grasp_template_planning::PlanningPipeline: "
      "Creating failure map took: " << failure_map_creation_duration);
  ROS_DEBUG_STREAM("grasp_template_planning::PlanningPipeline: "
      "Generating grasps took: " << pool_creation_duration);
}

boost::shared_ptr<const vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > > PlanningPipeline::extractTemplatesParallel() const
{
  vector < boost::shared_ptr<vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > > > ordered_results;
  ordered_results.resize(omp_get_max_threads());

  //use omp to parallelize the process of heightmap sampling
#pragma omp parallel shared(ordered_results)
  {
    unsigned int res_index = omp_get_thread_num();
    ordered_results[res_index].reset(new vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > ());
    HeightmapSampling generator = *templt_generator_;
    HsIterator it = generator.getIterator();
    unsigned int first = it.elements_ * omp_get_thread_num() / omp_get_num_threads();
    unsigned int behind_last = it.elements_ * (omp_get_thread_num() + 1) / omp_get_num_threads();
    ROS_DEBUG_STREAM("grasp_template_planning::PlanningPipeline: Thread " << omp_get_thread_num() << ": " << first << " to " << behind_last << " - 1");
    for (it.setIndex(first); it.index_ < behind_last && !it.passedLast(); it.inc())
    {
      GraspTemplate t;
      generator.generateTemplateOnHull(t, it);

      ordered_results[res_index]->push_back(t);
    }
  }

  boost::shared_ptr < vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > > container;
  container.reset(new vector<GraspTemplate, Eigen::aligned_allocator<GraspTemplate> > );

  for (unsigned int i = 0; i < ordered_results.size(); i++)
  {
    if (ordered_results[i] != NULL)
    {
      ROS_DEBUG_STREAM("grasp_template_planning::PlanningPipeline: Job " << i
          << " sampled " << ordered_results[i]->size() << " heightmaps");
      container->insert(container->end(), ordered_results[i]->begin(), ordered_results[i]->end());
    }
  }

  return container;
}

} //namespace


