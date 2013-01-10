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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
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
    const string& library_path, const string& failures_path,
    const string& successes_path, const string& log_data_path)
{
  demonstrations_folder_ = demo_path;
  library_path_ = library_path;
  failures_path_ = failures_path;
  successes_path_ = successes_path;
  log_data_path_ = log_data_path;
}

PlanningPipeline::~PlanningPipeline()
{
	log_bag_.close();
}

bool PlanningPipeline::getRelatedObject(const GraspAnalysis& analysis,
    sensor_msgs::PointCloud2& container) const
{
  GraspDemoLibrary lib(demonstrations_folder_, library_path_);
  if (!lib.loadDemonstration(analysis.demo_filename))
  {
    return false;
  }

  container = lib.getObjects()->begin()->second;

  return true;
}

bool PlanningPipeline::initialize(const string& object_filename, bool log_data)
{
  offline_ = true;
  log_data_ = log_data;
  log_bag_.close();

  if(log_data_)
  {
	  string new_log_uuid = createId();
	// open a bag file
	{
		string bag_filename = log_data_path_;
		bag_filename.append(getLogBagName(new_log_uuid));

		ROS_DEBUG_STREAM("Opening new bag for logging grasp planning results: " << bag_filename);
		try {
			log_bag_.open(bag_filename, rosbag::bagmode::Write);

		} catch (rosbag::BagIOException ex) {
			ROS_DEBUG_STREAM("Problem when opening bag file " <<
					bag_filename.c_str() << " : " << ex.what());
		}
	}

	  log_ = GraspLog();
	  log_.uuid = new_log_uuid;
	  log_.stamp = ros::Time::now();
  }

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
//  geometry_msgs::Point32 min_z;
//  min_z.z = numeric_limits<float>::max();
//  for (unsigned int i = 0; i < target_object_.points.size(); i++)
//  {
//    if (target_object_.points[i].z < min_z.z)
//    {
//      min_z = target_object_.points[i];
//    }
//  }
//
//  geometry_msgs::Point table_center;
//  table_center.x = min_z.x;
//  table_center.y = min_z.y;
//  table_center.z = min_z.z;

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

  templt_generator_.reset(new HeightmapSampling(viewpoint_trans, viewpoint_rot));
  pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
  pcl::fromROSMsg(target_object_, tmp_cloud);
  templt_generator_->initialize(tmp_cloud, table_frame_);

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

bool PlanningPipeline::initialize(const sensor_msgs::PointCloud2& cluster,
    const geometry_msgs::Pose& table_pose, bool log_data)
{
  target_object_ = cluster;
  table_frame_ = table_pose;
  log_data_ = log_data;
  log_bag_.close();

  if(log_data_)
  {
	  string new_log_uuid = createId();
	// open a bag file
	{
		string bag_filename = log_data_path_;
		bag_filename.append(getLogBagName(new_log_uuid));

		ROS_DEBUG_STREAM("Opening new bag for logging grasp planning results: " << bag_filename);
		try {
			log_bag_.open(bag_filename, rosbag::bagmode::Write);

		} catch (rosbag::BagIOException ex) {
			ROS_DEBUG_STREAM("Problem when opening bag file " <<
					bag_filename.c_str() << " : " << ex.what());
		}
	}

	  log_ = GraspLog();
	  log_.uuid = new_log_uuid;
	  log_.stamp = ros::Time::now();
  }



  /* setup heightmap sampler */
  templt_generator_.reset(new HeightmapSampling());
  pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
  pcl::fromROSMsg(target_object_, tmp_cloud);
  templt_generator_->initialize(tmp_cloud, table_frame_);

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

bool PlanningPipeline::addSuccess(const GraspAnalysis& lib_grasp, const GraspAnalysis& success)
{
  string filename = successes_path_;
  filename.append(getRelatedSuccessLib(lib_grasp));

  GraspDemoLibrary succ_lib("", filename);
  bool ret_suc = succ_lib.addAnalysisToLib(success);

  return ret_suc;
}

bool PlanningPipeline::logPlannedGrasps(const TemplateMatching& pool, unsigned int max_num_grsps)
{
	if(!log_data_)
		return false;

	unsigned int num_grasps = min(pool.size(), max_num_grsps);

	log_.target_object = target_object_;
	log_.table_frame = table_frame_;
	log_.matching_scores.resize(num_grasps);
	log_.candidate_to_match.resize(num_grasps);
	log_.candidates.resize(num_grasps);

	for(unsigned int i = 0; i < num_grasps; i++)
	{
	  log_.matching_scores[i] = pool.getLibScore(i);
	  log_.candidate_to_match[i] = pool.getPositiveMatch(i).uuid;
	  log_.candidates[i] = pool.getGrasp(i);
	}

	return true;
}

bool PlanningPipeline::logGraspResult(const GraspAnalysis& res_ana,
		const TemplateMatching& pool, int rank)
{
	if(!log_data_)
		return false;

	log_.applied_grasp = res_ana;
	if(rank >= 0)
	{
		log_.matched_grasp = pool.getPositiveMatch(rank);
		log_.rank = rank;
	}

	return true;
}

bool PlanningPipeline::writeLogToBag()
{
	if(!log_data_)
		return false;

//	string bag_filename = log_data_path_;
//	bag_filename.append(getLogBagName(log_.uuid));

//	rosbag::Bag log_bag;
//	ROS_DEBUG_STREAM("Opening bag for logging grasp planning results: " << bag_filename);
//	try {
//		log_bag.open(bag_filename, rosbag::bagmode::Append);
//
//	} catch (rosbag::BagIOException ex) {
//		ROS_DEBUG_STREAM("Problem when opening bag file " <<
//				bag_filename.c_str() << " : " << ex.what());
//
//		return false;
//	}

	GraspLog log_0, log_1, log_2;
	log_0.uuid = log_.uuid;
	log_0.stamp =  ros::Time::now();
	log_0.seq = 0;

	log_1.uuid = log_.uuid;
	log_1.stamp =  ros::Time::now();
	log_1.seq = 1;

	log_2.uuid = log_.uuid;
	log_2.stamp =  ros::Time::now();
	log_2.seq = 2;

	log_0.target_object = log_.target_object;

	log_1.table_frame = log_.table_frame;
	log_1.applied_grasp = log_.applied_grasp;
	log_1.matched_grasp = log_.matched_grasp;
	log_1.rank = log_.rank;
	log_1.matching_scores = log_.matching_scores;
	log_1.candidate_to_match = log_.candidate_to_match;
	log_1.mask = log_.mask;

	log_2.candidates = log_.candidates;

	ROS_DEBUG_STREAM("Writing grasp planning log to bag: " << log_bag_.getFileName().c_str());
	try {
		log_bag_.write(topicPlanningLog(), ros::Time::now(), log_0);
	} catch (rosbag::BagIOException ex) {
		ROS_DEBUG_STREAM("Problem when writing log file " <<
				log_bag_.getFileName().c_str() << " : " << ex.what());

		return false;
	}

	ROS_DEBUG_STREAM("Writing grasp planning log to bag: " << log_bag_.getFileName().c_str());
	try {
		log_bag_.write(topicPlanningLog(), ros::Time::now(), log_1);
	} catch (rosbag::BagIOException ex) {
		ROS_DEBUG_STREAM("Problem when writing log file " <<
				log_bag_.getFileName().c_str() << " : " << ex.what());

		return false;
	}

	ROS_DEBUG_STREAM("Writing grasp planning log to bag: " << log_bag_.getFileName().c_str());
	try {
		log_bag_.write(topicPlanningLog(), ros::Time::now(), log_2);
	} catch (rosbag::BagIOException ex) {
		ROS_DEBUG_STREAM("Problem when writing log file " <<
				log_bag_.getFileName().c_str() << " : " << ex.what());

		return false;
	}

	return true;
}

//returns '0', if did not work out
//std::string PlanningPipeline::writeDemoFromAnalysis(const GraspAnalysis& ana)
//{
//	string ret = createId();
//	string demo_filename = "grasp_";
//	/* remove ".bag" */
//	{
//	  size_t dot_pos;
//	  dot_pos = out.find_last_of('.');
//	  out = out.substr(0, dot_pos);
//	}
//	demo_filename.append("_");
//	demo_filename.append(ret);
//	demo_filename.append(".bag");
//
//	string path = demonstrations_folder_;
//	path.append(demo_filename);
//
//	DemoWriter demo_writer(path);
//	if (!demo_writer.writeDemonstration(target_object_, ret, ana.gripper_pose,
//			table_pose, viewpoint, fingerpositions));
//	  return string("0");
//	demo_writer.close();
//
//	return ret;
//}

//bool PlanningPipeline::addSuccess(const geometry_msgs::Pose& grasp_pose,
//    const GraspAnalysis& succ_demo, const TemplateMatching& match_handl)
//{
//  geometry_msgs::PoseStamped table_pose;
//  table_pose.pose = table_frame_;
//  table_pose.header.frame_id = target_object_.header.frame_id;
//  table_pose.header.stamp = ros::Time::now();
//
//  geometry_msgs::PoseStamped grasp_pose_stamped;
//  grasp_pose_stamped.pose = grasp_pose;
//  grasp_pose_stamped.header = table_pose.header;
//
//  geometry_msgs::PoseStamped viewpoint;
//  const Vector3d& vp_trans = templt_generator_->viewp_trans_;
//  const Quaterniond& vp_rot = templt_generator_->viewp_rot_;
//  viewpoint.header = table_pose.header;
//  viewpoint.pose.position.x = vp_trans.x();
//  viewpoint.pose.position.y = vp_trans.y();
//  viewpoint.pose.position.z = vp_trans.z();
//  viewpoint.pose.orientation.w = vp_rot.w();
//  viewpoint.pose.orientation.x = vp_rot.x();
//  viewpoint.pose.orientation.y = vp_rot.y();
//  viewpoint.pose.orientation.z = vp_rot.z();
//
//  vector<double> fingerpositions = succ_demo.fingerpositions;
//
//  string demo_full_path = demonstrations_folder_;
//
//  string demo_filename = grasp_lib_entry.demo_filename;
//  /* remove ".bag" */
//  {
//    size_t dot_pos;
//    dot_pos = out.find_last_of('.');
//    out = out.substr(0, dot_pos);
//  }
//  out.append("_");
//  out.append(grasp_lib_entry.uuid);
//  out.append("_further_success.bag");
//
//
//  demo_full_path.append(demo_filename);
//  DemoWriter demo_writer(demo_full_path);
//  if (!demo_writer.writeDemonstrationWithExistingID(succ_demo.uuid, target_object_,
//		  grasp_pose_stamped, table_pose, viewpoint, fingerpositions));
//    return false;
//  demo_writer.close();
//
//  GraspDemoLibrary demo_lib(demonstrations_folder_, library_path_);
//  if (!demo_lib.loadDemonstration(demo_filename))
//    return false;
//  DemonstrationParser analyzer(demo_lib);
//  GraspAnalysis analysis;
//  if (!analyzer.analyzeGrasp(analysis))
//    return false;
//  if (!demo_lib.addAnalysisToLib(analysis))
//    return false;
//
//  //TODO:: add success to succ_bag that belongs to this certain demonstraition
//  return true;
//}

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
  setIdAndTime(result);
  DemonstrationParser::templtToAnalysis(templt, templt_generator_->getTemplateFrameId(), result);

  result.viewpoint_transform.header.stamp = ros::Time::now();
  result.viewpoint_transform.header.frame_id = templt_generator_->frameBase();

  result.viewpoint_transform.pose.position.x = templt_generator_->viewp_trans_.x();
  result.viewpoint_transform.pose.position.y = templt_generator_->viewp_trans_.y();
  result.viewpoint_transform.pose.position.z = templt_generator_->viewp_trans_.z();

  result.viewpoint_transform.pose.orientation.w = templt_generator_->viewp_rot_.w();
  result.viewpoint_transform.pose.orientation.x = templt_generator_->viewp_rot_.x();
  result.viewpoint_transform.pose.orientation.y = templt_generator_->viewp_rot_.y();
  result.viewpoint_transform.pose.orientation.z = templt_generator_->viewp_rot_.z();

  result.demo_filename = lib_grasp.demo_filename;
  result.demo_id = lib_grasp.demo_id;
  result.gripper_joint_state = lib_grasp.gripper_joint_state;
  result.gripper_pose = projectedGripperPose(lib_grasp, templt);
  result.fingerpositions = lib_grasp.fingerpositions;

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
  boost::shared_ptr < vector<vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > > lib_failures, lib_succs;
  lib_failures.reset(new vector<vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > ());
  lib_succs.reset(new vector<vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > ());

  ////DEBUG CODE
    int num_erased = 0;
    int num_not_erased = 0;
    //    	1342889464.300883841
  //  1343261576.160697464
  //	ros::Time threashold_time(1343261576, 160697464);
  ////DEBUG CODE

    *lib_grasps = *(library_->getAnalysisMsgs());
    for (unsigned int i = 0; i < lib_grasps->size(); i++)
    {
      lib_failures->push_back(vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > ());

    string failures_file = failures_path_;
    failures_file.append(getRelatedFailureLib((*lib_grasps)[i]));
    GraspDemoLibrary failure_lib("", failures_file);
    failure_lib.loadLibrary();

//DEBUG CODE
    if (failure_lib.getAnalysisMsgs() != NULL)
      (*lib_failures)[i] = *(failure_lib.getAnalysisMsgs());
////DEBUG CODE

////DEBUG CODE
    for(int fb_lib_id = (*lib_failures)[i].size() - 1; fb_lib_id >= 0; fb_lib_id--)
    {
    	if(
//    			(*lib_failures)[i][fb_lib_id].stamp > threashold_time ||
    			(*lib_failures)[i][fb_lib_id].grasp_template.heightmap.size()
    			!= grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X * grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X)
    	{
    		std::cout << "template width is " << (*lib_failures)[i][fb_lib_id].grasp_template.heightmap.size() <<
    				" != " << grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X *
    				grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X << std::endl;

    		std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> >::iterator lib_iter = (*lib_failures)[i].begin();
    		lib_iter += fb_lib_id;
    		(*lib_failures)[i].erase(lib_iter);

    		num_erased++;
    	}
    	else
    	{
    		num_not_erased++;
    	}
    }
////DEBUG CODE

	lib_succs->push_back(vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > ());
    string succs_file = successes_path_;
    succs_file.append(getRelatedSuccessLib((*lib_grasps)[i]));
    GraspDemoLibrary succ_lib("", succs_file);
    succ_lib.loadLibrary();

////DEBUG CODE
    if (succ_lib.getAnalysisMsgs() != NULL)
      (*lib_succs)[i] = *(succ_lib.getAnalysisMsgs());
////DEBUG CODE

////DEBUG CODE
    for(int fb_lib_id = (*lib_succs)[i].size() - 1; fb_lib_id >= 0; fb_lib_id--)
    {
    	if(
//    			(*lib_succs)[i][fb_lib_id].stamp > threashold_time ||
    			(*lib_succs)[i][fb_lib_id].grasp_template.heightmap.size()
    			!= grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X * grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X)
    	{
    		std::cout << "template width is" << (*lib_succs)[i][fb_lib_id].grasp_template.heightmap.size() <<
    				" != " << grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X *
    				grasp_template::TemplateHeightmap::TH_DEFAULT_NUM_TILES_X << std::endl;

    		std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> >::iterator lib_iter = (*lib_succs)[i].begin();
    		lib_iter += fb_lib_id;
    		(*lib_succs)[i].erase(lib_iter);

    		num_erased++;
    	}
    	else
    	{
    		num_not_erased++;
    	}
    }
////DEBUG CODE

  }

////DEBUG CODE
  ROS_INFO_STREAM("Ignored " << num_erased <<
//		  " feedback grasps due to time threashold " << threashold_time <<
		  " and kept " << num_not_erased);
////DEBUG CODE

  ros::Time t_failure_map_creation = ros::Time::now();
  ros::Duration failure_map_creation_duration = t_failure_map_creation - t_extract;

  pool.reset(new TemplateMatching(this, templts, lib_grasps, lib_failures, lib_succs));
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


