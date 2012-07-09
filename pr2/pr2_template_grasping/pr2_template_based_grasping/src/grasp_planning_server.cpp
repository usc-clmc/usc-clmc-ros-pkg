/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_planning_server.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <vector>
#include <string>
#include <boost/bind.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <grasp_template_planning/object_detection_listener.h>
#include <pr2_template_based_grasping/grasp_planning_server.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;
using namespace Eigen;
using namespace grasp_template_planning;
using namespace grasp_template;
using namespace geometry_msgs;

namespace pr2_template_based_grasping
{

GraspPlanningServer::GraspPlanningServer(ros::NodeHandle& nh, const string& demo_path,
    const string& lib_path, const string& failure_path, const string& success_path) :
  nh_(nh), planning_pipe_(demo_path, lib_path, failure_path, success_path), visualizer_(false),
      attempt_pub_(nh.advertise<PoseStamped> ("grasp_attempt_viz", 5))
{
  object_detection_.connectToObjectDetector(nh_);
  visualizer_.initialize(nh_);
  visualizer_.startPublishing();

  planning_service_ = nh_.advertiseService("pr2_template_grasp_planner", &GraspPlanningServer::plan, this);
  ROS_INFO("Template grasp planner is up");
  vis_service_
      = nh_.advertiseService("pr2_template_grasp_planner_visualization", &GraspPlanningServer::visualize, this);
  ROS_INFO("Template grasp planner visualization service is up.");
  planning_feedback_service_ = nh_.advertiseService("pr2_template_grasp_planner_feedback",
                                                    &GraspPlanningServer::giveFeedback, this);
  ROS_INFO("Template grasp planner feedback service is up.");
  planning_summary_service_ = nh_.advertiseService("pr2_template_grasp_planner_logging", &GraspPlanningServer::getLog,
                                                   this);
  ROS_INFO("Template grasp planner logging service is up.");
}

bool GraspPlanningServer::plan(object_manipulation_msgs::GraspPlanning::Request &req,
    object_manipulation_msgs::GraspPlanning::Response &res)
{
  ros::Time t_start = ros::Time::now();
  boost::mutex::scoped_lock lock(mutex_);

  ObjectDetectionListener object_detection;
  object_detection.connectToObjectDetector(nh_);
  if (!object_detection.fetchClusterFromObjectDetector())
  {
    ROS_ERROR("Grasp planner could not obtain table pose.");
  }
  table_frame_ = object_detection.getTableFrame().pose;

  if (!object_detection_.fetchClusterFromObjectDetector())
  {
    sensor_msgs::convertPointCloudToPointCloud2(req.target.cluster, target_cloud_);
  }
  else
  {
    target_cloud_ = object_detection_.getClusterPC2();
  }
  Pose table = table_frame_;
  planning_pipe_.initialize(target_cloud_, table);

  ros::Time t_init = ros::Time::now();
  ros::Duration init_duration = t_init - t_start;

  planning_pipe_.planGrasps(grasp_pool_);

  ros::Time t_extract = ros::Time::now();
  ros::Duration extract_duration = t_extract - t_init;

  ROS_DEBUG_STREAM("pr2_template_based_grasping::GraspPlanningServer: Number of generated Grasps: "
      << grasp_pool_->size() << endl);

  ros::Time t_shift = ros::Time::now();
  ros::Duration shift_duration = t_shift - t_extract;

  convertGrasps(*grasp_pool_, res.grasps);

  ros::Time t_convert = ros::Time::now();
  ros::Duration convert_duration = t_convert - t_shift;

  ros::Duration call_duration = ros::Time::now() - t_start;

  ROS_DEBUG_STREAM("pr2_template_based_grasping::GraspPlanningServer: Initializing took: " << init_duration);
  ROS_DEBUG_STREAM("Converting took: " << convert_duration);
  ROS_DEBUG_STREAM("Overall planning took: " << call_duration);

  return true;
}

void GraspPlanningServer::convertGrasps(const TemplateMatching& pool,
    vector<object_manipulation_msgs::Grasp>& grasps)
{
  tf::StampedTransform led_to_wrist;
  getTransform(led_to_wrist, frameGripper(), "r_wrist_roll_link");

  grasp_keys_.clear();
  const unsigned int num_grasps = min(PC_NUM_GRASP_OUTPUT, static_cast<unsigned int> (pool.size()));
  for (unsigned int i = 0; i < num_grasps; i++)
  {

    Pose led_pose = pool.getGrasp(i).gripper_pose.pose;
    Pose wrist_pose;

    tf::Transform led_to_base;
    tf::poseMsgToTF(led_pose, led_to_base);
    tf::poseTFToMsg(led_to_base * led_to_wrist, wrist_pose);

    grasps.push_back(object_manipulation_msgs::Grasp());
    grasps.back().grasp_pose = wrist_pose;

    sensor_msgs::JointState pre_g_posture;
    pre_g_posture.header.stamp = ros::Time::now();
    pre_g_posture.header.frame_id = "/base_link";
    pre_g_posture.name.push_back("r_gripper_l_finger_joint");
    pre_g_posture.name.push_back("r_gripper_r_finger_joint");
    pre_g_posture.name.push_back("r_gripper_r_finger_tip_joint");
    pre_g_posture.name.push_back("r_gripper_l_finger_tip_joint");
    pre_g_posture.position.push_back(10);
    pre_g_posture.position.push_back(10);
    pre_g_posture.position.push_back(10);
    pre_g_posture.position.push_back(10);
    pre_g_posture.effort.push_back(100);
    pre_g_posture.effort.push_back(100);
    pre_g_posture.effort.push_back(100);
    pre_g_posture.effort.push_back(100);
    grasps.back().pre_grasp_posture = pre_g_posture;

    sensor_msgs::JointState g_posture = pre_g_posture;
    g_posture.header.stamp = ros::Time::now();
    g_posture.position[0] = 0;
    g_posture.position[1] = 0;
    g_posture.position[2] = 0;
    g_posture.position[3] = 0;
    grasps.back().grasp_posture = g_posture;

    grasps.back().success_probability = 1 - static_cast<double> (i) / num_grasps;

    grasps.back().desired_approach_distance = 0.1;
    grasps.back().min_approach_distance = 0.05;

    string mp_key;
    {
      stringstream ss;
      string tmp;
      ss << grasps.back().pre_grasp_posture.header.stamp;
      ss >> mp_key;
      ss.clear();
      ss << grasps.back().grasp_posture.header.stamp;
      ss >> tmp;
      mp_key.append(tmp);
    }
    grasp_keys_.insert(make_pair<string, unsigned int> (mp_key, i));
  }
}

bool GraspPlanningServer::updateGraspLibrary()
{
  const object_manipulation_msgs::Grasp& attempt = grasp_feedback_.feedback.grasp;
  string mp_key;
  {
    stringstream ss;
    string tmp;
    ss << attempt.pre_grasp_posture.header.stamp;
    ss >> mp_key;
    ss.clear();
    ss << attempt.grasp_posture.header.stamp;
    ss >> tmp;
    mp_key.append(tmp);
  }
  unsigned int pool_key = grasp_keys_.find(mp_key)->second;
  if (abs(grasp_feedback_.success) < 0.00001)
  {
    //fail
    return planning_pipe_.addFailure(grasp_pool_->getLib(pool_key), grasp_pool_->getGrasp(pool_key));
  }
  else
  {
    //success
//    tf::StampedTransform wrist_to_led;
//    getTransform(wrist_to_led, "r_wrist_roll_link", frameGripper());
//
//    Pose led_pose;
//    Pose wrist_pose = grasp_feedback_.feedback.grasp.grasp_pose;
//
//    tf::Transform wrist_to_base;
//    tf::poseMsgToTF(wrist_pose, wrist_to_base);
//    tf::poseTFToMsg(wrist_to_base * wrist_to_led, led_pose);
//
//    return planning_pipe_.addSuccess(led_pose, grasp_pool_->getLib(pool_key), *grasp_pool_);
	  return planning_pipe_.addSuccess(grasp_pool_->getLib(pool_key), grasp_pool_->getGrasp(pool_key));
  }
}

bool GraspPlanningServer::giveFeedback(PlanningFeedback::Request& req, PlanningFeedback::Response& res)
{
  boost::mutex::scoped_lock lock(mutex_);

  bool upgrade_lib_result = false;
  switch (req.action)
  {
    case PlanningFeedback::Request::DONT_UPGRADE_LIB:
      grasp_feedback_ = req;
      upgrade_lib_result = true;
      break;
    case PlanningFeedback::Request::UPDATE_LIB:
      grasp_feedback_ = req;
      upgrade_lib_result = updateGraspLibrary();
      break;
    case PlanningFeedback::Request::CHANGE_SUCCESS_AND_DO_UPGRADE:
      grasp_feedback_.success = req.success;
      upgrade_lib_result = updateGraspLibrary();
      break;
    default:
      upgrade_lib_result = false;
      break;
  }

  unsigned int vis_id = 0;
  while(vis_id < grasp_feedback_.feedback.attempted_grasp_results.size()
      && grasp_feedback_.feedback.attempted_grasp_results[vis_id].result_code
      != object_manipulation_msgs::ManipulationResult::SUCCESS)
  {
    vis_id++;
  }

  if(vis_id < grasp_feedback_.feedback.attempted_grasp_results.size())
  {
    PlanningVisualization::Request vis_req;
    vis_req.index = vis_id;
    PlanningVisualization::Response dummy_response;
    lock.unlock(); // might be not such a good idea!
    visualize(vis_req, dummy_response);
  }
  else
  {
    ROS_INFO("In the evaluation of grasps none was successful.");
  }

  return upgrade_lib_result;
}

bool GraspPlanningServer::visualize(PlanningVisualization::Request& req,
    PlanningVisualization::Response& res)
{
  boost::mutex::scoped_lock lock(mutex_);
  const unsigned int r = req.index % grasp_pool_->size();
  visualizer_.resetData(planning_pipe_, grasp_pool_->getPositiveMatch(r), grasp_pool_->getGrasp(r));

  const int result_index = getGraspResultIndex(r);
  if (result_index >= 0)
  {
    const object_manipulation_msgs::GraspResult& attempt_results =
        grasp_feedback_.feedback.attempted_grasp_results[result_index];
    const object_manipulation_msgs::Grasp& attempt = grasp_feedback_.feedback.attempted_grasps[result_index];
    {
      PoseStamped ps;
      ps.header = attempt.grasp_posture.header;
      ps.pose = attempt.grasp_pose;
      attempt_pub_.publish(ps);
    }
    ROS_INFO_STREAM("Visualizing grasp with rank " << r << " and result code " << attempt_results.result_code);
  }
  else
  {
    ROS_INFO_STREAM("Visualizing grasp with rank " << r << ". The grasp has not been evaluated.");
  }

  return true;
}

int GraspPlanningServer::getGraspResultIndex(unsigned int pool_index) const
{
  string mp_key = "-1";
  for (map<string, unsigned int>::const_iterator it = grasp_keys_.begin();
      it != grasp_keys_.end(); it++)
  {
    if (it->second == pool_index)
    {
      mp_key = it->first;
      break;
    }
  }

  if (mp_key == "-1")
  {
    return -1;
  }

  for (unsigned int i = 0; i < grasp_feedback_.feedback.attempted_grasps.size(); i++)
  {
    const object_manipulation_msgs::Grasp& attempt = grasp_feedback_.feedback.attempted_grasps[i];
    string key;
    {
      stringstream ss;
      string tmp;
      ss << attempt.pre_grasp_posture.header.stamp;
      ss >> key;
      ss.clear();
      ss << attempt.grasp_posture.header.stamp;
      ss >> tmp;
      key.append(tmp);
    }

    if (mp_key.compare(key) == 0)
    {
      return i;
    }
  }

  return -1;
}

bool GraspPlanningServer::getLog(PlanningSummary::Request& req, PlanningSummary::Response& res)
{
  res.score_labels.clear();
  res.score_labels.push_back("a = m(c, l))");
  res.score_labels.push_back("b = m(c, f))");
  res.score_labels.push_back("c = m(f, l))");
  res.score_labels.push_back(grasp_pool_->getScoreFormula());
  for (unsigned int i = 0; i < grasp_pool_->size() && i < PC_NUM_GRASP_OUTPUT; i++)
  {
    res.grasp_hypotheses.push_back(grasp_pool_->getGrasp(i));

    res.score_values.push_back(grasp_pool_->getLibScore(i));
    res.score_values.push_back(grasp_pool_->getFailScore(i));
    res.score_values.push_back(grasp_pool_->getLibQuality(i));
    res.score_values.push_back(grasp_pool_->getHypothesisScore(i));
  }

  res.table_frame = table_frame_;
  res.target_object = target_cloud_;
  res.attempted_grasp = grasp_feedback_.feedback.grasp;
  res.success = grasp_feedback_.success;

  return true;
}

}
