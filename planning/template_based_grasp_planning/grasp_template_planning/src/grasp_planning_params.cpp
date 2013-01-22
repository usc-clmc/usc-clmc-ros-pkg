/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_planning_params.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <sstream>
#include <time.h>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>


#include <grasp_template_planning/grasp_planning_params.h>

using namespace std;
using namespace geometry_msgs;
//using namespace visualization_msgs;
using namespace Eigen;

namespace grasp_template_planning
{

#ifdef GTP_SAFER_CREATE_ID_
boost::mutex GraspPlanningParams::mutex_;
unsigned int GraspPlanningParams::safe_id_counter_ = 0;
#endif

GraspPlanningParams::GraspPlanningParams()
{
  ros::param::get("~frame_gripper", frame_gripper_);
  ros::param::get("~template_extraction_offset_x", template_extraction_point_.x());
  ros::param::get("~template_extraction_offset_y", template_extraction_point_.y());
  ros::param::get("~template_extraction_offset_z", template_extraction_point_.z());
  ros::param::get("~learning_lib_qual_factor", learning_lib_quality_factor_);
  ros::param::get("~learning_fail_distance_factor", learning_fail_distance_factor_);
//  ros::param::get("~learning_success_add_dist", learning_success_add_dist_);
  ros::param::get("~service_name_image_poll", service_name_image_poll_);
  ros::param::get("~rostopic_colored_image", rostopic_colored_image_);
  ros::param::get("~rostopic_pickup_status", rostopic_pickup_status_);
  ros::param::get("~occlusion_punishment_fac", occlusion_punishment_fac_);
}

string GraspPlanningParams::createId()
{

#ifdef GTP_SAFER_CREATE_ID_
  boost::mutex::scoped_lock lock(mutex_);
#endif
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
//  stringstream ss;
//  ss.clear();
//  ss << time(NULL) << clock();
#ifdef GTP_SAFER_CREATE_ID_
  ss << safe_id_counter_++;
#endif
  const std::string result = boost::lexical_cast<std::string>(uuid);

  return result;
}

//void GraspPlanningParams::anounceLogBagName(std::string uuid)
//{
//	string path = getLogBagName(uuid);
//	ros::param::set("~log_bag_path", path);
//}

string GraspPlanningParams::getLogBagName(std::string uuid)
{
	string path = "planning_log_";
	path.append(uuid);
	path.append(".bag");
	return path;
}

//string GraspPlanningParams::getLogBagName()
//{
//	string name;
//	ros::param::get("~log_bag_path", name);
//
//	return name;
//}

void GraspPlanningParams::setIdAndTime(GraspAnalysis& ana)
{
	ana.uuid = createId();
	ana.stamp = ros::Time::now();
}

bool GraspPlanningParams::getTransform(tf::StampedTransform& transform,
    const string& from, const string& to) const
{
  tf::TransformListener listener;
  bool result = false;
  try
  {
    if (!listener.waitForTransform(from, to, ros::Time(0), ros::Duration(3.0)))
    {
      ROS_DEBUG_STREAM("grasp_template_planning::GraspTeachingParams: Wait for transform timed out! "
          "Tried to transform from " << from << " to " << to);
    }
    else
    {
      listener.lookupTransform(from, to, ros::Time(0), transform);
      result = true;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_DEBUG("grasp_template_planning::GraspTeachingParams: %s", ex.what());
  }

  return result;
}

string GraspPlanningParams::getRelatedFailureLib(const GraspAnalysis& grasp_lib_entry) const
{
  string out = grasp_lib_entry.demo_filename;
  /* remove ".bag" */
  {
    size_t dot_pos;
    dot_pos = out.find_last_of('.');
    out = out.substr(0, dot_pos);
  }
  out.append("_");
  out.append(grasp_lib_entry.demo_id);
  out.append("_failures.bag");

  return out;
}

string GraspPlanningParams::getRelatedSuccessLib(const GraspAnalysis& grasp_lib_entry) const
{
	  string out = grasp_lib_entry.demo_filename;
	  /* remove ".bag" */
	  {
	    size_t dot_pos;
	    dot_pos = out.find_last_of('.');
	    out = out.substr(0, dot_pos);
	  }
	  out.append("_");
	  out.append(grasp_lib_entry.demo_id);
	  out.append("_successes.bag");

	  return out;
}

//string GraspPlanningParams::createNewDemoFilename(const GraspAnalysis& grasp_lib_entry) const
//{
//  string out = grasp_lib_entry.demo_filename;
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
//  return out;
//}

} //namespace
