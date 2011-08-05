/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		skill_library.cpp

  \author	Peter Pastor
  \date		Jan 22, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

// local includes
#include <skill_library/skill_library.h>

using namespace Eigen;
using namespace std;
using namespace dmp;
using namespace dmp_utilities;
using namespace inverse_kinematics;

namespace skill_library
{

bool SkillLibrary::initialize()
{
  std::string data_directory_name;
  ROS_ASSERT(usc_utilities::read(node_handle_, "data_directory_name", data_directory_name));
  usc_utilities::appendTrailingSlash(data_directory_name);

  std::string package_name;
  ROS_ASSERT(usc_utilities::read(node_handle_, "package_name", package_name));
  std::string absolute_path = ros::package::getPath(package_name);
  usc_utilities::appendTrailingSlash(absolute_path);

  std::string library_root_directory = absolute_path + data_directory_name;
  ROS_INFO("Initializing DMP library client with base directory >%s<.", library_root_directory.c_str());
  ROS_VERIFY(dmp_library_client_.initialize(library_root_directory));

  add_affordance_service_server_ = node_handle_.advertiseService("/SkillLibrary/addAffordance", &SkillLibrary::addAffordance, this);
  get_affordance_service_server_ = node_handle_.advertiseService("/SkillLibrary/getAffordance", &SkillLibrary::getAffordance, this);

  return (initialized_ = true);
}

bool SkillLibrary::getAffordance(getAffordance::Request& request, getAffordance::Response& response)
{

  if(!dmp_library_client_.getDMP(request.affordance.object.name, response.affordance.dmp.icra2009_dmp))
  {
    response.result =  getAffordance::Response::FAILED;
    return true;
  }
  response.result = getAffordance::Response::SUCCEEDED;
  return true;
}

bool SkillLibrary::addAffordance(addAffordance::Request& request, addAffordance::Response& response)
{

  if(request.affordance.object.name.empty())
  {
    ROS_ERROR("Cannot add affordance without name.");
    response.result = addAffordance::Response::FAILED;
    return true;
  }

  dmp_lib::DMPPtr dmp;
  ROS_VERIFY(DynamicMovementPrimitiveUtilities::getDMP(request.affordance.dmp, dmp));

  // ROS_VERIFY(dmp->setup());
  // vector<geometry_msgs::PoseStamped> poses;
  // vector<VectorXd> rest_postures;
  // ROS_VERIFY(TrajectoryUtilities::createTrajectory(dmp, poses, rest_postures));
  // vector<VectorXd> joint_angles;
  // ROS_VERIFY(inverse_kinematics_.ik(poses, rest_postures, rest_postures[0], joint_angles));

  dmp_library_client_.addDMP(dmp, request.affordance.object.name);
  return true;
}

}
