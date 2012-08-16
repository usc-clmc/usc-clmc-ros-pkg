/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   robot_info.cpp

  \author Mrinal Kalakrishnan, Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/

#include <map>
#include <iostream>
#include <boost/xpressive/xpressive.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// local includes
#include <robot_info/robot_info.h>

using namespace boost::xpressive;

namespace robot_info
{

const std::vector<std::string>& RobotInfo::getRobotPartNames(const int endeffector_id)
{
  checkInitialized();
  if(endeffector_id < 0)
  {
    return robot_part_names_;
  }
  else if(endeffector_id == RIGHT_ENDEFFECTOR)
  {
    return right_arm_robot_part_names_;
  }
  else if(endeffector_id == LEFT_ENDEFFECTOR)
  {
    return left_arm_robot_part_names_;
  }
  ROS_ASSERT_MSG(false, "RobotInfo: Invalid endeffector id >%i< provided.", endeffector_id);
  return robot_part_names_;
}

bool RobotInfo::getRobotPart(const std::string& robot_part_name, RobotPart& robot_part)
{
  std::string name = robot_part_name;
  boost::iterator_range<std::string::iterator> found;

  found = boost::algorithm::ifind_first(name, "right");
  if (!found.empty())
  {
    robot_part = RIGHT_ARM;
    return true;
  }

  found = boost::algorithm::ifind_first(name, "left");
  if (!found.empty())
  {
    robot_part = LEFT_ARM;
    return true;
  }

  found = boost::algorithm::ifind_first(name, "head");
  if (!found.empty())
  {
    robot_part = HEAD;
    return true;
  }

  ROS_ERROR("Could not match robot part name >%s<. Cannot return robot part enum.", robot_part_name.c_str());
  return false;
}

bool RobotInfo::getRobotPartId(const std::string& robot_part_name, int& robot_part_id)
{
  checkInitialized();
  std::tr1::unordered_map<std::string, int>::iterator it = robot_part_name_to_id_map_.find(robot_part_name);
  if (it == robot_part_name_to_id_map_.end())
  {
    ROS_ERROR("Could not find robot part >%s< to obtain part id.", robot_part_name.c_str());
    return false;
  }
  robot_part_id = it->second;
  return true;
}

int RobotInfo::getJointId(const std::string& joint_name)
{
  checkInitialized();
  std::tr1::unordered_map<std::string, int>::iterator it = joint_name_to_id_map_.find(joint_name);
  if (it == joint_name_to_id_map_.end())
  {
    return -1;
  }
  return it->second;
}

void RobotInfo::getJointIds(const std::vector<std::string>& joint_names, std::vector<int>& joint_ids)
{
  checkInitialized();
  joint_ids.resize(joint_names.size());
  for (int i = 0; i < (int)joint_names.size(); ++i)
  {
    joint_ids[i] = getJointId(joint_names[i]);
  }
}

bool RobotInfo::getJointIds(const std::string& robot_part_name, std::vector<int>& joint_ids)
{
  checkInitialized();
  std::tr1::unordered_map<std::string, std::vector<int> >::iterator it = robot_part_id_map_.find(robot_part_name);
  if (it == robot_part_id_map_.end())
  {
    return false;
  }
  joint_ids = it->second;
  return true;
}

int RobotInfo::getNumJoints()
{
  checkInitialized();
  return N_DOFS;
}

const std::vector<std::string>& RobotInfo::getJointNames()
{
  checkInitialized();
  return joint_names_;
}

const std::vector<std::string>& RobotInfo::getWrenchNames()
{
  checkInitialized();
  return wrench_names_;
}

const std::vector<std::string>& RobotInfo::getStrainGaugeNames()
{
  checkInitialized();
  return strain_gauge_names_;
}

const std::vector<std::string>& RobotInfo::getAccelerationNames()
{
  checkInitialized();
  return acceleration_names_;
}

bool RobotInfo::getNumVariableNames(const std::string& robot_part_name, int& num_variable_names)
{
  checkInitialized();
  std::vector<std::string> joint_names;
  if (!getNames(robot_part_name, joint_names))
  {
    return false;
  }
  num_variable_names = static_cast<int>(joint_names.size());
  return true;
}

bool RobotInfo::getNames(const std::string& robot_part_name, std::vector<std::string>& names)
{
  checkInitialized();
  std::tr1::unordered_map<std::string, std::vector<std::string> >::iterator it = robot_part_names_map_.find(robot_part_name);
  if (it == robot_part_names_map_.end())
  {
    ROS_ERROR("Could not find robot part named >%s<.", robot_part_name.c_str());
    ROS_ERROR("Available robot parts are:");
    for (int i = 0; i < (int)robot_part_names_.size(); ++i)
    {
      ROS_ERROR(">%s<", robot_part_names_[i].c_str());
    }
    return false;
  }
  names = it->second;
  return true;
}

bool RobotInfo::getArmJointNames(const std::vector<std::string>& robot_part_names,
                                 std::vector<std::string>& arm_joint_names)
{
  checkInitialized();
  arm_joint_names.clear();

  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    if (robot_part_names[i].compare(robot_part_right_arm_) == 0)
    {
      std::vector<std::string> right_arm_joint_names;
      if (!getNames(robot_part_right_arm_, right_arm_joint_names))
      {
        return false;
      }
      arm_joint_names.insert(arm_joint_names.begin(), right_arm_joint_names.begin(), right_arm_joint_names.end());
    }
    if (robot_part_names[i].compare(robot_part_left_arm_) == 0)
    {
      std::vector<std::string> left_arm_joint_names;
      if (!getNames(robot_part_left_arm_, left_arm_joint_names))
      {
        return false;
      }
      arm_joint_names.insert(arm_joint_names.begin(), left_arm_joint_names.begin(), left_arm_joint_names.end());
    }
  }
  return true;
}

bool RobotInfo::getHandJointNames(const std::vector<std::string>& robot_part_names,
                                  std::vector<std::string>& arm_joint_names)
{
  checkInitialized();
  arm_joint_names.clear();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    if (robot_part_names[i].compare(robot_part_right_hand_) == 0)
    {
      std::vector<std::string> right_hand_joint_names;
      if (!getNames(robot_part_right_hand_, right_hand_joint_names))
      {
        return false;
      }
      arm_joint_names.insert(arm_joint_names.begin(), right_hand_joint_names.begin(), right_hand_joint_names.end());
    }
    if (robot_part_names[i].compare(robot_part_left_hand_) == 0)
    {
      std::vector<std::string> left_hand_joint_names;
      if (!getNames(robot_part_left_hand_, left_hand_joint_names))
      {
        return false;
      }
      arm_joint_names.insert(arm_joint_names.begin(), left_hand_joint_names.begin(), left_hand_joint_names.end());
    }
  }
  return true;
}

bool RobotInfo::getRightArmJointNames(std::vector<std::string>& right_arm_joint_names)
{
  checkInitialized();
  if (!has_right_arm_)
  {
    return false;
  }
  right_arm_joint_names.clear();
  if (!getNames(robot_part_right_arm_, right_arm_joint_names))
  {
    return false;
  }

  ROS_DEBUG("RightArmJointNames:");
  for (unsigned int i = 0; i < right_arm_joint_names.size(); ++i)
  {
    ROS_DEBUG(">%s<", right_arm_joint_names[i].c_str());
  }

  return true;
}

bool RobotInfo::getRightHandJointNames(std::vector<std::string>& right_hand_joint_names)
{
  checkInitialized();
  if (!has_right_arm_)
  {
    return false;
  }
  right_hand_joint_names.clear();
  if (!getNames(robot_part_right_hand_, right_hand_joint_names))
  {
    return false;
  }
  return true;
}

bool RobotInfo::getLeftArmJointNames(std::vector<std::string>& left_arm_joint_names)
{
  checkInitialized();
  if (!has_left_arm_)
  {
    return false;
  }
  left_arm_joint_names.clear();
  if (!getNames(robot_part_left_arm_, left_arm_joint_names))
  {
    return false;
  }

  ROS_DEBUG("LeftArmJointNames:");
  for (unsigned int i = 0; i < left_arm_joint_names.size(); ++i)
  {
    ROS_DEBUG(">%s<", left_arm_joint_names[i].c_str());
  }

  return true;
}

bool RobotInfo::getLeftHandJointNames(std::vector<std::string>& left_hand_joint_names)
{
  checkInitialized();
  if (!has_left_arm_)
  {
    return false;
  }
  left_hand_joint_names.clear();
  if (!getNames(robot_part_left_hand_, left_hand_joint_names))
  {
    return false;
  }
  return true;
}

const std::vector<std::string>& RobotInfo::getRightEndeffectorNames()
{
  checkInitialized();
  return right_endeffector_names_;
}

const std::vector<std::string>& RobotInfo::getLeftEndeffectorNames()
{
  checkInitialized();
  return left_endeffector_names_;
}

const std::vector<std::string>& RobotInfo::getRightEndeffectorPositionNames()
{
  checkInitialized();
  return right_endeffector_position_names_;
}

const std::vector<std::string>& RobotInfo::getLeftEndeffectorPositionNames()
{
  checkInitialized();
  return left_endeffector_position_names_;
}

const std::vector<std::string>& RobotInfo::getRightEndeffectorOrientationNames()
{
  checkInitialized();
  return right_endeffector_orientation_names_;
}

const std::vector<std::string>& RobotInfo::getLeftEndeffectorOrientationNames()
{
  checkInitialized();
  return left_endeffector_orientation_names_;
}

bool RobotInfo::getJointInfo(const std::string& robot_part_name, std::vector<JointInfo>& joint_infos)
{
  checkInitialized();
  std::tr1::unordered_map<std::string, std::vector<JointInfo> >::iterator it = robot_part_joint_info_.find(
      robot_part_name);
  if (it == robot_part_joint_info_.end())
  {
    ROS_ERROR("Could not find robot part named >%s<.", robot_part_name.c_str());
    ROS_ERROR("Available robot parts are:");
    for (int i = 0; i < (int)robot_part_names_.size(); ++i)
    {
      ROS_ERROR(">%s<", robot_part_names_[i].c_str());
    }
    return false;
  }
  joint_infos = it->second;
  return true;
}

bool RobotInfo::getJointInfo(const std::string& robot_part_name, const int index, JointInfo& joint_info)
{
  checkInitialized();
  std::vector<JointInfo> joint_infos;
  if (!getJointInfo(robot_part_name, joint_infos))
  {
    return false;
  }
  if (index >= 0 && index < (int)joint_infos.size())
  {
    ROS_ERROR("Index >%i< out of bounds [0..%i). Cannot set joint info.", index, (int)joint_infos.size());
    return false;
  }
  joint_info = joint_infos[index];
  return true;
}

bool RobotInfo::isRightArm(const std::string& robot_part_name)
{
  checkInitialized();
  if (!has_right_arm_)
  {
    return false;
  }
  return (robot_part_right_arm_.compare(robot_part_name) == 0);
}

bool RobotInfo::isLeftArm(const std::string& robot_part_name)
{
  checkInitialized();
  if (!has_left_arm_)
  {
    return false;
  }
  return (robot_part_left_arm_.compare(robot_part_name) == 0);
}

bool RobotInfo::containsRightArm(const std::vector<std::string>& variable_names)
{
  checkInitialized();
  if (!has_right_arm_)
  {
    return false;
  }
  std::vector<std::string> right_arm = getRightEndeffectorNames();
  std::vector<std::string> right_arm_joint_names;
  ROS_VERIFY(getRightArmJointNames(right_arm_joint_names));
  right_arm.insert(right_arm.end(), right_arm_joint_names.begin(), right_arm_joint_names.end());
  for (int i = 0; i < (int)right_arm.size(); ++i)
  {
    if (!isContained(variable_names, right_arm[i]))
    {
      return false;
    }
  }
  return true;
}

bool RobotInfo::containsLeftArm(const std::vector<std::string>& variable_names)
{
  checkInitialized();
  if (!has_left_arm_)
  {
    return false;
  }
  std::vector<std::string> left_arm = getLeftEndeffectorNames();
  std::vector<std::string> left_arm_joint_names;
  ROS_VERIFY(getLeftArmJointNames(left_arm_joint_names));
  left_arm.insert(left_arm.end(), left_arm_joint_names.begin(), left_arm_joint_names.end());
  for (int i = 0; i < (int)left_arm.size(); ++i)
  {
    if (!isContained(variable_names, left_arm[i]))
    {
      return false;
    }
  }
  return true;
}

bool RobotInfo::isRightArmPart(const std::string& robot_part_name)
{
  if (robot_part_name.substr(0,5).compare("RIGHT") == 0)
  {
    return true;
  }
  return false;
}
bool RobotInfo::isLeftArmPart(const std::string& robot_part_name)
{
  if (robot_part_name.substr(0,4).compare("LEFT") == 0)
  {
    return true;
  }
  return false;
}

const urdf::Model& RobotInfo::getURDF()
{
  checkInitialized();
  return urdf_;
}

const KDL::Tree& RobotInfo::getKDLTree()
{
  checkInitialized();
  return kdl_tree_;
}

//bool RobotInfo::getNumJoints(const std::string& robot_part_name,
//                             int num_joints)
//{
//  checkInitialized();
//  std::tr1::unordered_map<std::string, std::vector<int> >::iterator it = robot_part_joints_.find(robot_part_name);
//  if (it == robot_part_joints_.end())
//  {
//    ROS_ERROR("Could not find robot part named >%s<.", robot_part_name.c_str());
//    ROS_ERROR("Available robot parts are:");
//    for (int i = 0; i < (int)robot_part_names_.size(); ++i)
//    {
//      ROS_ERROR(">%s<", robot_part_names_[i].c_str());
//    }
//    return false;
//  }
//  num_joints = it->second.size();
//  return true;
//}

void RobotInfo::getRandomJointAngles(const std::string& robot_part_name, KDL::JntArray& joint_angles)
{
  checkInitialized();
  int num_joints = 0;
  ROS_VERIFY(getNumVariableNames(robot_part_name, num_joints));
  joint_angles.resize(num_joints);
  std::vector<JointInfo> joint_info;
  ROS_VERIFY(getJointInfo(robot_part_name, joint_info));
  for (int i = 0; i < num_joints; ++i)
  {
    double rand01 = (*random_generator_)();
    const double& min = joint_info[i].min_position_;
    const double& max = joint_info[i].max_position_;
    joint_angles(i) = (max - min) * rand01 + min;
  }
}

void RobotInfo::checkInitialized()
{
  if (!initialized_)
  {
    ROS_ERROR("RobotInfo not initialized. Please call robot_info::init() before creating any objects from the robot_info library");
    ROS_BREAK();
  }
}

bool RobotInfo::initialize()
{
  if (initialized_)
  {
    ROS_WARN("Initializing RobotInfo multiple times!");
    return true;
  }

  ros::NodeHandle robot_info_node_handle("/robot_info");

  std::string robot_name;
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_name", robot_name));

  ROS_DEBUG("Initializing robot >%s<.", robot_name.c_str());

  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_names", robot_part_names_));

  NUM_ROBOT_PARTS = (int)robot_part_names_.size();

  boost::iterator_range<std::string::iterator> found;
  for (int i = 0; i < (int)robot_part_names_.size(); ++i)
  {
    found = boost::algorithm::ifind_first(robot_part_names_[i], "right");
    if (!found.empty())
    {
      ROS_DEBUG("Initializing right robot part name >%s< (%i).", robot_part_names_[i].c_str(), i);
      right_arm_robot_part_names_.push_back(robot_part_names_[i]);
    }
  }
  for (int i = 0; i < (int)robot_part_names_.size(); ++i)
  {
    found = boost::algorithm::ifind_first(robot_part_names_[i], "left");
    if (!found.empty())
    {
      ROS_DEBUG("Initializing left robot part name >%s< (%i).", robot_part_names_[i].c_str(), i);
      left_arm_robot_part_names_.push_back(robot_part_names_[i]);
    }
  }

  ROS_DEBUG("Reading robot info parameters. Found >%i< left arm robot parts and >%i< right arm robot parts.",
            (int)right_arm_robot_part_names_.size(), (int)left_arm_robot_part_names_.size());

  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_names_containing_joints", robot_part_names_containing_joints_));
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_names_containing_wrenches", robot_part_names_containing_wrenches_));
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_names_containing_accelerations", robot_part_names_containing_accelerations_));
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_names_containing_strain_gauges", robot_part_names_containing_strain_gauges_));

  has_right_arm_ = false;
  if (usc_utilities::read(robot_info_node_handle, "robot_part_right_arm", robot_part_right_arm_))
  {
    has_right_arm_ = true;
    ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_right_hand", robot_part_right_hand_));
  }

  has_left_arm_ = false;
  if (usc_utilities::read(robot_info_node_handle, "robot_part_left_arm", robot_part_left_arm_, false))
  {
    has_left_arm_ = true;
    ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "robot_part_left_hand", robot_part_left_hand_));
  }

  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "default_sampling_frequency", DEFAULT_SAMPLING_FREQUENCY));

  XmlRpc::XmlRpcValue robot_part_group;
  if (!robot_info_node_handle.getParam("groups", robot_part_group))
  {
    ROS_ERROR("Could not read parameter >groups< in namespace >%s<.", robot_info_node_handle.getNamespace().c_str());
    return false;
  }

  if (robot_part_group.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Robot parts must be arranged in groups.");
    return false;
  }

  for (int i = 0; i < robot_part_group.size(); ++i)
  {
    std::string robot_part_name;
    ROS_VERIFY(usc_utilities::getParam(robot_part_group[i], "name", robot_part_name));
    ROS_DEBUG("Robot part groups: %s", robot_part_name.c_str());
  }
  ROS_ASSERT_MSG(robot_part_group.size() >= NUM_ROBOT_PARTS, "Robot part group has size >%i<, but need to be greater than >%i<.",
                 (int)robot_part_group.size(), (int)NUM_ROBOT_PARTS);

  joint_names_.clear();
  std::map<std::string, int> joint_map;
  int joint_count = 0;
  for (int i = 0; i < robot_part_group.size(); ++i)
  {
    if (!robot_part_group[i].hasMember("name") || !robot_part_group[i].hasMember("joints"))
    {
      ROS_ERROR("Could not parse >%s<. Either >name< tag and/or >joints< tag is/are missing.", robot_info_node_handle.getNamespace().c_str());
      return false;
    }

    for (int j = 0; j < (int)robot_part_names_.size(); ++j)
    {
      if (robot_part_group[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        ROS_ERROR("Robot part >name< must be of type string.");
        return false;
      }
      std::string robot_part_name;
      if (!usc_utilities::getParam(robot_part_group[i], "name", robot_part_name))
      {
        ROS_ERROR("Could not obtain >name< tag.");
        return false;
      }

      if (robot_part_name.compare(robot_part_names_[j]) == 0)
      {
        XmlRpc::XmlRpcValue robot_part;
        if (robot_part_group[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR("Joints of robot part >%s< must be a string.", robot_part_names_[j].c_str());
          return false;
        }

        std::vector<std::string> joint_names;
        std::vector<int> joint_ids;
        std::string joints_string = robot_part_group[i]["joints"];
        usc_utilities::tokenizeString(joints_string, joint_names);
        ROS_ASSERT(!joint_names.empty());
        for (int k = 0; k < (int)joint_names.size(); ++k)
        {
          ROS_DEBUG("Robot joints >%s<.", joint_names[k].c_str());
          joint_map.insert(std::pair<std::string, int>(joint_names[k], 0));
          joint_ids.push_back(joint_count);
          joint_count++;
        }

        if (RobotInfo::isContained(robot_part_names_containing_joints_, robot_part_names_[j]))
        {
          joint_names_.push_back(robot_part_names_[j]);
        }
        else if (RobotInfo::isContained(robot_part_names_containing_wrenches_, robot_part_names_[j]))
        {
          wrench_names_.push_back(robot_part_names_[j]);
        }
        else if (RobotInfo::isContained(robot_part_names_containing_strain_gauges_, robot_part_names_[j]))
        {
          strain_gauge_names_.push_back(robot_part_names_[j]);
        }
        else if (RobotInfo::isContained(robot_part_names_containing_accelerations_, robot_part_names_[j]))
        {
          acceleration_names_.push_back(robot_part_names_[j]);
        }
        else
        {
          ROS_ERROR("Invalid robot part name >%s< read from yaml file. Could not initialize robot info.", robot_part_names_[j].c_str());
          return false;
        }
        ROS_DEBUG("Adding: ");
        for (int k = 0; k < (int)joint_names.size(); ++k)
        {
          ROS_DEBUG(">%s<", joint_names[k].c_str());
        }
        ROS_DEBUG("..to >%s<.", robot_part_names_[j].c_str());
        robot_part_names_map_.insert(std::tr1::unordered_map<std::string, std::vector<std::string> >::value_type(robot_part_names_[j], joint_names));
        robot_part_id_map_.insert(std::tr1::unordered_map<std::string, std::vector<int> >::value_type(robot_part_names_[j], joint_ids));
      }
    }
  }
  ROS_ASSERT((int)joint_map.size() == joint_count);

  N_DOFS = joint_count;
  ROS_INFO("Found >%i< joints of robot >%s<.", N_DOFS, robot_name.c_str());

  ROS_DEBUG("Setting up robot info parameters.");

  node_handle_.reset(new ros::NodeHandle());
  joint_info_.resize(N_DOFS);

  // load urdf:
  ROS_VERIFY(urdf_.initParam("/robot_description"));

  // create kdl tree:
  ROS_VERIFY(kdl_parser::treeFromUrdfModel(urdf_, kdl_tree_));

  ROS_VERIFY(readJointInfo());
  ROS_DEBUG("Joint information:");
  for (int i = 0; i < N_DOFS; ++i)
  {
    ROS_DEBUG("%s: %f to %f", joint_info_[i].name_.c_str(), joint_info_[i].min_position_, joint_info_[i].max_position_);
  }

//  for (int rp = 0; rp < NUM_ROBOT_PARTS; ++rp)
//  {
//    for (int i = 0; i < (int)robot_part_joints_[rp].size(); ++i)
//    {
//      int joint_id;
//      ROS_VERIFY(getJointInfo());
//      robot_part_joint_info_.insert(std::tr1::unordered_map<std::string, std::vector<JointInfo> >::value_type(robot_part_names_[j], joint_info_
//              int joint = robot_part_joints_[rp][i];
//              robot_part_joint_names_[rp].push_back(joint_info_[joint].name_);
//              robot_part_joint_info_[rp].push_back(joint_info_[joint]);
//            }
//          }

// initialize random number generators:
  boost::mt19937 mt19937;
  boost::uniform_01<> uniform_01;
  random_generator_.reset(new boost::variate_generator<boost::mt19937, boost::uniform_01<> >(mt19937, uniform_01));

  right_endeffector_position_names_.clear();
  left_endeffector_position_names_.clear();
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "right_endeffector_position_variable_names", right_endeffector_position_names_));
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "left_endeffector_position_variable_names", left_endeffector_position_names_));

  right_endeffector_orientation_names_.clear();
  left_endeffector_orientation_names_.clear();
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "right_endeffector_orientation_variable_names", right_endeffector_orientation_names_));
  ROS_VERIFY(usc_utilities::read(robot_info_node_handle, "left_endeffector_orientation_variable_names", left_endeffector_orientation_names_));

  right_endeffector_names_.clear();
  right_endeffector_names_.insert(right_endeffector_names_.end(), right_endeffector_position_names_.begin(),
                                  right_endeffector_position_names_.end());
  right_endeffector_names_.insert(right_endeffector_names_.end(), right_endeffector_orientation_names_.begin(),
                                  right_endeffector_orientation_names_.end());

  left_endeffector_names_.clear();
  left_endeffector_names_.insert(left_endeffector_names_.end(), left_endeffector_position_names_.begin(),
                                 left_endeffector_position_names_.end());
  left_endeffector_names_.insert(left_endeffector_names_.end(), left_endeffector_orientation_names_.begin(),
                                 left_endeffector_orientation_names_.end());

  return (initialized_ = true);
}

bool RobotInfo::readJointInfo()
{
//  XmlRpc::XmlRpcValue joint_info_xml;
//  if (!node_handle_->getParam("/robot_model/joint_info", joint_info_xml))
//  {
//    ROS_ERROR("Couldn't read parameter /robot_model/joint_info");
//    return false;
//  }
//
//  ROS_VERIFY(joint_info_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
//  ROS_ASSERT(joint_info_xml.size() == N_DOFS);
//  for (int i = 0; i < N_DOFS; ++i)
//  {
//    XmlRpc::XmlRpcValue joint_xml = joint_info_xml[i];
//    if (!usc_utilities::getParam(joint_xml, "name", joint_info_[i].name_))
//    {
//      ROS_ERROR("Could not read joint name of joint >%i<", i);
//      return false;
//    }
//    joint_names_[i] = joint_info_[i].name_;
//    joint_info_[i].joint_id_ = i;
//
//    boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_.getJoint(joint_info_[i].name_);
//    ROS_ASSERT(urdf_joint);
//    joint_info_[i].min_position_ = urdf_joint->limits->lower;
//    joint_info_[i].max_position_ = urdf_joint->limits->upper;
//
//    joint_name_to_id_map_.insert(std::tr1::unordered_map<std::string, int>::value_type(joint_info_[i].name_, i));
//  }
  return true;
}

void RobotInfo::getKDLChain(const std::string& root, const std::string& tip, KDL::Chain& chain)
{
  checkInitialized();
  if (!kdl_tree_.getChain(root, tip, chain))
  {
    ROS_ERROR("Failed to get kdl chain from %s to %s.", root.c_str(), tip.c_str());
  }
}

bool RobotInfo::isContained(const std::vector<std::string>& part_names, const std::string& part_name)
{
  for (std::vector<std::string>::const_iterator ci = part_names.begin(); ci != part_names.end(); ++ci)
  {
    if (ci->compare(part_name) == 0)
    {
      return true;
    }
  }
  return false;
}

bool RobotInfo::containsJointParts(const std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    if (RobotInfo::isContained(robot_part_names_containing_joints_, robot_part_names[i]))
    {
      return true;
    }
  }
  return false;
}

bool RobotInfo::containsWrenchParts(const std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    if (RobotInfo::isContained(robot_part_names_containing_wrenches_, robot_part_names[i]))
    {
      return true;
    }
  }
  return false;
}

bool RobotInfo::containsAccelerationParts(const std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    if (RobotInfo::isContained(robot_part_names_containing_accelerations_, robot_part_names[i]))
    {
      return true;
    }
  }
  return false;
}

bool RobotInfo::containsStrainGaugeParts(const std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    ROS_DEBUG("Checking whether >%s< has strain gauge parts.", robot_part_names[i].c_str());
    if (RobotInfo::isContained(robot_part_names_containing_strain_gauges_, robot_part_names[i]))
    {
      ROS_DEBUG("Yes, >%s< does.", robot_part_names[i].c_str());
      return true;
    }
  }
  return false;
}

void RobotInfo::extract(const std::vector<std::string>& names_to_be_extracted,
                        std::vector<std::string>& names)
{
  std::vector<int> remove_indices;
  for (int i = 0; i < (int)names.size(); ++i)
  {
    if (!RobotInfo::isContained(names_to_be_extracted, names[i]))
    {
      remove_indices.push_back(i);
    }
  }
  for (int i = (int)remove_indices.size() - 1; i >= 0; --i)
  {
    names.erase(names.begin() + remove_indices[i]);
  }
}

void RobotInfo::remove(const std::vector<std::string>& names_to_be_removed,
                       std::vector<std::string>& names)
{
  std::vector<int> remove_indices;
  for (int i = 0; i < (int)names.size(); ++i)
  {
    if (RobotInfo::isContained(names_to_be_removed, names[i]))
    {
      remove_indices.push_back(i);
    }
  }
  for (int i = (int)remove_indices.size() - 1; i >= 0; --i)
  {
    names.erase(names.begin() + remove_indices[i]);
  }
}

void RobotInfo::extractJointParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  extract(robot_part_names_containing_joints_, robot_part_names);
}

void RobotInfo::extractWrenchParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  extract(robot_part_names_containing_wrenches_, robot_part_names);
}

void RobotInfo::extractAccelerationParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  extract(robot_part_names_containing_accelerations_, robot_part_names);
}

void RobotInfo::extractStrainGaugeParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  extract(robot_part_names_containing_strain_gauges_, robot_part_names);
}

void RobotInfo::removeJointParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  remove(robot_part_names_containing_joints_, robot_part_names);
}

void RobotInfo::removeWrenchParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  remove(robot_part_names_containing_wrenches_, robot_part_names);
}

void RobotInfo::removeAccelerationParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  remove(robot_part_names_containing_accelerations_, robot_part_names);
}

void RobotInfo::removeStrainGaugeParts(std::vector<std::string>& robot_part_names)
{
  checkInitialized();
  remove(robot_part_names_containing_strain_gauges_, robot_part_names);
}

bool RobotInfo::getVariableNames(const std::vector<std::string>& robot_part_names,
                                 std::vector<std::string>& variable_names)
{
  checkInitialized();
  variable_names.clear();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    std::vector<std::string> names;
    if (!getNames(robot_part_names[i], names))
    {
      ROS_ERROR("Could not get variable names for robot part >%s<.", robot_part_names[i].c_str());
      return false;
    }
    ROS_DEBUG("RobotPart (%i) >%s<:", i, robot_part_names[i].c_str());
    for (int j = 0; j < (int)names.size(); ++j)
    {
      ROS_DEBUG(" - %s", names[j].c_str());
    }
    variable_names.insert(variable_names.end(), names.begin(), names.end());
  }
  return true;
}

bool RobotInfo::extractJointNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> joint_names;
  if (!getVariableNames(robot_part_names_containing_joints_, joint_names))
  {
    return false;
  }
  extract(joint_names, variable_names);
  return true;
}

bool RobotInfo::extractWrenchNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> wrench_names;
  if (!getVariableNames(robot_part_names_containing_wrenches_, wrench_names))
  {
    return false;
  }
  extract(wrench_names, variable_names);
  return true;
}

bool RobotInfo::extractAccelerationNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> acceleration_names;
  if (!getVariableNames(robot_part_names_containing_accelerations_, acceleration_names))
  {
    return false;
  }
  extract(acceleration_names, variable_names);
  return true;
}

bool RobotInfo::extractStrainGaugeNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> strain_gauge_names;
  if (!getVariableNames(robot_part_names_containing_strain_gauges_, strain_gauge_names))
  {
    return false;
  }
  extract(strain_gauge_names, variable_names);
  return true;
}

bool RobotInfo::removeJointNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> joint_names;
  if (!getVariableNames(robot_part_names_containing_joints_, joint_names))
  {
    return false;
  }
  remove(joint_names, variable_names);
  return true;
}

bool RobotInfo::removeWrenchNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> wrench_names;
  if (!getVariableNames(robot_part_names_containing_wrenches_, wrench_names))
  {
    return false;
  }
  remove(wrench_names, variable_names);
  return true;
}

bool RobotInfo::removeAccelerationNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> acceleration_names;
  if (!getVariableNames(robot_part_names_containing_accelerations_, acceleration_names))
  {
    return false;
  }
  remove(acceleration_names, variable_names);
  return true;
}

bool RobotInfo::removeStrainGaugeNames(std::vector<std::string>& variable_names)
{
  checkInitialized();
  std::vector<std::string> strain_gauge_names;
  if (!getVariableNames(robot_part_names_containing_strain_gauges_, strain_gauge_names))
  {
    return false;
  }
  remove(strain_gauge_names, variable_names);
  return true;
}

int RobotInfo::getHandEndeffectorId(const std::string& name)
{
  checkInitialized();
  if (name.compare(right_hand_name_) == 0)
  {
    return RobotInfo::getRightHandEndeffectorId();
  }
  else if (name.compare(left_hand_name_) == 0)
  {
    return RobotInfo::getLeftHandEndeffectorId();
  }
  ROS_ASSERT_MSG(false, "RobotInfo: Invalid endeffector name >%s<. Cannot return endeffector id.", name.c_str());
  return -1;
}

int RobotInfo::getRightHandEndeffectorId()
{
  checkInitialized();
  return RIGHT_ENDEFFECTOR;
}
int RobotInfo::getLeftHandEndeffectorId()
{
  checkInitialized();
  return LEFT_ENDEFFECTOR;
}

bool RobotInfo::isRightHand(const int endeffector_id)
{
  checkInitialized();
  return (endeffector_id == RIGHT_ENDEFFECTOR);
}
bool RobotInfo::isLeftHand(const int endeffector_id)
{
  checkInitialized();
  return (endeffector_id == LEFT_ENDEFFECTOR);
}

std::string RobotInfo::getEndeffectorName(const int endeffector_id)
{
  checkInitialized();
  std::string endeffector_name = "INVALID_ENDEFFECTOR_ID";
  if (RobotInfo::isRightHand(endeffector_id))
  {
    endeffector_name = RobotInfo::right_hand_name_;
  }
  else if (RobotInfo::isLeftHand(endeffector_id))
  {
    endeffector_name = RobotInfo::left_hand_name_;
  }
  else
  {
    ROS_ERROR("RobotInfo: Invalid endeffector_id provided >%i<. Cannot return endeffector name.", endeffector_id);
  }
  return endeffector_name;
}

std::string RobotInfo::getEndeffectorNameLower(const int endeffector_id)
{
  checkInitialized();
  std::string endeffector_name = RobotInfo::getEndeffectorName(endeffector_id);
  return boost::to_lower_copy(endeffector_name);
}

std::string RobotInfo::getWhichArmLowerLetterFromRobotPart(const std::string& robot_part_name)
{
  std::string which_arm = "";
  if (robot_info::RobotInfo::isRightArmPart(robot_part_name))
  {
    which_arm.assign("r");
  }
  else if (robot_info::RobotInfo::isLeftArmPart(robot_part_name))
  {
    which_arm.assign("l");
  }
  else
  {
    ROS_ASSERT_MSG(false, "Invalid robot part name >%s<. Cannot return lower letter from robot part name.", robot_part_name.c_str());
  }
  return which_arm;
}

std::string RobotInfo::getWhichArm(const int endeffector_id)
{
  checkInitialized();
  std::string endeffector_name = "INVALID_ENDEFFECTOR_ID";
  if (RobotInfo::isRightHand(endeffector_id))
  {
    endeffector_name = "Right";
  }
  else if (RobotInfo::isLeftHand(endeffector_id))
  {
    endeffector_name = "Left";
  }
  else
  {
    ROS_ERROR("RobotInfo: Invalid endeffector_id provided >%i<. Cannot determine which arm.", endeffector_id);
  }
  return endeffector_name;
}

std::string RobotInfo::getWhichArmLower(const int endeffector_id)
{
  checkInitialized();
  std::string endeffector_name = RobotInfo::getWhichArm(endeffector_id);
  return boost::to_lower_copy(endeffector_name);
}

bool RobotInfo::initialized_ = false;

double RobotInfo::DEFAULT_SAMPLING_FREQUENCY = 0.0;

int RobotInfo::N_DOFS = 0;
int RobotInfo::NUM_ROBOT_PARTS = 0;

std::vector<std::string> RobotInfo::robot_part_names_;
std::vector<std::string> RobotInfo::right_arm_robot_part_names_;
std::vector<std::string> RobotInfo::left_arm_robot_part_names_;

bool RobotInfo::has_right_arm_;
std::string RobotInfo::robot_part_right_arm_;
std::string RobotInfo::robot_part_right_hand_;

bool RobotInfo::has_left_arm_;
std::string RobotInfo::robot_part_left_arm_;
std::string RobotInfo::robot_part_left_hand_;

std::vector<std::string> RobotInfo::robot_part_names_containing_joints_;
std::vector<std::string> RobotInfo::robot_part_names_containing_wrenches_;
std::vector<std::string> RobotInfo::robot_part_names_containing_accelerations_;
std::vector<std::string> RobotInfo::robot_part_names_containing_strain_gauges_;

std::string RobotInfo::right_hand_name_;
std::string RobotInfo::left_hand_name_;

boost::scoped_ptr<ros::NodeHandle> RobotInfo::node_handle_;

urdf::Model RobotInfo::urdf_;
KDL::Tree RobotInfo::kdl_tree_;

std::vector<JointInfo> RobotInfo::joint_info_;

std::vector<std::string> RobotInfo::joint_names_;
std::vector<std::string> RobotInfo::wrench_names_;
std::vector<std::string> RobotInfo::strain_gauge_names_;
std::vector<std::string> RobotInfo::acceleration_names_;

std::tr1::unordered_map<std::string, int> RobotInfo::joint_name_to_id_map_;
std::tr1::unordered_map<std::string, int> RobotInfo::robot_part_name_to_id_map_;

std::tr1::unordered_map<std::string, std::vector<JointInfo> > RobotInfo::robot_part_joint_info_;
std::tr1::unordered_map<std::string, std::vector<int> > RobotInfo::robot_part_id_map_;
std::tr1::unordered_map<std::string, std::vector<std::string> > RobotInfo::robot_part_names_map_;

std::vector<std::string> RobotInfo::right_endeffector_names_;
std::vector<std::string> RobotInfo::left_endeffector_names_;

std::vector<std::string> RobotInfo::right_endeffector_position_names_;
std::vector<std::string> RobotInfo::left_endeffector_position_names_;

std::vector<std::string> RobotInfo::right_endeffector_orientation_names_;
std::vector<std::string> RobotInfo::left_endeffector_orientation_names_;

boost::scoped_ptr<boost::variate_generator<boost::mt19937, boost::uniform_01<> > > RobotInfo::random_generator_;

}
