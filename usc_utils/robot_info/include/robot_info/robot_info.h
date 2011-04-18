/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   robot_info.h

  \author Mrinal Kalakrishnan, Peter Pastor
  \date   Jan 12, 2011

 *********************************************************************/


#ifndef ROBOT_INFO_H_
#define ROBOT_INFO_H_

#include <vector>
#include <string>
#include <tr1/unordered_map>

#include <boost/scoped_ptr.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <ros/ros.h>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <usc_utilities/assert.h>

// forward declarations
namespace KDL
{
class ChainFkSolverPos;
class ChainFkSolverVel;
class ChainIkSolverPos;
}

namespace robot_info
{

struct JointInfo
{
  int joint_id_;                /**< Joint ID */
  std::string name_;            /**< Joint name */
  double min_position_;         /**< Min joint limit */
  double max_position_;         /**< Max joint limit */

  // double preferred_velocity_;
};

class RobotInfo
{

public:

  friend class TrajectoryTimingGenerator;

  /*!
   * @return
   */
  static bool initialize();
  static int getNumJoints();

  /*!
   * @return
   */
  static const std::vector<std::string>& getRobotPartNames();

  /*!
   * @param robot_part_name
   * @param robot_part_id
   * @return
   */
  static bool getRobotPartId(const std::string& robot_part_name, int& robot_part_id);

  /*!
   */
  static double DEFAULT_SAMPLING_FREQUENCY;

  /*!
   */
  static int N_DOFS;
  static int NUM_ROBOT_PARTS;

  enum RobotPart
  {
    RIGHT_ARM,
    LEFT_ARM,
    HEAD
  };
  static bool getRobotPart(const std::string& robot_part_name, RobotPart& robot_part);

  /*!
   * @return
   */
  static const urdf::Model& getURDF();
  static const KDL::Tree& getKDLTree();

  /*!
   * @param root
   * @param tip
   * @param chain
   */
  static void getKDLChain(const std::string& root, const std::string& tip, KDL::Chain& chain);

  /*!
   * @return
   */
  static const std::vector<std::string>& getJointNames();
  /*!
   * @return
   */
  static const std::vector<std::string>& getWrenchNames();
  /*!
   * @return
   */
  static const std::vector<std::string>& getStrainGaugeNames();

  /*! Extracts all robot part names that contain joints. Thus, all non-joint parts will be removed from the vector
   * @param robot_part_names
   */
  static void extractJointParts(std::vector<std::string>& robot_part_names);

  /*! Extracts all robot part names that contain wrenches. Thus, all non-wrench parts will be removed from the vector
   * @param robot_part_names
   */
  static void extractWrenchParts(std::vector<std::string>& robot_part_names);

  /*! Extracts all robot part names that contain strain gauges. Thus, all non-strain-gauge parts will be removed from the vector
   * @param robot_part_names
   */
  static void extractStrainGaugeParts(std::vector<std::string>& robot_part_names);

  /*! Extracts all non-joint names
   * @param variable_names
   */
  static bool extractJointNames(std::vector<std::string>& variable_names);

  /*! Extracts all non-wrench names
   * @param variable_names
   */
  static bool extractWrenchNames(std::vector<std::string>& variable_names);

  /*! Extracts all non-strain-gauge names
   * @param variable_names
   */
  static bool extractStrainGaugeNames(std::vector<std::string>& variable_names);

  /*!
   * @param robot_part_names
   * @return True if robot_part_names contains joint parts, False otherwise
   */
  static bool containsJointParts(const std::vector<std::string>& robot_part_names);
  /*!
   * @param robot_part_names
   * @return True if robot_part_names contains wrench parts, False otherwise
   */
  static bool containsWrenchParts(const std::vector<std::string>& robot_part_names);
  /*!
   * @param robot_part_names
   * @return True if robot_part_names contains straing gauge parts, False otherwise
   */
  static bool containsStrainGaugeParts(const std::vector<std::string>& robot_part_names);

  /*! Removes all robot part names that contain joint parts. Thus, all strain-gauge parts will be removed from the vector
   * @param robot_part_names
   */
  static void removeJointParts(std::vector<std::string>& robot_part_names);

  /*! Removes all robot part names that contain wrench parts. Thus, all strain-gauge parts will be removed from the vector
   * @param robot_part_names
   */
  static void removeWrenchParts(std::vector<std::string>& robot_part_names);

  /*! Removes all robot part names that contain strain gauges. Thus, all strain-gauge parts will be removed from the vector
   * @param robot_part_names
   */
  static void removeStrainGaugeParts(std::vector<std::string>& robot_part_names);

  /*!
   * @param robot_part_name
   * @param joint_names
   * @return True on success, otherwise False
   */
  static bool getNames(const std::string& robot_part_name, std::vector<std::string>& names);

  /*!
   * @param robot_part_names
   * @param variable_names Vector of all variable names used by robot_part_names
   * @return True on success, otherwise False
   */
  static bool getVariableNames(const std::vector<std::string>& robot_part_names, std::vector<std::string>& variable_names);

  /*!
   * @param robot_part_name
   * @param num_variable_names
   * @return True on success, otherwise False
   */
  static bool getNumVariableNames(const std::string& robot_part_name, int& num_variable_names);

  /*!
   * @param robot_part_names
   * @param arm_joint_names
   * @return True on success, otherwise False
   */
  static bool getArmJointNames(const std::vector<std::string>& robot_part_names,
                               std::vector<std::string>& arm_joint_names);

  /*!
   * @param robot_part_names
   * @param hand_joint_names
   * @return True on success, otherwise False
   */
  static bool getHandJointNames(const std::vector<std::string>& robot_part_names,
                                std::vector<std::string>& hand_joint_names);

  /*!
   * @return
   */
  static bool getRightArmJointNames(std::vector<std::string>& right_arm_joint_names);
  /*!
   * @return
   */
  static bool getRightHandJointNames(std::vector<std::string>& right_hand_joint_names);

  /*!
   * @return
   */
  static const std::vector<std::string>& getRightEndeffectorNames();
  static const std::vector<std::string>& getLeftEndeffectorNames();

  static const std::vector<std::string>& getRightEndeffectorPositionNames();
  static const std::vector<std::string>& getLeftEndeffectorPositionNames();

  static const std::vector<std::string>& getRightEndeffectorOrientationNames();
  static const std::vector<std::string>& getLeftEndeffectorOrientationNames();

  /*!
   * @param robot_part
   * @param joint_infos
   * @return True on success, otherwise False
   */
  static bool getJointInfo(const std::string& robot_part_name, std::vector<JointInfo>& joint_infos);

  /*!
   * @param robot_part
   * @param index
   * @return True on success, otherwise False
   */
  static bool getJointInfo(const std::string& robot_part_name, const int index, JointInfo& joint_info);

  /*!
   * @return
   */
  static const std::vector<JointInfo>& getHeadJointInfo();

  /*!
   * @param robot_part_name
   * @param joint_angles
   */
  static void getRandomJointAngles(const std::string& robot_part_name, KDL::JntArray& joint_angles);

  /*!
   * Gets the joint ID of the given joint name
   * @param joint_name
   * @return joint ID, -1 if there is no such joint
   *
   */
  static int getJointId(const std::string& joint_name);

  /*!
   * Gets the joint IDs of all the given joint names
   * @param joint_names (input) Joint names
   * @param ids (output) Joint IDs
   */
  static void getJointIds(const std::vector<std::string>& joint_names, std::vector<int>& joint_ids);

  /*!
   * @param robot_part_name
   * @param joint_ids
   * @return True on success, otherwise False
   */
  static bool getJointIds(const std::string& robot_part_name, std::vector<int>& joint_ids);

private:

  RobotInfo(); // cannot construct

  static bool initialized_;
  static boost::scoped_ptr<ros::NodeHandle> node_handle_;

  static std::vector<std::string> robot_part_names_;

  static std::string robot_part_right_arm_;
  static std::string robot_part_right_hand_;
  static std::string robot_part_left_arm_;
  static std::string robot_part_left_hand_;

  static std::vector<std::string> robot_part_names_containing_joints_;
  static std::vector<std::string> robot_part_names_containing_wrenches_;
  static std::vector<std::string> robot_part_names_containing_strain_gauges_;

  static urdf::Model urdf_;
  static KDL::Tree kdl_tree_;

  static std::vector<JointInfo> joint_info_;

  static std::vector<std::string> joint_names_;
  static std::vector<std::string> wrench_names_;
  static std::vector<std::string> strain_gauge_names_;

  static std::tr1::unordered_map<std::string, int> joint_name_to_id_map_;
  static std::tr1::unordered_map<std::string, int> robot_part_name_to_id_map_;

  static std::tr1::unordered_map<std::string, std::vector<JointInfo> > robot_part_joint_info_;

  static std::tr1::unordered_map<std::string, std::vector<int> > robot_part_id_map_;
  static std::tr1::unordered_map<std::string, std::vector<std::string> > robot_part_names_map_;

  static std::vector<std::string> right_endeffector_names_;
  static std::vector<std::string> left_endeffector_names_;

  static std::vector<std::string> right_endeffector_position_names_;
  static std::vector<std::string> left_endeffector_position_names_;

  static std::vector<std::string> right_endeffector_orientation_names_;
  static std::vector<std::string> left_endeffector_orientation_names_;

  static boost::scoped_ptr<boost::variate_generator<boost::mt19937, boost::uniform_01<> > > random_generator_;

  static bool readJointInfo();
  static void checkInitialized();

  /*!
   * @param names
   * @param name
   * @return True if part_name is contained in part_names, otherwise False
   */
  static bool isContained(const std::vector<std::string>& names, const std::string& name);

  /*!
   * @param extracted_names
   * @param names
   */
  static void extract(const std::vector<std::string>& extracted_names,
                      std::vector<std::string>& names);

  /*!
   * @param names_to_be_removed
   * @param names
   */
  static void remove(const std::vector<std::string>& names_to_be_removed,
                     std::vector<std::string>& names);

};

}

#endif /* ROBOT_INFO_H_ */
