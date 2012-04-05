/*
 * bag_to_ctp.h
 *
 *  Created on: Mar 4, 2011
 *      Author: kalakris
 */

#ifndef BAG_TO_CTP_H_
#define BAG_TO_CTP_H_

/*
 * bag_to_ctp.cpp
 *
 *  Created on: Feb 17, 2011
 *      Author: kalakris
 */

#include <ros/ros.h>
#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>

namespace policy_learning_tools
{

class BagToCTP
{
public:
  BagToCTP(ros::NodeHandle node_handle);
  ~BagToCTP();

  int run(int argc, char** argv);
  bool run(const std::string& input_file, const std::string& output_file);

  bool run();

private:
  std::string abs_bag_file_name_;
  std::string abs_output_bag_file_name_;
  std::vector<std::string> joint_names_;
  double sampling_frequency_;
  double input_sampling_frequency_;
  double start_time_;
  double end_time_;
  ros::NodeHandle node_handle_;

  bool use_joints_;
  bool use_cartesian_;
  bool use_forces_;

  std::string root_frame_;
  std::string tip_frame_;

  dmp_lib::Trajectory joint_trajectory_;
  dmp_lib::Trajectory cartesian_trajectory_;
  dmp_lib::Trajectory combined_trajectory_;
  dmp_lib::Trajectory force_trajectory_;

  bool createJointStateTrajectory(dmp_lib::Trajectory& trajectory,
                                  std::vector<ros::Time>& time_stamps,
                                  const std::vector<std::string>& joint_variable_names,
                                  const std::string& abs_bag_file_name,
                                  const std::string& topic_name);

  bool downSampleAndChop(dmp_lib::Trajectory& trajectory,
                const std::vector<ros::Time>& time_stamps,
                double& sampling_frequency,
                double start_time,
                double end_time);

  bool writeToCTP(dmp_lib::Trajectory& trajectory,
                  const std::string& abs_output_bag_file_name);

  bool transformCTP(const std::string& bag_file_name);

  bool parseArguments(int argc, char** argv);

  bool readParameters();
};

}
#endif /* BAG_TO_CTP_H_ */
