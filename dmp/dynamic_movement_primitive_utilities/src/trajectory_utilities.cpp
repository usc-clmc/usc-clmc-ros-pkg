/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		trajectory_utilities.cpp

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Jan 5, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <math.h>

#include <usc_utilities/file_io.h>
#include <usc_utilities/constants.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/bspline.h>
#include <usc_utilities/kdl_chain_wrapper.h>

#include <robot_info/robot_info.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <arm_controller_msgs/Acceleration.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/TypeMsg.h>

// local includes
#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>

using namespace Eigen;
using namespace std;

namespace dmp_utilities
{

/*! Abbreviation vor convinience
 */
typedef sensor_msgs::JointState JointStateMsg;
typedef geometry_msgs::WrenchStamped WrenchStampedMsg;
typedef geometry_msgs::Point PointMsg;
typedef geometry_msgs::Quaternion QuaternionMsg;

bool TrajectoryUtilities::createTrajectory(const dmp_lib::DMPPtr dmp,
                                           const std::string& base_frame_id,
                                           std::vector<PoseStampedMsg>& poses,
                                           std::vector<VectorXd>& rest_postures,
                                           const double movement_duration)
{
  ROS_ASSERT(dmp->isInitialized());
  if (!dmp->hasType(dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE))
  {
    ROS_ERROR("Cannot create trajectory. DMP is of type >%i<, but should be of type >%i<", dmp->getType(),
        dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE);
    return false;
  }

  double duration = movement_duration;
  if (duration < 0)
  {
    ROS_VERIFY(dmp->getInitialDuration(duration));
  }

  dmp_lib::Trajectory trajectory;
  ROS_VERIFY(dmp->propagateFull(trajectory, duration));

  // trajectory needs to contain more dimensions than just end effector poses.
  ROS_ASSERT(trajectory.getDimension() > usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT);

  poses.clear();
  rest_postures.clear();

  double sampling_frequency = trajectory.getSamplingFrequency();
  double delta_t = (double)1.0 / sampling_frequency;
  ros::Time time(0);

  // TODO: think about whether trajectory should first be "prepared" using endeffector variable names read from robot_info::RobotInfo

  VectorXd current_trajectory_positions = VectorXd::Zero(dmp->getNumDimensions());
  for (int i = 0; i < trajectory.getNumContainedSamples(); ++i)
  {
    ROS_VERIFY(trajectory.getTrajectoryPosition(i, current_trajectory_positions));

    PoseStampedMsg pose_stamped;
    pose_stamped.header.frame_id = base_frame_id;
    time += ros::Duration(delta_t);
    pose_stamped.header.stamp = time;
    pose_stamped.pose.position.x = current_trajectory_positions(usc_utilities::Constants::X);
    pose_stamped.pose.position.y = current_trajectory_positions(usc_utilities::Constants::Y);
    pose_stamped.pose.position.z = current_trajectory_positions(usc_utilities::Constants::Z);
    pose_stamped.pose.orientation.w = current_trajectory_positions(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW);
    pose_stamped.pose.orientation.x = current_trajectory_positions(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX);
    pose_stamped.pose.orientation.y = current_trajectory_positions(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY);
    pose_stamped.pose.orientation.z = current_trajectory_positions(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ);
    poses.push_back(pose_stamped);

    int rest_posture_dimension = trajectory.getDimension() - usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT;
    VectorXd rest_posture = VectorXd::Zero(rest_posture_dimension);
    for (int j = 0; j < rest_posture_dimension; ++j)
    {
      rest_posture(j) = current_trajectory_positions(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT + j);
    }
    rest_postures.push_back(rest_posture);
  }

  return true;
}

bool TrajectoryUtilities::createWrenchTrajectory(dmp_lib::Trajectory& trajectory,
                                                 const vector<string>& wrench_variable_names,
                                                 const string& abs_bag_file_name,
                                                 const double sampling_frequency,
                                                 const string& topic_name,
                                                 const bool compute_derivatives)
{
  if(wrench_variable_names.size() != 6)
  {
    ROS_ERROR("Number of wrench variable names >%i< is invalid, cannot create wrench trajectory from bag file >%s<.", (int)wrench_variable_names.size(), abs_bag_file_name.c_str());
    ROS_ERROR_COND(wrench_variable_names.empty(), "Provided wrench variable names are empy !");
    ROS_ERROR_COND(!wrench_variable_names.empty(), "Provided wrench variable names are:");
    for (int i = 0; i < (int)wrench_variable_names.size(); ++i)
    {
      ROS_ERROR(">%s<", wrench_variable_names[i].c_str());
    }
    return false;
  }

  // read all wrench state messages from bag file
  vector<WrenchStampedMsg> wrench_state_msgs;
  ROS_VERIFY(usc_utilities::FileIO<WrenchStampedMsg>::readFromBagFile(wrench_state_msgs, topic_name, abs_bag_file_name, false));
  ROS_DEBUG("Read >%i< wrench messages from bag file >%s<.", (int)wrench_state_msgs.size(), abs_bag_file_name.c_str());

  const int NUM_FORCES = static_cast<int> (wrench_variable_names.size());
  const int NUM_DATA_POINTS = static_cast<int> (wrench_state_msgs.size());
  VectorXd wrench_positions = VectorXd::Zero(NUM_FORCES);

  // initialize trajectory
  // TODO: using sampling_frequency, which actually is not required.
  ROS_VERIFY(trajectory.initialize(wrench_variable_names, sampling_frequency, true, NUM_DATA_POINTS));
  vector<ros::Time> time_stamps;

  // check whether the provided wrench variable names are complete and in the correct order.
  vector<string> right_arm_force_variable_names;
  ROS_VERIFY(robot_info::RobotInfo::getNames("RIGHT_ARM_FORCE", right_arm_force_variable_names));
  ROS_VERIFY((int)right_arm_force_variable_names.size() == NUM_FORCES);
  bool right_arm_force_variables_match = true;
  vector<string> left_arm_force_variable_names;
  ROS_VERIFY(robot_info::RobotInfo::getNames("LEFT_ARM_FORCE", left_arm_force_variable_names));
  ROS_VERIFY((int)left_arm_force_variable_names.size() == NUM_FORCES);
  bool left_arm_force_variables_match = true;
  for (int i = 0; i < NUM_FORCES; ++i)
  {
    if(right_arm_force_variable_names[i].compare(wrench_variable_names[i]) != 0)
    {
      right_arm_force_variables_match = false;
    }
    if(left_arm_force_variable_names[i].compare(wrench_variable_names[i]) != 0)
    {
      left_arm_force_variables_match = false;
    }
  }

  if(!right_arm_force_variables_match && !left_arm_force_variables_match)
  {
    for (int i = 0; i < NUM_FORCES; ++i)
    {
      ROS_ERROR("Wrench variable name >%s< or >%s< number >%i< does not match expected variable name >%s<. Cannot create wrench trajectory.",
                right_arm_force_variable_names[i].c_str(), left_arm_force_variable_names[i].c_str(), i, wrench_variable_names[i].c_str());
    }
    return false;
  }

  // iterate through all messages
  for (vector<WrenchStampedMsg>::const_iterator ci = wrench_state_msgs.begin(); ci != wrench_state_msgs.end(); ++ci)
  {
    wrench_positions(0) = ci->wrench.force.x;
    wrench_positions(1) = ci->wrench.force.y;
    wrench_positions(2) = ci->wrench.force.z;
    wrench_positions(3) = ci->wrench.torque.x;
    wrench_positions(4) = ci->wrench.torque.y;
    wrench_positions(5) = ci->wrench.torque.z;
    // add data
    ROS_VERIFY(trajectory.add(wrench_positions));
    time_stamps.push_back(ci->header.stamp);
  }

  ROS_VERIFY(TrajectoryUtilities::filter(trajectory, "WrenchLowPass"));
  ROS_VERIFY(TrajectoryUtilities::resample(trajectory, time_stamps, sampling_frequency, compute_derivatives));
  return true;
}

bool TrajectoryUtilities::createAccelerationTrajectory(dmp_lib::Trajectory& trajectory,
                                                       const vector<string>& acceleration_variable_names,
                                                       const string& abs_bag_file_name,
                                                       const double sampling_frequency,
                                                       const string& topic_name,
                                                       const bool compute_derivatives)
{
  const unsigned int NUM_ACC = 3;
  if(acceleration_variable_names.size() != NUM_ACC)
  {
    ROS_ERROR("Number of acceleration variable names >%i< is invalid, cannot create acceleration trajectory from bag file >%s<.", (int)acceleration_variable_names.size(), abs_bag_file_name.c_str());
    return false;
  }

  // read all wrench state messages from bag file
  vector<arm_controller_msgs::Acceleration> acceleration_msgs;
  ROS_VERIFY(usc_utilities::FileIO<arm_controller_msgs::Acceleration>::readFromBagFile(acceleration_msgs, topic_name, abs_bag_file_name, false));
  ROS_INFO("Read >%i< wrench messages from bag file >%s<.", (int)acceleration_msgs.size(), abs_bag_file_name.c_str());

  const int NUM_DATA_POINTS = static_cast<int> (acceleration_msgs.size());
  VectorXd accelerations = VectorXd::Zero(NUM_ACC);

  // initialize trajectory
  // TODO: using sampling_frequency, which actually is not required.
  ROS_VERIFY(trajectory.initialize(acceleration_variable_names, sampling_frequency, true, NUM_DATA_POINTS));
  vector<ros::Time> time_stamps;

  // check whether the provided wrench variable names are complete and in the correct order.
  vector<string> right_hand_acceleration_variable_names;
  ROS_VERIFY(robot_info::RobotInfo::getNames("RIGHT_HAND_ACC", right_hand_acceleration_variable_names));
  ROS_VERIFY(right_hand_acceleration_variable_names.size() == NUM_ACC);
  bool right_hand_acceleration_variables_match = true;
  vector<string> left_hand_acceleration_variable_names;
  ROS_VERIFY(robot_info::RobotInfo::getNames("LEFT_HAND_ACC", left_hand_acceleration_variable_names));
  ROS_VERIFY(left_hand_acceleration_variable_names.size() == NUM_ACC);
  bool left_hand_acceleration_variables_match = true;
  for (unsigned int i = 0; i < NUM_ACC; ++i)
  {
    if(right_hand_acceleration_variable_names[i].compare(acceleration_variable_names[i]) != 0)
    {
      right_hand_acceleration_variables_match = false;
    }
    if(left_hand_acceleration_variable_names[i].compare(acceleration_variable_names[i]) != 0)
    {
      left_hand_acceleration_variables_match = false;
    }
  }

  if(!right_hand_acceleration_variables_match && !left_hand_acceleration_variables_match)
  {
    for (unsigned int i = 0; i < NUM_ACC; ++i)
    {
      ROS_ERROR("Acceleration variable name >%s< or >%s< number >%i< does not match expected variable name >%s<. Cannot create wrench trajectory.",
                right_hand_acceleration_variable_names[i].c_str(), left_hand_acceleration_variable_names[i].c_str(), i, acceleration_variable_names[i].c_str());
    }
    return false;
  }

  // iterate through all messages
  for (vector<arm_controller_msgs::Acceleration>::const_iterator ci = acceleration_msgs.begin(); ci != acceleration_msgs.end(); ++ci)
  {
    for (unsigned int i = 0; i < NUM_ACC; ++i)
    {
      accelerations(i) = ci->acceleration[i];
    }
    // add data
    ROS_VERIFY(trajectory.add(accelerations));
    time_stamps.push_back(ci->header.stamp);
  }

  ROS_VERIFY(TrajectoryUtilities::filter(trajectory, "AccelerationLowPass"));
  ROS_VERIFY(TrajectoryUtilities::resample(trajectory, time_stamps, sampling_frequency, compute_derivatives));
  return true;
}

bool TrajectoryUtilities::createJointStateTrajectory(dmp_lib::Trajectory& trajectory,
                                                     const vector<string>& joint_variable_names,
                                                     const string& abs_bag_file_name,
                                                     const double sampling_frequency,
                                                     const string& topic_name,
                                                     const bool compute_derivatives)
{
  if(joint_variable_names.empty())
  {
    ROS_ERROR("No joint variable names provided, cannot create joint trajectory from bag file >%s<.", abs_bag_file_name.c_str());
    return false;
  }

  // read all joint state messages from bag file
  vector<JointStateMsg> joint_state_msgs;
  ROS_VERIFY(usc_utilities::FileIO<JointStateMsg>::readFromBagFile(joint_state_msgs, topic_name, abs_bag_file_name, false));
  ROS_INFO("Read >%i< joint messages from bag file >%s<.", (int)joint_state_msgs.size(), abs_bag_file_name.c_str());

  const int num_joints = static_cast<int> (joint_variable_names.size());
  const int num_data_points = static_cast<int> (joint_state_msgs.size());
  VectorXd joint_positions = VectorXd::Zero(num_joints);

  // initialize trajectory
  // TODO: using sampling_frequency, which actually is not required.
  ROS_VERIFY(trajectory.initialize(joint_variable_names, sampling_frequency, true, num_data_points));
  vector<ros::Time> time_stamps;

  // iterate through all messages
  for (vector<JointStateMsg>::const_iterator ci = joint_state_msgs.begin(); ci != joint_state_msgs.end(); ++ci)
  {
    int index = 0;
    int num_joints_found = 0;
    // for each message, iterate through the list of names
    for (vector<string>::const_iterator vsi = ci->name.begin(); vsi != ci->name.end(); vsi++)
    {
      // iterate through all variable names
      for (int i = 0; i < num_joints; ++i)
      {
        // find match
        // ROS_DEBUG("Comparing: >%s< and >%s<", vsi->c_str(), joint_variable_names[i].c_str());
        if(vsi->compare(joint_variable_names[i]) == 0)
        {
          // ROS_DEBUG("MATCH !");
          joint_positions(i) = ci->position[index];
          num_joints_found++;
        }
      }
      index++;
    }

    // check whether all variable names were found
    if (num_joints_found != num_joints)
    {
      ROS_ERROR("Number of joints is >%i<, but there have been only >%i< matches.", num_joints, num_joints_found);
      return false;
    }
    // add data
    ROS_VERIFY(trajectory.add(joint_positions));
    time_stamps.push_back(ci->header.stamp);
  }

  ROS_VERIFY(TrajectoryUtilities::resample(trajectory, time_stamps, sampling_frequency, compute_derivatives));
  return true;
}

bool TrajectoryUtilities::createPoseTrajectory(dmp_lib::Trajectory& pose_trajectory,
                                               const dmp_lib::Trajectory& joint_trajectory,
                                               const std::string& start_link_name,
                                               const std::string& end_link_name,
                                               const vector<string>& variable_names)
{

  if(variable_names.empty())
  {
    ROS_ERROR("There are no variable names provided. Cannot create pose trajectory.");
    return false;
  }

  dmp_lib::Trajectory arm_joint_trajectory = joint_trajectory;
  // arm_joint_trajectory.onlyKeep(variable_names);

  const int num_trajectory_points = arm_joint_trajectory.getNumContainedSamples();
  const int trajectory_dimension = arm_joint_trajectory.getDimension();

  usc_utilities::KDLChainWrapper arm_chain;
  arm_chain.initialize(start_link_name, end_link_name);

  // Debugging...
  ROS_DEBUG("Creating pose trajectory from link >%s< to link >%s< with >%d< joints.", start_link_name.c_str(), end_link_name.c_str(), arm_chain.getNumJoints());
  std::vector<std::string> chain_joint_names;
  arm_chain.getJointNames(chain_joint_names);
  for(int i=0; i<(int)chain_joint_names.size(); ++i)
  {
    ROS_DEBUG("Joint >%i< is >%s<.", i, chain_joint_names[i].c_str());
  }

  // error checking
  // ROS_ASSERT(static_cast<int>(variable_names.size()) == arm_chain.getNumJoints());
  ROS_ASSERT(arm_joint_trajectory.isInitialized());
  ROS_ASSERT(num_trajectory_points > 0);
  ROS_ASSERT(trajectory_dimension > 0);

  ROS_VERIFY(pose_trajectory.initialize(variable_names, arm_joint_trajectory.getSamplingFrequency(), true, num_trajectory_points));

  KDL::Frame endeffector_frame;
  KDL::JntArray arm_kdl_joint_array(arm_chain.getNumJoints());
  Eigen::VectorXd endeffector_pose = VectorXd::Zero(usc_utilities::Constants::N_CART + usc_utilities::Constants::N_QUAT);

  if(trajectory_dimension != arm_chain.getNumJoints())
  {
    ROS_ERROR("Number of joints in the chain >%i< does not correspond to number of joints >%i< stored in the trajectory.", arm_chain.getNumJoints(), trajectory_dimension);
    return false;
  }

  for (int i = 0; i < num_trajectory_points; ++i)
  {
    ROS_VERIFY(arm_joint_trajectory.getTrajectoryPosition(i, arm_kdl_joint_array.data));
    arm_chain.forwardKinematics(arm_kdl_joint_array, endeffector_frame);
    endeffector_pose(usc_utilities::Constants::X) = endeffector_frame.p.x();
    endeffector_pose(usc_utilities::Constants::Y) = endeffector_frame.p.y();
    endeffector_pose(usc_utilities::Constants::Z) = endeffector_frame.p.z();
    double qx, qy, qz, qw;
    endeffector_frame.M.GetQuaternion(qx, qy, qz, qw);
    endeffector_pose(usc_utilities::Constants::N_CART + usc_utilities::Constants::QW) = qw;
    endeffector_pose(usc_utilities::Constants::N_CART + usc_utilities::Constants::QX) = qx;
    endeffector_pose(usc_utilities::Constants::N_CART + usc_utilities::Constants::QY) = qy;
    endeffector_pose(usc_utilities::Constants::N_CART + usc_utilities::Constants::QZ) = qz;
    ROS_VERIFY(pose_trajectory.add(endeffector_pose));
  }

  return true;
}

bool TrajectoryUtilities::resample(dmp_lib::Trajectory& trajectory,
                                   const vector<ros::Time>& time_stamps,
                                   const double sampling_frequency,
                                   const bool compute_derivatives)
{
  // error checking
  ROS_ASSERT(trajectory.isInitialized());
  ROS_ASSERT(trajectory.getNumContainedSamples() == (int)time_stamps.size());
  ROS_ASSERT(sampling_frequency > 0.0 && sampling_frequency < 2000);
  ROS_ASSERT(time_stamps.size() > 0);

  // compute mean dt of the provided time stamps
  const int trajectory_length = trajectory.getNumContainedSamples();
  const int trajectory_dimension = trajectory.getDimension();

  double dts[trajectory_length - 1];
  double mean_dt = 0.0;

  vector<double> input_vector(trajectory_length);
  input_vector[0] = time_stamps[0].toSec();
  for (int i = 0; i < trajectory_length - 1; ++i)
  {
      dts[i] = time_stamps[i + 1].toSec() - time_stamps[i].toSec();
      mean_dt += dts[i];
      input_vector[i + 1] = input_vector[i] + dts[i];
  }
  for (int i = 0; i < trajectory_length; ++i)
  {
    input_vector[i] -= time_stamps[0].toSec();
    // ROS_INFO("input x: %f", input_vector[i]);
  }
  mean_dt /= static_cast<double> (trajectory_length - 1);

  ros::Time start_time = time_stamps[0];
  ros::Time end_time = time_stamps.back();

  double trajectory_duration = end_time.toSec() - start_time.toSec();
  ROS_WARN_COND(trajectory_duration < 0.001, "Trajectory is very short (>%f< seconds). Its start time is >%f< and its end time is >%f<.", trajectory_duration, start_time.toSec(), end_time.toSec());
  const int new_trajectory_length = ceil( trajectory_duration * sampling_frequency );

  ros::Duration interval = static_cast<ros::Duration> (static_cast<double>(1.0) / sampling_frequency);
  // TODO: figure out why setting factor to 2.0 does not work to create bspline
  double cutoff_wave_length = 300.0 * interval.toSec();

  ROS_INFO("Resampling trajectory of duration >%.1f< seconds from >%i< samples to >%i< samples with sampling frequency >%.1f< to intervals of >%.4f< seconds.",
      trajectory_duration, trajectory_length, new_trajectory_length, sampling_frequency, interval.toSec());

  vector<double> input_querry(new_trajectory_length);
  for (int i = 0; i < new_trajectory_length; i++)
  {
    input_querry[i] = i * interval.toSec();
    // ROS_INFO("input q: %f", input_querry[i]);
  }

  vector<vector<double> > positions_resampled;
  positions_resampled.resize(trajectory_dimension);
  vector<double> position_target_vector;
  position_target_vector.resize(trajectory_length);
  for (int i = 0; i < trajectory_dimension; ++i)
  {
    for (int j = 0; j < trajectory_length; ++j)
    {
      double position = 0;
      ROS_VERIFY(trajectory.getTrajectoryPosition(j, i, position));
      position_target_vector[j] = position;
      // ROS_INFO("input y: %f", position_target_vector[j]);
    }
    if(!usc_utilities::resample(input_vector, position_target_vector, cutoff_wave_length, input_querry, positions_resampled[i], false))
    {
      ROS_ERROR("Could not rescale position trajectory, splining failed.");
      return false;
    }
    ROS_ASSERT(new_trajectory_length == (int)positions_resampled[i].size());
  }

  // create new trajectory that will hold the resampled trajectory
  dmp_lib::Trajectory resampled_trajectory;
  const bool positions_only = true;
  ROS_VERIFY(resampled_trajectory.initialize(trajectory.getVariableNames(), sampling_frequency, positions_only, new_trajectory_length));

  for (int j = 0; j < new_trajectory_length; ++j)
  {
    VectorXd positions = VectorXd::Zero(trajectory_dimension);
    for (int i = 0; i < trajectory_dimension; ++i)
    {
      positions(i) = positions_resampled[i][j];
    }
    ROS_VERIFY(resampled_trajectory.add(positions));
  }

  // HACK: set the first trajectory point to second... TODO: fix this !!!
  // VectorXd trajectory_point = VectorXd::Zero(resampled_trajectory.getDimension());
  // ROS_VERIFY(resampled_trajectory.getTrajectoryPoint(2, trajectory_point));
  // ROS_VERIFY(resampled_trajectory.setTrajectoryPoint(1, trajectory_point));
  // ROS_VERIFY(resampled_trajectory.setTrajectoryPoint(0, trajectory_point));

  // ROS_INFO_STREAM(resampled_trajectory.getInfo());

  if(compute_derivatives)
  {
    ROS_VERIFY(resampled_trajectory.computeDerivatives());
  }

  // set trajectory
  trajectory = resampled_trajectory;
  return true;
}

bool TrajectoryUtilities::filter(dmp_lib::Trajectory& trajectory, const std::string& filter_name)
{
  const int num_traces = trajectory.getDimension();
  const int num_samples = trajectory.getNumContainedSamples();
  VectorXd trajectory_positions = VectorXd::Zero(num_traces);

  ros::NodeHandle node_handle("~");
  std::vector<double> filtered_data(num_traces, 0.0);
  std::vector<double> unfiltered_data(num_traces, 0.0);
  filters::MultiChannelTransferFunctionFilter<double> filter;

  ROS_VERIFY(((filters::MultiChannelFilterBase<double>&)filter).configure(num_traces, node_handle.getNamespace() + std::string("/") + filter_name, node_handle));

  // initialize filter
  ROS_VERIFY(trajectory.getTrajectoryPosition(0, trajectory_positions));
  for (int j = 0; j < num_traces; ++j)
  {
    unfiltered_data[j] = trajectory_positions(j);
  }
  for (int i = 0; i < 100; ++i)
  {
    ROS_VERIFY(filter.update(unfiltered_data, filtered_data));
  }

  for (int i = 0; i < num_samples; ++i)
  {
    ROS_VERIFY(trajectory.getTrajectoryPosition(i, trajectory_positions));
    for (int j = 0; j < num_traces; ++j)
    {
      unfiltered_data[j] = trajectory_positions(j);
    }
    ROS_VERIFY(filter.update(unfiltered_data, filtered_data));
    for (int j = 0; j < num_traces; ++j)
    {
      trajectory_positions(j) = filtered_data[j];
    }
    ROS_VERIFY(trajectory.setTrajectoryPoint(i, trajectory_positions));
  }
  return true;
}

bool TrajectoryUtilities::getDurations(const double initial_duration,
                                       const std::vector<double>& duration_fractions,
                                       std::vector<double>& durations)
{
  // error checking
  double beginning = 0.0;
  for (int i = 0; i < (int)duration_fractions.size(); ++i)
  {
    if (beginning > duration_fractions[i])
    {
      ROS_ERROR("Duration fractions must monotonically increase.");
      return false;
    }
    beginning = duration_fractions[i];
  }
  if (!(beginning < 1.0))
  {
    ROS_ERROR("Duration fractions must be strictly less then 1.0.");
    return false;
  }

  durations.clear();
  if (duration_fractions.empty())
  {
    durations.push_back(initial_duration);
    return true;
  }
  else
  {
    durations.push_back(duration_fractions[0] * initial_duration);
    for (int i = 1; i < (int)duration_fractions.size(); ++i)
    {
      durations.push_back((duration_fractions[i] - duration_fractions[i - 1]) * initial_duration);
    }
    durations.push_back((1.0 - duration_fractions[duration_fractions.size() - 1]) * initial_duration);
  }

  double total_duration = 0.0;
  for (int i = 0; i < (int)durations.size(); ++i)
  {
    total_duration += durations[i];
  }

  return (fabs(initial_duration - total_duration) < 10e-6);
}

}
