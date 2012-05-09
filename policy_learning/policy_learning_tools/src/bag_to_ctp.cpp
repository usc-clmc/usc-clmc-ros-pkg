/*
 * bag_to_ctp.cpp
 *
 *  Created on: Feb 17, 2011
 *      Author: kalakris
 */

#include <policy_learning_tools/bag_to_ctp.h>
#include <ros/ros.h>
#include <dynamic_movement_primitive_utilities/trajectory_utilities.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/file_io.h>
#include <usc_utilities/bspline.h>
#include <policy_library/covariant_trajectory_policy.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace Eigen;
//using namespace Eigen;;

namespace policy_learning_tools
{

BagToCTP::BagToCTP(ros::NodeHandle node_handle):
    node_handle_(node_handle)
{
  ROS_VERIFY(readParameters());
}

BagToCTP::~BagToCTP()
{
}

bool BagToCTP::parseArguments(int argc, char** argv)
{
  if (!usc_utilities::read(node_handle_, "input", abs_bag_file_name_))
  {
    ROS_ERROR("Please specify input:=<abs_bag_file_path> as an argument to the launch file.");
    return false;
  }
  if (!usc_utilities::read(node_handle_, "output", abs_output_bag_file_name_))
  {
    ROS_ERROR("Please specify output:=<abs_bag_file_path> as an argument to the launch file.");
    return false;
  }
  return true;
}

bool BagToCTP::readParameters()
{
  ROS_VERIFY(usc_utilities::readStringArraySpaceSeparated(node_handle_, "joint_names", joint_names_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "sampling_frequency", sampling_frequency_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "start_time", start_time_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "end_time", end_time_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_joints", use_joints_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_cartesian", use_cartesian_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_forces", use_forces_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "root_frame", root_frame_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "tip_frame", tip_frame_));
  return true;
}

bool BagToCTP::run(const std::string& input_file, const std::string& output_file)
{
  abs_bag_file_name_ = input_file;
  abs_output_bag_file_name_ = output_file;
  return run();
}

int BagToCTP::run(int argc, char** argv)
{
  if (!parseArguments(argc, argv))
    return 1;
  if (!run())
    return 1;
  return 0;
}

bool BagToCTP::run()
{
  std::vector<ros::Time> time_stamps;

  if (!createJointStateTrajectory(joint_trajectory_, time_stamps, joint_names_, abs_bag_file_name_, "/joint_states"))
  {
    ROS_ERROR("Error creating joint state trajectory from bag file.");
    return false;
  }

  if (use_joints_)
  {
    combined_trajectory_ = joint_trajectory_;
  }

  if (use_cartesian_)
  {
    std::vector<std::string> cart_names;
    cart_names.push_back("CART_X");
    cart_names.push_back("CART_Y");
    cart_names.push_back("CART_Z");
    cart_names.push_back("CART_QW");
    cart_names.push_back("CART_QX");
    cart_names.push_back("CART_QY");
    cart_names.push_back("CART_QZ");
    if (!dmp_utilities::TrajectoryUtilities::createPoseTrajectory(cartesian_trajectory_,
                     joint_trajectory_, root_frame_, tip_frame_, cart_names))
    {
      ROS_ERROR("Error creating cartesian trajectory from joint trajectory.");
      return 1;
    }

    ROS_INFO("cart traj = %d dimensions", cartesian_trajectory_.getDimension());

    if (use_joints_)
      combined_trajectory_.combine(cartesian_trajectory_);
    else
      combined_trajectory_ = cartesian_trajectory_;
  }

  ROS_INFO("Trajectory of length %d loaded.", combined_trajectory_.getNumContainedSamples());

  if (sampling_frequency_ > 0.0)
  {
    if (!downSampleAndChop(combined_trajectory_, time_stamps, sampling_frequency_, start_time_, end_time_))
    {
      ROS_ERROR("Error resampling trajectory using bsplines.");
      return false;
    }
  }
  else
  {
    sampling_frequency_ = input_sampling_frequency_;
  }

  // add zero forces by default:
  if (use_forces_)
  {
    std::vector<std::string> force_names;
    force_names.push_back("FORCE_X");
    force_names.push_back("FORCE_Y");
    force_names.push_back("FORCE_Z");
    force_names.push_back("TORQUE_X");
    force_names.push_back("TORQUE_Y");
    force_names.push_back("TORQUE_Z");
    force_trajectory_.initialize(force_names, sampling_frequency_, true, combined_trajectory_.getNumContainedSamples());
    // add dummy trajectory points
    for (int i=0; i<combined_trajectory_.getNumContainedSamples(); ++i)
    {
      Eigen::VectorXd traj_point = Eigen::VectorXd::Zero(6);
      force_trajectory_.add(traj_point);
    }
    combined_trajectory_.combine(force_trajectory_);
  }

  ROS_INFO("Saving trajectory of length %d.", combined_trajectory_.getNumContainedSamples());

  combined_trajectory_.computeDerivatives();
  combined_trajectory_.writeToCLMCFile("/tmp/d00000");

  if (!writeToCTP(combined_trajectory_, abs_output_bag_file_name_))
  {
    ROS_ERROR("Error writing trajectory to CTP bag file.");
    return false;
  }

  transformCTP(abs_output_bag_file_name_);

  return true;
}


bool BagToCTP::createJointStateTrajectory(dmp_lib::Trajectory& trajectory,
                                std::vector<ros::Time>& time_stamps,
                                const std::vector<std::string>& joint_variable_names,
                                const std::string& abs_bag_file_name,
                                const std::string& topic_name)
{
  if(joint_variable_names.empty())
  {
    ROS_ERROR("No variable names provided, cannot create trajectory from bag file.");
    return false;
  }

  // read all joint state messages from bag file
  std::vector<sensor_msgs::JointState> joint_state_msgs;
  ROS_VERIFY(usc_utilities::FileIO<sensor_msgs::JointState>::readFromBagFile(joint_state_msgs, topic_name, abs_bag_file_name));
  ROS_INFO("Read >%i< messages from bag file.", (int)joint_state_msgs.size());

  if (joint_state_msgs.size() < 2)
  {
    ROS_ERROR("Too few joint_states messages found to form a trajectory.");
    return false;
  }

  const int num_joints = static_cast<int> (joint_variable_names.size());
  const int num_data_points = static_cast<int> (joint_state_msgs.size());
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(num_joints);

  // initialize trajectory
  double dummy_sampling_frequency = 300.0; // real value is set at the end of this function
  ROS_VERIFY(trajectory.initialize(joint_variable_names, dummy_sampling_frequency, true, num_data_points));
  time_stamps.clear();

  // iterate through all messages
  for (std::vector<sensor_msgs::JointState>::const_iterator ci = joint_state_msgs.begin(); ci != joint_state_msgs.end(); ++ci)
  {
    int index = 0;
    int num_joints_found = 0;
    // for each message, iterate through the list of names
    for (std::vector<std::string>::const_iterator vsi = ci->name.begin(); vsi != ci->name.end(); vsi++)
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

  input_sampling_frequency_ = (time_stamps.size()-1) / (time_stamps.back() - time_stamps.front()).toSec();
  trajectory.setSamplingFrequency(input_sampling_frequency_);

  return true;
}


bool BagToCTP::downSampleAndChop(dmp_lib::Trajectory& trajectory,
              const std::vector<ros::Time>& time_stamps,
              double& sampling_frequency,
              double start_time,
              double end_time)
{
  // error checking
  ROS_ASSERT(trajectory.isInitialized());
  ROS_ASSERT(trajectory.getNumContainedSamples() == (int)time_stamps.size());

  int num_input_samples = trajectory.getNumContainedSamples();
  int num_dimensions = trajectory.getDimension();
  double input_trajectory_duration = time_stamps[num_input_samples-1].toSec() - time_stamps[0].toSec();
  if (end_time < 0.0)
    end_time = input_trajectory_duration;
  double mean_dt = (input_trajectory_duration) / (num_input_samples - 1);

  double in_sampling_frequency = 1.0/mean_dt;
  int downsample_factor = lrint(in_sampling_frequency / sampling_frequency);
  if (downsample_factor < 1)
    downsample_factor = 1;

  double output_dt = mean_dt * downsample_factor;

  int start_index = start_time / mean_dt;
  int num_output_samples = (end_time - start_time) / output_dt + 1;

  // create new trajectory that will hold the resampled trajectory
  dmp_lib::Trajectory resampled_trajectory;
  const bool positions_only = true;
  sampling_frequency = 1.0 / output_dt;
  ROS_VERIFY(resampled_trajectory.initialize(trajectory.getVariableNames(), sampling_frequency, positions_only, num_output_samples));

  int in_index = start_index;
  VectorXd positions = VectorXd::Zero(num_dimensions);
  for (int j = 0; j < num_output_samples; ++j)
  {
    if (in_index >= num_input_samples)
      break;
    trajectory.getTrajectoryPosition(in_index, positions);
    resampled_trajectory.add(positions);
    in_index += downsample_factor;
  }

  // set trajectory
  trajectory = resampled_trajectory;

  return true;
}

bool BagToCTP::writeToCTP(dmp_lib::Trajectory& trajectory,
                const std::string& abs_output_bag_file_name)
{
  int num_samples = trajectory.getNumContainedSamples();
  int num_dimensions = trajectory.getDimension();

  policy_msgs::CovariantTrajectoryPolicy ctp_msg;
  ctp_msg.discretization_interval = 1.0/trajectory.getSamplingFrequency();
  ctp_msg.dimension_names = trajectory.getVariableNames();
  ctp_msg.points.resize(num_samples);
  for (int i=0; i<num_samples; ++i)
  {
    ctp_msg.points[i].positions.resize(num_dimensions);
    for (int d=0; d<num_dimensions; ++d)
    {
      double p = 0.0;
      trajectory.getTrajectoryPosition(i, d, p);
      ctp_msg.points[i].positions[d] = p;
    }
  }
  ctp_msg.derivative_costs.resize(3, 0.0);
  ctp_msg.cost_ridge_factor = 0.0;

  return usc_utilities::FileIO<policy_msgs::CovariantTrajectoryPolicy>::writeToBagFile(ctp_msg, policy_library::CovariantTrajectoryPolicy::TOPIC_NAME, abs_output_bag_file_name);
}

bool BagToCTP::transformCTP(const std::string& bag_file_name)
{
  // read CTP from bag file
  policy_library::CovariantTrajectoryPolicy ctp;

  ROS_VERIFY(ctp.readFromFile(bag_file_name));

  geometry_msgs::Pose start_pose, transform_pose;
  ROS_VERIFY(ctp.getStartPose(start_pose));

  tf::Transform transform;
  tf::poseMsgToTF(start_pose, transform);
  transform = transform.inverse();
  tf::poseTFToMsg(transform, transform_pose);

  ROS_VERIFY(ctp.transformCartesianPosePolicy(transform_pose));
  ROS_VERIFY(ctp.transformCartesianWrenchPolicy(transform_pose));
  ctp.setNominalStartPose(start_pose);
  ROS_VERIFY(ctp.writeToFile(bag_file_name));

  return true;
}

}
