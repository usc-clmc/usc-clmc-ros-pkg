/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan, E. Gil Jones */

#include <stomp_motion_planner/stomp_planner_node.h>
#include <stomp_motion_planner/stomp_trajectory.h>
#include <stomp_motion_planner/stomp_utils.h>
#include <stomp_motion_planner/stomp_parameters.h>
#include <stomp_motion_planner/stomp_optimizer.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <spline_smoother/cubic_trajectory.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <boost/shared_ptr.hpp>

#include <map>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

const int num_displays=10;

namespace stomp_motion_planner
{

StompPlannerNode::StompPlannerNode(ros::NodeHandle node_handle) : node_handle_(node_handle)
                                                                  //filter_constraints_chain_("motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request")
{

}

bool StompPlannerNode::init()
{
  // load in some default parameters
  node_handle_.param("trajectory_duration", trajectory_duration_, 3.0);
  node_handle_.param("trajectory_discretization", trajectory_discretization_, 0.03);
  node_handle_.param("use_additional_trajectory_filter", use_trajectory_filter_, true);
  node_handle_.param("minimum_spline_points", minimum_spline_points_, 40);
  node_handle_.param("maximum_spline_points", maximum_spline_points_, 100);

  if(node_handle_.hasParam("joint_velocity_limits")) {
    XmlRpc::XmlRpcValue velocity_limits;
    
    node_handle_.getParam("joint_velocity_limits", velocity_limits);

    if(velocity_limits.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      if(velocity_limits.size() > 0) {
        for(XmlRpc::XmlRpcValue::iterator it = velocity_limits.begin();
            it != velocity_limits.end();
            it++) {
          joint_velocity_limits_[it->first] = it->second;
          ROS_DEBUG_STREAM("Vel limit for " << it->first << " is " << joint_velocity_limits_[it->first]);
        }
      }
    }  
  } 

  //filter_constraints_chain_.configure("filter_chain",node_handle_);

  collision_models_ = new planning_environment::CollisionModels("robot_description");

  if(!collision_models_->loadedModels()) {
    ROS_ERROR("Collision models could not load models");
    return false;
  }

  monitor_ = new planning_environment::PlanningMonitor(collision_models_, &tf_);
  joint_state_monitor_ = new planning_environment::JointStateMonitor();

  for (int i=0; i<num_displays; ++i)
  {
    stringstream pub_name;
    pub_name << "stomp_motion_planner/path_" << i;
    path_display_publishers_.push_back(
        root_handle_.advertise<motion_planning_msgs::DisplayTrajectory>(pub_name.str().c_str(), 10 ));
  }

  monitor_->waitForState();
  monitor_->setUseCollisionMap(true);
  monitor_->startEnvironmentMonitor();

  reference_frame_ = monitor_->getRobotFrameId();

  // build the robot model
  if (!stomp_robot_model_.init(monitor_, reference_frame_))
    return false;

  // load stomp parameters:
  stomp_parameters_.initFromNodeHandle();

  double max_radius_clearance = stomp_robot_model_.getMaxRadiusClearance();

  // initialize the collision space
  if (!stomp_collision_space_.init(monitor_,max_radius_clearance, reference_frame_))
    return false;

  // initialize the visualization publisher:
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>( "stomp_motion_planner/visualization_marker_array", 10 );
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>( "stomp_motion_planner/visualization_marker", 10 );
  stats_publisher_ = root_handle_.advertise<STOMPStatistics>( "stomp_motion_planner/statistics", 10 );

  // advertise the planning service
  plan_kinematic_path_service_ = root_handle_.advertiseService("stomp_motion_planner/plan_path", &StompPlannerNode::planKinematicPath, this);

  filter_joint_trajectory_service_ = root_handle_.advertiseService("stomp_motion_planner/filter_trajectory_with_constraints", &StompPlannerNode::filterJointTrajectory, this);

  if(use_trajectory_filter_)
  {
    filter_trajectory_client_ = root_handle_.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter/filter_trajectory_with_constraints");
    ros::service::waitForService("trajectory_filter/filter_trajectory_with_constraints");
  }

  ROS_INFO("Initalized STOMP planning service...");

  return true;
}

StompPlannerNode::~StompPlannerNode()
{
  delete collision_models_;
  delete monitor_;
  delete joint_state_monitor_;
}

int StompPlannerNode::run()
{
  ros::spin();
  return 0;
}

void StompPlannerNode::clearAnimations()
{
  visualization_msgs::Marker msg;
  msg.points.resize(0);
  msg.header.frame_id = "/base_link";
  msg.header.stamp = ros::Time::now();
  msg.ns = "stomp_endeffector";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::DELETE;
  vis_marker_publisher_.publish(msg);
}


bool StompPlannerNode::planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
//  if (!(req.motion_plan_request.goal_constraints.position_constraints.empty() && req.motion_plan_request.goal_constraints.orientation_constraints.empty()))
//  {
//    ROS_ERROR("STOMP cannot handle pose contraints yet.");
//    return false;
//  }

  sensor_msgs::JointState joint_goal_stomp = motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
  clearAnimations();
  ros::spinOnce();
  ROS_DEBUG("Stomp goal");

  if(joint_goal_stomp.name.size() != joint_goal_stomp.position.size())
  {
    ROS_ERROR("Invalid stomp goal");
    return false;
  }

  for(unsigned int i=0; i<joint_goal_stomp.name.size(); i++)
  {
    ROS_DEBUG("%s %f",joint_goal_stomp.name[i].c_str(),joint_goal_stomp.position[i]);
  }

  ros::WallTime start_time = ros::WallTime::now();
  ROS_DEBUG("Received planning request...");

  // get the planning group:
  const StompRobotModel::StompPlanningGroup* group = stomp_robot_model_.getPlanningGroup(req.motion_plan_request.group_name);

  if (group==NULL)
  {
    ROS_ERROR("Could not load planning group %s", req.motion_plan_request.group_name.c_str());
    return false;
  }

  StompTrajectory trajectory(&stomp_robot_model_, trajectory_duration_, trajectory_discretization_);

  // set the start state:
  stomp_robot_model_.jointStateToArray(req.motion_plan_request.start_state.joint_state, trajectory.getTrajectoryPoint(0));

  ROS_DEBUG_STREAM("Joint state has " << req.motion_plan_request.start_state.joint_state.name.size() << " joints");

  //configure the distance field for the start state
  stomp_collision_space_.setStartState(*group, req.motion_plan_request.start_state);

  //updating collision points for the potential for new attached objects
  //note that this assume that setStartState has been run by stomp_collision_space_
  stomp_robot_model_.generateAttachedObjectCollisionPoints(&(req.motion_plan_request.start_state));
  stomp_robot_model_.populatePlanningGroupCollisionPoints();

  // set the goal state equal to start state, and override the joints specified in the goal
  // joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  stomp_robot_model_.jointStateToArray(motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints), trajectory.getTrajectoryPoint(goal_index));

  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (int i=0; i<group->num_joints_; i++)
  {
    if (group->stomp_joints_[i].wrap_around_)
    {
      int kdl_index = group->stomp_joints_[i].kdl_joint_index_;
      double start = trajectory(0, kdl_index);
      double end = trajectory(goal_index, kdl_index);
      trajectory(goal_index, kdl_index) = start + angles::shortest_angular_distance(start, end);
    }
  }

  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // set the max planning time:
  stomp_parameters_.setPlanningTimeLimit(req.motion_plan_request.allowed_planning_time.toSec());

  // set parameters for exact collision checking:
  std::vector<std::string> group_joint_names = group->getJointNames();
  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  monitor_->prepareForValidityChecks(
      group_joint_names,
      req.motion_plan_request.ordered_collision_operations,
      req.motion_plan_request.allowed_contacts,
      req.motion_plan_request.path_constraints,
      req.motion_plan_request.goal_constraints,
      req.motion_plan_request.link_padding,
      error_code);

  // optimize!
  ros::WallTime create_time = ros::WallTime::now();

  boost::shared_ptr<StompOptimizer> optimizer;
  optimizer.reset(new StompOptimizer(&trajectory, &stomp_robot_model_, group, &stomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, stats_publisher_, &stomp_collision_space_, req.motion_plan_request.path_constraints,
      monitor_));
  ROS_DEBUG("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  optimizer->setSharedPtr(optimizer);
  optimizer->optimize();
  optimizer->resetSharedPtr();
  ROS_DEBUG("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());

  // assume that the trajectory is now optimized, fill in the output structure:

  std::vector<double> velocity_limits(group->num_joints_, std::numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(group->num_joints_);
  for (int i=0; i<group->num_joints_; i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = group->stomp_joints_[i].joint_name_;
    // try to retrieve the joint limits:
    if (joint_velocity_limits_.find(res.trajectory.joint_trajectory.joint_names[i])==joint_velocity_limits_.end())
    {
      joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]] = std::numeric_limits<double>::max();
    }
    velocity_limits[i] = joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]];
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i<=goal_index; i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(group->num_joints_);
    for (int j=0; j<group->num_joints_; j++)
    {
      int kdl_joint_index = stomp_robot_model_.urdfNameToKdlNumber(res.trajectory.joint_trajectory.joint_names[j]);
      res.trajectory.joint_trajectory.points[i].positions[j] = trajectory(i, kdl_joint_index);
    }
    if (i==0)
      res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j<group->num_joints_; j++)
      {
        double d = fabs(res.trajectory.joint_trajectory.points[i].positions[j] - res.trajectory.joint_trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      res.trajectory.joint_trajectory.points[i].time_from_start = res.trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(duration);
    }
  }

  for (int i=0; i<num_displays; ++i)
  {
    int index = lrint(double(i)*(double(goal_index)/double(num_displays-1)));
    if (i==num_displays-1 || index > goal_index)
      index = goal_index;
    if (index < 0)
      index = 0;
    boost::shared_ptr<motion_planning_msgs::DisplayTrajectory> display_trajectory(new motion_planning_msgs::DisplayTrajectory());
    display_trajectory->model_id="pr2";
//    display_trajectory->trajectory.joint_trajectory.header.frame_id = req.motion_plan_request.start_state.joint_state.header.frame_id;
    display_trajectory->trajectory.joint_trajectory.header.frame_id = "base_footprint";
    display_trajectory->trajectory.joint_trajectory.header.stamp = ros::Time::now();
    display_trajectory->trajectory.joint_trajectory.joint_names = res.trajectory.joint_trajectory.joint_names;
    display_trajectory->trajectory.joint_trajectory.points.push_back(res.trajectory.joint_trajectory.points[index]);
    display_trajectory->trajectory.joint_trajectory.header.frame_id = req.motion_plan_request.start_state.joint_state.header.frame_id;
    display_trajectory->robot_state.joint_state =  joint_state_monitor_->getJointStateRealJoints();
    path_display_publishers_[i].publish(display_trajectory);
  }

  ROS_DEBUG("Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("STOMP: Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[goal_index].time_from_start.toSec());
  return true;
}

bool StompPlannerNode::filterJointTrajectory(motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request &req, motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response &res)
{
  return false; // not completely fixed for diamondback
  ros::WallTime start_time = ros::WallTime::now();
  ROS_DEBUG_STREAM("Received filtering request with trajectory size " << req.trajectory.points.size());

  //create a spline from the trajectory
  spline_smoother::CubicTrajectory trajectory_solver;
  spline_smoother::SplineTrajectory spline;

  getLimits(req.trajectory, req.limits);

  for (unsigned int i=0; i< req.trajectory.points.size(); i++)
  {
    req.trajectory.points[i].velocities.resize(req.trajectory.joint_names.size());
  }

  trajectory_solver.parameterize(req.trajectory,req.limits,spline);  

  double smoother_time;
  spline_smoother::getTotalTime(spline, smoother_time);
  
  ROS_DEBUG_STREAM("Total time is " << smoother_time);

  unsigned int NUM_POINTS=100;

  double t = 0.0;
  std::vector<double> times(NUM_POINTS);
  for(unsigned int i = 0; i < NUM_POINTS; i++,t += smoother_time/(1.0*NUM_POINTS)) {
    times[i] = t;
  }

  trajectory_msgs::JointTrajectory jtraj;
  spline_smoother::sampleSplineTrajectory(spline, times, jtraj);

  double planner_time = req.trajectory.points.back().time_from_start.toSec();
  
  t = 0.0;
  for(unsigned int i = 0; i < jtraj.points.size(); i++, t += planner_time/(1.0*NUM_POINTS)) {
    jtraj.points[i].time_from_start = ros::Duration(t);
  }

  ROS_DEBUG_STREAM("Sampled trajectory has " << jtraj.points.size() << " points with " << jtraj.points[0].positions.size() << " joints");

  // get the filter group - will need to figure out
  const StompRobotModel::StompPlanningGroup* group = stomp_robot_model_.getPlanningGroup("right_arm");

  if (group==NULL)
  {
    ROS_ERROR("Could not load planning group %s", "right_arm");
    return false;
  }

  StompTrajectory trajectory(&stomp_robot_model_, group, jtraj);

  //configure the distance field - this should just use current state
  motion_planning_msgs::RobotState robot_state;
  monitor_->getCurrentRobotState(robot_state);

  stomp_robot_model_.jointStateToArray(robot_state.joint_state, trajectory.getTrajectoryPoint(0));
  stomp_collision_space_.setStartState(*group, robot_state);

  //updating collision points for the potential for new attached objects
  //note that this assume that setStartState has been run by stomp_collision_space_

  stomp_robot_model_.generateAttachedObjectCollisionPoints(&(robot_state));
  stomp_robot_model_.populatePlanningGroupCollisionPoints();

  //set the goal state equal to start state, and override the joints specified in the goal
  //joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);

  sensor_msgs::JointState goal_state = motion_planning_msgs::createJointState(req.trajectory.joint_names, jtraj.points.back().positions);

  stomp_robot_model_.jointStateToArray(goal_state, trajectory.getTrajectoryPoint(goal_index));
  
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (int i=0; i<group->num_joints_; i++)
  {
    if (group->stomp_joints_[i].wrap_around_)
    {
      int kdl_index = group->stomp_joints_[i].kdl_joint_index_;
      double start = trajectory(0, kdl_index);
      double end = trajectory(goal_index, kdl_index);
      trajectory(goal_index, kdl_index) = start + angles::shortest_angular_distance(start, end);
    }
  }
  /*
    for(int i = 0; i < trajectory.getNumPoints(); i++) {
    
    ROS_INFO_STREAM(trajectory(i,group->stomp_joints_[0].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[1].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[2].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[3].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[4].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[5].kdl_joint_index_) << " " <<
    trajectory(i,group->stomp_joints_[6].kdl_joint_index_) << " ");
    }
  ROS_INFO_STREAM("Duration is " << trajectory.getDuration());
  */
  //sets other joints
  trajectory.fillInMinJerk();
  trajectory.overwriteTrajectory(jtraj);
  
  // set the max planning time:
  stomp_parameters_.setPlanningTimeLimit(req.allowed_time.toSec());
  
  // optimize!
  StompOptimizer optimizer(&trajectory, &stomp_robot_model_, group, &stomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, stats_publisher_, &stomp_collision_space_,
      req.path_constraints, monitor_);
  optimizer.optimize();
  
  // assume that the trajectory is now optimized, fill in the output structure:

  std::vector<double> velocity_limits(group->num_joints_, std::numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_names.resize(group->num_joints_);
  for (int i=0; i<group->num_joints_; i++)
  {
    res.trajectory.joint_names[i] = group->stomp_joints_[i].joint_name_;
    velocity_limits[i] = joint_limits_[res.trajectory.joint_names[i]].max_velocity;
  }
  
  res.trajectory.header.stamp = ros::Time::now();
  res.trajectory.header.frame_id = reference_frame_;

  // fill in the entire trajectory
  res.trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i< trajectory.getNumPoints(); i++)
  {
    res.trajectory.points[i].positions.resize(group->num_joints_);
    res.trajectory.points[i].velocities.resize(group->num_joints_);
    for (int j=0; j<group->num_joints_; j++)
    {
      int kdl_joint_index = stomp_robot_model_.urdfNameToKdlNumber(res.trajectory.joint_names[j]);
      res.trajectory.points[i].positions[j] = trajectory(i, kdl_joint_index);
    }
    if (i==0)
      res.trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j<group->num_joints_; j++)
      {
        double d = fabs(res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      try {
        res.trajectory.points[i].time_from_start = res.trajectory.points[i-1].time_from_start + ros::Duration(duration);
      } catch(...) {
        ROS_DEBUG_STREAM("Potentially weird duration of " << duration);
      }
    }
  }
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request  next_req;
  motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response next_res;

  if(use_trajectory_filter_) {
    next_req = req;
    next_req.trajectory = res.trajectory;  
    next_req.allowed_time=ros::Duration(1.0);//req.allowed_time/2.0;
    
    if(filter_trajectory_client_.call(next_req, next_res)) {
      ROS_DEBUG_STREAM("Filter call ok. Sent trajectory had " << res.trajectory.points.size() << " points.  Returned trajectory has " << next_res.trajectory.points.size() << " points ");
    } else {
      ROS_DEBUG("Filter call not ok");
    }
    
    res.trajectory = next_res.trajectory;
    res.error_code = next_res.error_code;
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = reference_frame_;
  } else {
    res.error_code.val = res.error_code.val = res.error_code.SUCCESS;
  }

  // for every point in time:
  for (unsigned int i=1; i<res.trajectory.points.size()-1; ++i)
  {
    double dt1 = (res.trajectory.points[i].time_from_start - res.trajectory.points[i-1].time_from_start).toSec();
    double dt2 = (res.trajectory.points[i+1].time_from_start - res.trajectory.points[i].time_from_start).toSec();

    // for every (joint) trajectory
    for (int j=0; j<group->num_joints_; ++j)
    {
      double dx1 = res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j];
      double dx2 = res.trajectory.points[i+1].positions[j] - res.trajectory.points[i].positions[j];

      double v1 = dx1/dt1;
      double v2 = dx2/dt2;

      res.trajectory.points[i].velocities[j] = 0.5*(v1 + v2);
    }
  }

  ROS_INFO("Serviced filter request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.points.back().time_from_start.toSec());
  return true;

}

void StompPlannerNode::getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                                 std::vector<motion_planning_msgs::JointLimits>& limits_out)
{
  int num_joints = trajectory.joint_names.size();
  limits_out.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    std::map<std::string, motion_planning_msgs::JointLimits>::const_iterator limit_it = joint_limits_.find(trajectory.joint_names[i]);
    motion_planning_msgs::JointLimits limits;
    if (limit_it == joint_limits_.end())
    {
      // load the limits from the param server
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/min_position", limits.min_position, -std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_position", limits.max_position, std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_velocity", limits.max_velocity, std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_acceleration", limits.max_acceleration, std::numeric_limits<double>::max());
      bool boolean;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_position_limits", boolean, false);
      limits.has_position_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_velocity_limits", boolean, false);
      limits.has_velocity_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_acceleration_limits", boolean, false);
      limits.has_acceleration_limits = boolean?1:0;
      joint_limits_.insert(make_pair(trajectory.joint_names[i], limits));
    }
    else
    {
      limits = limit_it->second;
    }
    limits_out[i] = limits;
  }
}

} // namespace stomp

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stomp_planner_node");

  //ros::AsyncSpinner spinner(1); // Use 1 thread
  //spinner.start();

  ros::NodeHandle node_handle("~");
  stomp_motion_planner::StompPlannerNode stomp_planner_node(node_handle);
  if (!stomp_planner_node.init())
    return 1;
  //ros::waitForShutdown();
  return stomp_planner_node.run();
}
