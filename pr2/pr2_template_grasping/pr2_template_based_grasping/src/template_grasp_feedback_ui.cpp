/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_grasp_feedback_ui.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <sstream>
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pr2_template_based_grasping/PlanningFeedback.h>
#include <pr2_template_based_grasping/PlanningVisualization.h>
#include <pr2_template_based_grasping/PlanningSummary.h>

using namespace std;
using namespace pr2_template_based_grasping;

bool plotSummary(ros::ServiceClient& sum_client)
{
  PlanningSummary planning_sum;
  if (!sum_client.call(planning_sum))
  {
    return false;
  }

  unsigned int cols = planning_sum.response.score_labels.size();
  for (unsigned int i = 0; i < planning_sum.response.score_labels.size(); i++)
  {
    cout << planning_sum.response.score_labels[i] << "\t";
  }
  cout << endl;

  for (unsigned int i = 0; i < planning_sum.response.score_values.size(); i++)
  {
    if (i % cols == 0)
    {
      cout << i / cols << ": ";
    }
    cout << planning_sum.response.score_values[i];
    cout << "\t";
    if (i % cols == cols - 1)
    {
      cout << endl;
    }
  }

  return true;
}

//bool writeSummaryToBag(ros::ServiceClient& sum_client, rosbag::Bag& bag)
//{
//  PlanningSummary planning_sum;
//  if (!sum_client.call(planning_sum))
//  {
//    return false;
//  }
//
//  ROS_INFO("Writing grasp planning log to bag: %s", bag.getFileName().c_str());
//  try
//  {
//    bag.write("grasp_template_planning_log", ros::Time::now(), planning_sum.response);
//  }
//  catch (rosbag::BagIOException ex)
//  {
//    ROS_ERROR("Problem when writing log file >%s< : %s.",
//        bag.getFileName().c_str(), ex.what());
//
//    return false;
//  }
//
//  return true;
//}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "template_grasp_feedback_ui");
  ros::NodeHandle n;

//  ros::ServiceClient vis_client = n.serviceClient<PlanningVisualization> ("pr2_template_grasp_planner_visualization");
  ros::ServiceClient fb_client = n.serviceClient<PlanningFeedback> ("pr2_template_grasp_planner_feedback");
  ros::ServiceClient sum_client = n.serviceClient<PlanningSummary> ("pr2_template_grasp_planner_logging");

//  string log_filename, bag_filename;
//  log_filename = argv[1];
//  log_filename.append("template_grasping_");
//  stringstream ss;
//  ss << ros::Time::now();
//  string tmp;
//  ss >> tmp;
//  log_filename.append(tmp);
//  bag_filename = log_filename;
//  log_filename.append(".log");
//  bag_filename.append(".bag");

//  ofstream log_file;
//  log_file.open(log_filename.c_str(), std::ios::out | std::ios::app);
//  rosbag::Bag log_bag;
//  ROS_INFO_STREAM("Opening new bag for logging grasp planning results: " << bag_filename);
//  try
//  {
//    log_bag.open(bag_filename, rosbag::bagmode::Write);
//
//  }
//  catch (rosbag::BagIOException ex)
//  {
//    ROS_ERROR("Problem when opening bag file >%s< : %s.",
//        bag_filename.c_str(), ex.what());
//  }

  string feedback_input = "i";
  while (ros::ok() && feedback_input != "q")
  {
    ROS_INFO_STREAM("Please, wait until the robot finishes the grasp execution. "
        "Then label the grasp by entering "
        "the first character of:" << endl <<
        "\t (f)ail or" << endl <<
        "\t (s)uccess or"<< endl <<
        "\t (n)neither of both or"<< endl << endl <<
        "\t (p)lot the values of the matching distance or" << endl <<
        "\t (q)uit" << endl << "followed by enter.");
    cin >> feedback_input;

    PlanningFeedback fb_service;
    fb_service.request.action = -1;
    if (feedback_input == "f")
    {
      fb_service.request.action = PlanningFeedback::Request::CHANGE_SUCCESS_AND_DO_UPGRADE;
      fb_service.request.success = 0.0;
//      log_file << "0" << endl;
      ROS_INFO("Returned FAILED ATTEMPT to planning server");

//      // log result
//      if (!writeSummaryToBag(sum_client, log_bag))
//      {
//        ROS_WARN("Failed to write planning log to bag file...");
//      }
    }
    else if (feedback_input == "s")
    {
      fb_service.request.action = PlanningFeedback::Request::CHANGE_SUCCESS_AND_DO_UPGRADE;
      fb_service.request.success = 1.0;

//      log_file << "1" << endl;
      ROS_INFO("Returned SUCCESSFUL ATTEMPT to planning server");

      // log result
//      if (!writeSummaryToBag(sum_client, log_bag))
//      {
//        ROS_WARN("Failed to write planning log to bag file...");
//      }
    }
    else if (feedback_input == "n")
    {
      fb_service.request.action = PlanningFeedback::Request::CHANGE_SUCCESS_AND_DO_UPGRADE;
      fb_service.request.success = 0.5;

//      log_file << "1" << endl;
      ROS_INFO("Returned NO DECISION about success to planning server");

      // log result
//      if (!writeSummaryToBag(sum_client, log_bag))
//      {
//        ROS_WARN("Failed to write planning log to bag file...");
//      }
    }
    else if (feedback_input == "p")
    {
      if (!plotSummary(sum_client))
      {
        ROS_WARN("Failed to write planning log to bag file...");
      }
    }

    if (feedback_input == "f" || feedback_input == "s" || feedback_input == "n")
    {
      if (!fb_client.call(fb_service))
      {
        ROS_ERROR("Feedback service failed. Grasp Library was not updated.");
      }
    }
  }

//  log_file.close();
//  log_bag.close();

  return 0;
}
