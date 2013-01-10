/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         grasp_demo_library.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <string>
#include <vector>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grasp_template_planning/grasp_demo_library.h>

using namespace std;
using namespace boost::filesystem;

namespace grasp_template_planning
{

GraspDemoLibrary::GraspDemoLibrary(const string& grasp_demonstrations_path,
    const string& grasp_library_file)
{
  grasp_demonstrations_path_ = grasp_demonstrations_path;
  grasp_library_file_ = grasp_library_file;

  dropDemonstrationData();
  dropLibraryData();
}

bool GraspDemoLibrary::getAllDemonstrationFilenames(vector<string>& container) const
{
  path dir_path(grasp_demonstrations_path_);

  if (!exists(dir_path))
    return false;

  directory_iterator end_itr; //default construction yields past-the-end
  for (directory_iterator itr(dir_path); itr != end_itr; ++itr)
  {
    if (!is_directory(*itr))
    {
      string filename = itr->path().filename().string();
      if(filename.substr(0,1) != ".") //ignore files starting with a '.'
        container.push_back(filename);
    }
  }
  return true;
}

void GraspDemoLibrary::dropDemonstrationData()
{
  events_.reset();
  objects_.reset();
  gripper_poses_.reset();
  viewpoint_transforms_.reset();
  table_poses_.reset();
  fingerpositions_.reset();
}

void GraspDemoLibrary::dropLibraryData()
{
  analysis_msgs_.reset();
}

bool GraspDemoLibrary::isDemonstrationLoaded() const
{
  return (events_ != NULL && objects_ != NULL && gripper_poses_ != NULL
      && /*joint_states_ != NULL &&*/viewpoint_transforms_ != NULL
      && table_poses_ != NULL); //fingerspreads are not necessary
}

bool GraspDemoLibrary::removeLibraryFileFromFilesystem()
{
  path dir_path(grasp_library_file_);

  if (!exists(dir_path))
  {
    return false;
  }
  remove(dir_path);

  return true;
}

bool GraspDemoLibrary::loadDemonstration(const string& filename)
{
  demo_filename_ = filename;

  string bag_file_path_ = grasp_demonstrations_path_;
  bag_file_path_.append(filename);

  /* reset pointers */
  events_.reset(new GraspDemoEventMap());
  objects_.reset(new GraspDemoObjectMap());
  gripper_poses_.reset(new GraspDemoPoseMap());
  table_poses_.reset(new GraspDemoPoseMap());
  viewpoint_transforms_.reset(new GraspDemoPoseMap());
  fingerpositions_.reset(new GraspDemoVecDoubleMap());

  ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Reading from grasp demonstration file: %s",
      bag_file_path_.c_str());
  try
  {
    rosbag::Bag bag(bag_file_path_, rosbag::bagmode::Read);

    vector<string> q_topics;
    q_topics.clear();
    q_topics.push_back(topicGraspDemoEvents());
    q_topics.push_back(topicGraspDemoObjectCluster());
    q_topics.push_back(topicGraspDemoGripperPose());
    q_topics.push_back(topicViewpointTransform());
    q_topics.push_back(topicGraspDemoTable());
    q_topics.push_back(topicFingerpositions());
    rosbag::View view(bag, rosbag::TopicQuery(q_topics));

    BOOST_FOREACH(rosbag::MessageInstance const msg_instance, view)
          {

            /* grasp event */
            SimpleLabel::ConstPtr ge_msg = msg_instance.instantiate<SimpleLabel> ();
            if (ge_msg != NULL)
            {
              events_->insert(make_pair(ge_msg->event_label, make_pair(msg_instance.getTime(), *ge_msg)));
              continue;
            }

            /* fingerpositions */
            DoubleVector::ConstPtr fings_msg = msg_instance.instantiate<DoubleVector> ();
            if (fings_msg != NULL)
            {
              fingerpositions_->insert(make_pair(msg_instance.getTime(), *fings_msg));
              continue;
            }

            /* object detector point cloud */
            if (msg_instance.getTopic() == topicGraspDemoObjectCluster() || ("/" + msg_instance.getTopic()
                == topicGraspDemoObjectCluster()))
            {
              sensor_msgs::PointCloud2::ConstPtr pc_msg = msg_instance.instantiate<sensor_msgs::PointCloud2> ();
              if (pc_msg != NULL)
              {
                objects_->insert(make_pair(msg_instance.getTime(), *pc_msg));
                continue;
              }
            }

            /* table frame */
            if (msg_instance.getTopic() == topicGraspDemoTable() || ("/" + msg_instance.getTopic()
                == topicGraspDemoTable()))
            {

              geometry_msgs::PoseStamped::ConstPtr tp_msg = msg_instance.instantiate<geometry_msgs::PoseStamped> ();
              if (tp_msg != NULL)
              {
                table_poses_->insert(make_pair(msg_instance.getTime(), *tp_msg));
                continue;
              }
            }

            /* frame transform */
            geometry_msgs::PoseStamped::ConstPtr gp_msg = msg_instance.instantiate<geometry_msgs::PoseStamped> ();
            if (gp_msg != NULL)
            {
              string tpic = msg_instance.getTopic();

              if (tpic.compare(topicGraspDemoGripperPose().c_str()) == 0)
              {
                gripper_poses_->insert(make_pair(msg_instance.getTime(), *gp_msg));
              }
              else if (tpic.compare(topicViewpointTransform().c_str()) == 0)
              {
                viewpoint_transforms_->insert(make_pair(msg_instance.getTime(), *gp_msg));
              }
              continue;
            }
          }
    bag.close();
  }
  catch (rosbag::BagIOException ex)
  {
    ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Problem when reading from grasp "
        "demonstration file >%s< : %s.", bag_file_path_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool GraspDemoLibrary::isIgnored(GraspAnalysis::ConstPtr ana) const
{
  for (vector<string>::const_iterator it = ignored_.begin(); it != ignored_.end(); it++)
  {
    if (it->compare(ana->demo_filename) == 0)
    {
      return true;
    }
  }

  return false;
}

bool GraspDemoLibrary::loadLibrary()
{
  /* reset pointer */
  analysis_msgs_.reset(new vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > ());

  ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Reading from grasp library file: %s",
      grasp_library_file_.c_str());
  try
  {
    rosbag::Bag bag(grasp_library_file_, rosbag::bagmode::Read);

    vector<string> q_topics;
    q_topics.clear();
    q_topics.push_back(topicAnalyzedGrasps());
    rosbag::View view(bag, rosbag::TopicQuery(q_topics));

    BOOST_FOREACH(rosbag::MessageInstance const msg_instance, view)
          {

            /* grasp analysis */
            GraspAnalysis::ConstPtr ga_msg = msg_instance.instantiate<GraspAnalysis> ();
            if (ga_msg != NULL)
            {
              if (!isIgnored(ga_msg))
              {
                analysis_msgs_->push_back(*ga_msg);
              }
            }

          }
    bag.close();
  }
  catch (rosbag::BagIOException ex)
  {
    ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Problem when reading from grasp library file >%s< : %s.",
        grasp_library_file_.c_str(), ex.what());
    return false;
  }
  return true;
}

bool GraspDemoLibrary::addAnalysisToLib(const GraspAnalysis& msg)
{
  ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Writing to grasp library file: %s",
      grasp_library_file_.c_str());
  try
  {
    rosbag::Bag bag;

    ifstream file_existance_check(grasp_library_file_.c_str());
    if (file_existance_check)
    {
      bag.open(grasp_library_file_, rosbag::bagmode::Append);
    }
    else
    {
      bag.open(grasp_library_file_, rosbag::bagmode::Write);
    }

    bag.write(topicAnalyzedGrasps(), ros::Time::now(), msg);

    bag.close();
  }
  catch (rosbag::BagIOException ex)
  {
    ROS_DEBUG("grasp_template_planning::GraspDemoLibrary: Problem when writing to grasp "
        "library file >%s< : %s.", grasp_library_file_.c_str(), ex.what());
    return false;
  }

  return true;
}

} //namespace
