/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_gui.cpp

  \author	Peter Pastor
  \date		Jun 18, 2011

 *********************************************************************/

// system includes
#include <QtGui>
#include <iomanip>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/data_sample_utilities.h>

#include <actionlib/client/simple_action_client.h>
#include <dmp_behavior_actions/LearningFromDemonstrationAction.h>
#include <dynamic_movement_primitive/TypeMsg.h>

#include <robot_info/robot_info.h>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>

// local includes
#include <dynamic_movement_primitive_gui/dynamic_movement_primitive_gui.h>

namespace dynamic_movement_primitive_gui
{
static const std::string TRAJECTORY_LIST_NAME = std::string("TrajectoryList");
static const std::string DMP_LIST_NAME = std::string("DMPList");
static const std::string ROBOT_PART_LIST_NAME = std::string("RobotPartList");
static const double EXTRA_TIME = 0.5;
//static const std::string BAG_FILE_ENDING = ".bag";

DynamicMovementPrimitiveGUI::DynamicMovementPrimitiveGUI(ros::NodeHandle node_handle, QWidget* parent, Qt::WFlags flags) :
  QMainWindow(parent, flags), node_handle_(node_handle), recording_joint_states_(false), skill_library_client_(node_handle)
{
  ROS_VERIFY(readParams());

  // this sets up GUI
  Ui_dynamic_movement_primitive_gui::setupUi(this);

  // signals/slots mechanism in action
  ROS_VERIFY(connect(trajectory_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*))));
  ROS_VERIFY(connect(dmp_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*))));
  ROS_VERIFY(connect(record_button_, SIGNAL(clicked()), this, SLOT(record())));
  ROS_VERIFY(connect(learn_button_, SIGNAL(clicked()), this, SLOT(learn())));
  ROS_VERIFY(connect(simulate_button_, SIGNAL(clicked()), this, SLOT(simulate())));


  ROS_VERIFY(connect(this, SIGNAL(insertDescriptionsSignal(QListWidget*, const std::vector<task_recorder2_msgs::Description>&)),
                     this, SLOT(insertDescriptions(QListWidget*, const std::vector<task_recorder2_msgs::Description>&))));
  ROS_VERIFY(connect(this, SIGNAL(insertDescriptionSignal(QListWidget*, const task_recorder2_msgs::Description&)),
                     this, SLOT(insertDescription(QListWidget*, const task_recorder2_msgs::Description&))));
  ROS_VERIFY(connect(this, SIGNAL(getAllDescriptionsSignal(QListWidget*, std::vector<task_recorder2_msgs::Description>&)),
                     this, SLOT(getAllDescriptions(QListWidget*, std::vector<task_recorder2_msgs::Description>&))));
  ROS_VERIFY(connect(this, SIGNAL(getSelectedDescriptionsSignal(QListWidget*, std::vector<task_recorder2_msgs::Description>&)),
                     this, SLOT(getSelectedDescriptions(QListWidget*, std::vector<task_recorder2_msgs::Description>&))));
  ROS_VERIFY(connect(this, SIGNAL(removeSelectedItemsSignal(QListWidget*)),
                     this, SLOT(removeSelectedItems(QListWidget*))));
  // gui_utilities::DescriptionList trajectory_list(trajectory_list_, TRAJECTORY_LIST_NAME);
  // widget_list_map_.insert(WidgetDescriptionListPair(trajectory_list_, trajectory_list));
  // gui_utilities::DescriptionList dmp_list(dmp_list_, DMP_LIST_NAME);
  // widget_list_map_.insert(WidgetDescriptionListPair(dmp_list_, dmp_list));

  trajectory_list_->setObjectName(QString(TRAJECTORY_LIST_NAME.c_str()));
  dmp_list_->setObjectName(QString(DMP_LIST_NAME.c_str()));
  robot_part_list_->setObjectName(QString(ROBOT_PART_LIST_NAME.c_str()));

  // gui_utilities::DescriptionList robot_part_list(robot_part_list_, ROBOT_PART_LIST_NAME, false);
  // widget_list_map_.insert(WidgetDescriptionListPair(robot_part_list_, robot_part_list));

  std::vector<std::string> robot_part_names = robot_info::RobotInfo::getRobotPartNames();
  for (int i = 0; i < (int)robot_part_names.size(); ++i)
  {
    task_recorder2_msgs::Description description;
    description.description = robot_part_names[i];
    insertDescriptionSignal(robot_part_list_, description);
    // widget_list_map_.at(robot_part_list_).insert(description);
  }

  load(recorder_data_directory_name_, trajectory_list_);
  load(dmp_data_directory_name_, dmp_list_);

//  ROS_VERIFY(task_recorder2_utilities::readDescriptionLabels(node_handle, description_names_));
//  for (int i = 0; i < (int)description_names_.size(); ++i)
//  {
//    description_cbox_->addItem(QString(description_names_[i].c_str()));
//    description_cbox_->setEnabled(true);
//  }
//  description_cbox_->setDuplicatesEnabled(false);
//  description_combo_box_->setDuplicatesEnabled(false);
//  id_combo_box_->setDuplicatesEnabled(false);

  std::vector<std::string> robot_parts = robot_info::RobotInfo::getRobotPartNames();
  for (int i = 0; i < (int)robot_parts.size(); ++i)
  {
    if(robot_info::RobotInfo::isRightArm(robot_parts[i]))
    {
      right_arm_ik_.reset(new inverse_kinematics::InverseKinematicsWithNullspaceOptimization(robot_parts[i]));
      ROS_VERIFY(right_arm_ik_->initialize());
    }
    else if(robot_info::RobotInfo::isLeftArm(robot_parts[i]))
    {
      left_arm_ik_.reset(new inverse_kinematics::InverseKinematicsWithNullspaceOptimization(robot_parts[i]));
      ROS_VERIFY(left_arm_ik_->initialize());
    }
  }

}

bool DynamicMovementPrimitiveGUI::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_samples_per_second", num_samples_per_second_));

  std::string recorder_package_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
  std::string recorder_data_directory_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
  recorder_data_directory_name_ = task_recorder2_utilities::getDirectoryPath(recorder_package_name, recorder_data_directory_name);

  std::string data_directory_name;
  ROS_VERIFY(usc_utilities::read(node_handle_, "data_directory_name", data_directory_name));
  dmp_data_directory_name_ = task_recorder2_utilities::getDirectoryPath(recorder_package_name, data_directory_name);
  usc_utilities::appendTrailingSlash(dmp_data_directory_name_);
  // TODO: make this an option
  dmp_data_directory_name_.append(dynamic_movement_primitive::DMPUtilitiesMsg::ICRA2009);

  ROS_VERIFY(task_recorder2_utilities::checkAndCreateDirectories(recorder_data_directory_name_));
  ROS_INFO("Setting TaskRecorderIO data directory name to >%s<.", recorder_data_directory_name_.c_str());
  return true;
}

void DynamicMovementPrimitiveGUI::record()
{
  if (!recording_joint_states_)
  {
    description_.description = description_edit_->text().toStdString();
    if(description_.description.empty())
    {
      setStatusReport("No description provided, not recording...", ERROR);
      return;
    }
    description_.id = id_spin_box_->value();
    setStatusReport("Start recording...", INFO);
    ROS_VERIFY(task_recorder_manager_client_.startRecording(description_, start_time_));
    start_time_ = start_time_ + ros::Duration(0.1);

  }
  else
  {
    std::vector<task_recorder2_msgs::Description> robot_part_descriptions;
    // if(!widget_list_map_.at(robot_part_list_).getDescriptions(robot_part_descriptions))
    getSelectedDescriptionsSignal(robot_part_list_, robot_part_descriptions);
    if(robot_part_descriptions.empty())
    {
      setStatusReport("No robot part selected...", ERROR);
      return;
    }

    std::vector<std::string> robot_part_names;
    for (int i = 0; i < (int)robot_part_descriptions.size(); ++i)
    {
      robot_part_names.push_back(robot_part_descriptions[i].description);
    }
    std::vector<std::string> joint_names;
    robot_info::RobotInfo::getVariableNames(robot_part_names, joint_names);

    std::vector<std::string> joint_names_th;
    std::vector<std::string> joint_names_only = joint_names;
    robot_info::RobotInfo::extractJointNames(joint_names_only);
    for (int i = 0; i < (int)joint_names_only.size(); ++i)
    {
      joint_names_th.push_back(joint_names_only[i] + std::string("_th"));
      ROS_DEBUG("Recording joint named: >%s<.", joint_names_th.back().c_str());
    }

    std::vector<std::string> all_but_joint_names = joint_names;
    robot_info::RobotInfo::removeJointNames(all_but_joint_names);
    joint_names_th.insert(joint_names_th.end(), all_but_joint_names.begin(), all_but_joint_names.end());

    // robot_info::RobotInfo::extractJointNames(only_joint_names);
//    std::vector<std::string> all_but_joint_names;
//    for(int i = 0; i < (int)joint_names.size(); ++i)
//    {
//      bool found = false;
//      for(int j = 0; !found && j < (int)joint_names_only.size(); ++j)
//      {
//        if(joint_names[i].compare(joint_names_only[j]) == 0)
//        {
//          found = true;
//        }
//      }
//      if(!found)
//      {
//        all_but_joint_names.push_back(joint_names[i]);
//      }
//    }
//    joint_names_th.insert(joint_names_th.end(), all_but_joint_names.begin(), all_but_joint_names.end());

    setStatusReport("Stop recording...", INFO);
    ros::Time end_time = ros::Time::now();
    ros::Duration(EXTRA_TIME).sleep();
    const int num_samples = (end_time - start_time_).toSec() * num_samples_per_second_;
    ROS_INFO("Asking for >%i< messages.", num_samples);
    data_samples_.clear();
    ROS_VERIFY(task_recorder_manager_client_.stopRecording(start_time_, end_time, num_samples, joint_names_th, data_samples_));
    ROS_INFO("Got >%i< messages.", (int)data_samples_.size());

    std::vector<sensor_msgs::JointState> joint_states;
    for (int i = 0; i < (int)data_samples_.size(); ++i)
    {
      sensor_msgs::JointState joint_state;
      joint_state.header = data_samples_[i].header;
      joint_state.name = joint_names;
      for (int j = 0; j < (int)data_samples_[i].data.size(); ++j)
      {
        joint_state.position.push_back(data_samples_[i].data[j]);
      }
      joint_states.push_back(joint_state);
    }
    std::string abs_file_name = recorder_data_directory_name_ + task_recorder2_utilities::getBagFileName(description_);
    std::string topic_name = std::string("/joint_states");
    ROS_VERIFY(usc_utilities::FileIO<sensor_msgs::JointState>::writeToBagFileWithTimeStamps(joint_states, topic_name, abs_file_name));

    load(recorder_data_directory_name_, trajectory_list_);
  }
  recording_joint_states_ = !recording_joint_states_;
}

void DynamicMovementPrimitiveGUI::learn()
{
  std::vector<task_recorder2_msgs::Description> robot_part_descriptions;
  // if(!widget_list_map_.at(robot_part_list_).getDescriptions(robot_part_descriptions))
  getSelectedDescriptionsSignal(robot_part_list_, robot_part_descriptions);
  if(robot_part_descriptions.empty())
  {
    setStatusReport("No robot part selected...", ERROR);
    return;
  }

  actionlib::SimpleActionClient<dmp_behavior_actions::LearningFromDemonstrationAction> action_client("/Behaviors/learningFromDemonstration", true);
  bool server_online = false;
  while(!server_online)
  {
    if(action_client.waitForServer(ros::Duration(0.5)))
    {
      server_online = true;
    }
    else
    {
      ROS_WARN("Waiting for action server to come online.");
    }
    ros::spinOnce();
  }

  std::vector<std::string> robot_part_names;
  for (int i = 0; i < (int)robot_part_descriptions.size(); ++i)
  {
    robot_part_names.push_back(robot_part_descriptions[i].description);
  }

  std::vector<task_recorder2_msgs::Description> trajectory_descriptions;
  // if(!widget_list_map_.at(trajectory_list_).getDescriptions(trajectory_descriptions))
  getSelectedDescriptionsSignal(trajectory_list_, trajectory_descriptions);
  if(trajectory_descriptions.empty())
  {
    setStatusReport("No trajectory selected...", ERROR);
    return;
  }

  for (int i = 0; i < (int)trajectory_descriptions.size(); ++i)
  {
    dmp_behavior_actions::LearningFromDemonstrationGoal goal;
    std::string dmp_name = task_recorder2_utilities::getFileName(trajectory_descriptions[i]);
    std::string bag_file_name = dmp_name;
    task_recorder2_utilities::appendBagFileAppendix(bag_file_name);
    goal.joint_states_bag_file_name = bag_file_name;
    goal.robot_part_names_from_trajectory = robot_part_names;
    goal.object.name = dmp_name;
    dynamic_movement_primitive::TypeMsg type;
    if(task_radio_button_->isChecked())
    {
      type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_CARTESIAN_AND_JOINT_SPACE;
    }
    else
    {
      type.type = dynamic_movement_primitive::TypeMsg::DISCRETE_JOINT_SPACE;
    }
    goal.type = type;

    action_client.sendGoal(goal);
    bool got_result = false;
    while(!got_result)
    {
      if(action_client.waitForResult(ros::Duration(0.5)))
      {
        got_result = true;
      }
      else
      {
        setStatusReport("Waiting for result...", WARN);
      }
      ros::spinOnce();
    }

    if(action_client.getResult()->result == dmp_behavior_actions::LearningFromDemonstrationResult::SUCCEEDED)
    {
      setStatusReport("Learned DMP " + dmp_name + ".", INFO);
    }
    else
    {
      setStatusReport("Problems while learning DMP " + dmp_name + ".", ERROR);
    }

    load(dmp_data_directory_name_, dmp_list_);
  }
}

void DynamicMovementPrimitiveGUI::load(const std::string directory_name, QListWidget *list)
{
  boost::filesystem::path path = boost::filesystem::path(directory_name);
  std::vector<std::string> descriptions;
  ROS_VERIFY(task_recorder2_utilities::getDirectoryList(path, descriptions));

  if(list->count() > 0)
  {
    list->clear();
  }
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    task_recorder2_msgs::Description description;
    task_recorder2_utilities::removeBagFileAppendix(descriptions[i]);
    ROS_VERIFY(task_recorder2_utilities::parseDescriptionString(descriptions[i], description.description, description.id));
    // widget_list_map_.at(list).insert(description);
    insertDescriptionSignal(list, description);
  }
}

bool DynamicMovementPrimitiveGUI::getDMPName(std::string& dmp_name)
{
  std::vector<task_recorder2_msgs::Description> dmp_descriptions;
  // if(!widget_list_map_.at(dmp_list_).getDescriptions(dmp_descriptions))
  getSelectedDescriptionsSignal(dmp_list_, dmp_descriptions);
  if(dmp_descriptions.empty())
  {
    setStatusReport("No DMP selected...", ERROR);
    return false;
  }
  ROS_ASSERT_MSG(dmp_descriptions.size() == 1, "There should only be 1 DMP selected. This should never happen.");
  dmp_name = task_recorder2_utilities::getFileName(dmp_descriptions[0]);
  return true;
}

void DynamicMovementPrimitiveGUI::simulate()
{
  std::string dmp_name;
  if(!getDMPName(dmp_name))
  {
    return;
  }

  dmp_lib::DMPPtr dmp;
  ROS_VERIFY(skill_library_client_.get(dmp_name, dmp));
  ROS_VERIFY(dmp->setup());

  dmp_lib::Trajectory trajectory;
  double initial_duration;
  ROS_VERIFY(dmp->getInitialDuration(initial_duration));
  ROS_VERIFY(dmp->propagateFull(trajectory, initial_duration));
  ROS_INFO_STREAM(trajectory.getInfo());

  std::vector<std::string> dmp_names = dmp->getVariableNames();
  for (int i = 0; i < (int)dmp_names.size(); ++i)
  {
    ROS_INFO("Name (%i): %s", i, dmp_names[i].c_str());
  }

  bool right_arm = robot_info::RobotInfo::containsRightArm(dmp_names);
  bool left_arm = robot_info::RobotInfo::containsLeftArm(dmp_names);

  if(right_arm)
  {
    std::vector<std::string> right_endeffector_names = robot_info::RobotInfo::getRightEndeffectorNames();
    dmp_lib::Trajectory right_arm_endeffector_trajectory = trajectory;
    ROS_VERIFY(right_arm_endeffector_trajectory.onlyKeep(right_endeffector_names));
    dmp_lib::Trajectory right_arm_rest_posture_trajectory = trajectory;
    std::vector<std::string> right_arm_joint_names;
    ROS_VERIFY(robot_info::RobotInfo::getRightArmJointNames(right_arm_joint_names));
    ROS_VERIFY(right_arm_rest_posture_trajectory.onlyKeep(right_arm_joint_names));
    simulate(right_arm_endeffector_trajectory, right_arm_rest_posture_trajectory, right_arm_ik_);
  }
  if(left_arm)
  {
    std::vector<std::string> left_endeffector_names = robot_info::RobotInfo::getLeftEndeffectorNames();
    dmp_lib::Trajectory left_arm_endeffector_trajectory = trajectory;
    ROS_VERIFY(left_arm_endeffector_trajectory.onlyKeep(left_endeffector_names));
    dmp_lib::Trajectory left_arm_rest_posture_trajectory = trajectory;
    std::vector<std::string> left_arm_joint_names;
    ROS_VERIFY(robot_info::RobotInfo::getLeftArmJointNames(left_arm_joint_names));
    ROS_VERIFY(left_arm_rest_posture_trajectory.onlyKeep(left_arm_joint_names));
    simulate(left_arm_endeffector_trajectory, left_arm_rest_posture_trajectory, left_arm_ik_);
  }
}

void DynamicMovementPrimitiveGUI::simulate(dmp_lib::Trajectory& endeffector, dmp_lib::Trajectory& restposture,
                                           boost::shared_ptr<inverse_kinematics::InverseKinematicsWithNullspaceOptimization> ik)
{
  ROS_ASSERT(endeffector.getNumContainedSamples() == restposture.getNumContainedSamples());

  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<Eigen::VectorXd> rest_postures;
  Eigen::VectorXd positions = Eigen::VectorXd::Zero((Eigen::DenseIndex)endeffector.getDimension());
  Eigen::VectorXd rest_posture = Eigen::VectorXd::Zero((Eigen::DenseIndex)restposture.getDimension());
  double dt = 1.0 / endeffector.getSamplingFrequency();
  for (int i = 0; i < endeffector.getNumContainedSamples(); ++i)
  {
    ROS_VERIFY(endeffector.getTrajectoryPosition(i, positions));
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time(i * dt);
    pose.pose.position.x = positions(0);
    pose.pose.position.y = positions(1);
    pose.pose.position.z = positions(2);
    pose.pose.orientation.w = positions(3);
    pose.pose.orientation.x = positions(4);
    pose.pose.orientation.y = positions(5);
    pose.pose.orientation.z = positions(6);
    poses.push_back(pose);

    ROS_VERIFY(endeffector.getTrajectoryPosition(i, rest_posture));
    rest_postures.push_back(rest_posture);
  }

  std::vector<Eigen::VectorXd> joint_angles;
  ROS_VERIFY(ik->ik(poses, rest_postures, rest_postures[0], joint_angles));
}

void DynamicMovementPrimitiveGUI::labelSelected(QListWidgetItem* current, QListWidgetItem* previous)
{
  simulate_button_->setEnabled(false);
  execute_button_->setEnabled(false);

  // if(widget_list_map_.at(current->listWidget()).getName().compare(TRAJECTORY_LIST_NAME) == 0)
  if(current->listWidget() == trajectory_list_)
  {
    dmp_space_frame_->setEnabled(true);
    task_radio_button_->setEnabled(true);
    joint_radio_button_->setEnabled(true);
    learn_button_->setEnabled(true);
  }
  // else if(widget_list_map_.at(current->listWidget()).getName().compare(DMP_LIST_NAME) == 0)
  else if(current->listWidget() == dmp_list_)
  {
    simulate_button_->setEnabled(true);
    execute_button_->setEnabled(true);
  }
  // else if(widget_list_map_.at(current->listWidget()).getName().compare(ROBOT_PART_LIST_NAME) == 0)
  else if(current->listWidget() == robot_part_list_)
  {

  }
  else
  {
    ROS_ASSERT_MSG(false, "Unknown list name >%s<. This should never happen.",
                   current->listWidget()->objectName().toStdString().c_str());
  }

}

void DynamicMovementPrimitiveGUI::clearStatusReport()
{
  status_text_edit_->clear();
}

void DynamicMovementPrimitiveGUI::setStatusReport(const std::string& status_report, const StatusReportMode mode)
{
  ROS_INFO_STREAM(status_report);
  std::string status_string;
  std::string debug_color = "#000000"; // black
  std::string info_color = "#006400"; // dark green
  std::string warn_color = "#FF6600"; // orange
  std::string error_color = "#FF0000"; // red
  std::string fatal_color = "#9400D3"; // dark violoet
  switch (mode)
  {
    case DEBUG:
    {
      status_string.assign("<span style=\"color: " + debug_color + "\">" + status_report + "</span><br />");
      break;
    }
    case INFO:
    {
      status_string.assign("<span style=\"color: " + info_color + "\">" + status_report + "</span><br />");

      break;
    }
    case WARN:
    {
      status_string.assign("<span style=\"color: " + warn_color + "\">" + status_report + "</span><br />");

      break;
    }
    case ERROR:
    {
      status_string.assign("<span style=\"color: " + error_color + "\">" + status_report + "</span><br />");

      break;
    }
    case FATAL:
    {
      status_string.assign("<span style=\"color: " + fatal_color + "\">" + status_report + "</span><br />");

      break;
    }
    default:
    {
      status_string.assign("<span style=\"color: " + warn_color + "\">(Unknown mode) " + status_report + "</span><br />");
      break;
    }
  }
  status_text_edit_->moveCursor(QTextCursor::End);
  status_text_edit_->insertHtml(QString(status_string.c_str()));
  QScrollBar* scroll_bar = status_text_edit_->verticalScrollBar();
  scroll_bar->setValue(scroll_bar->maximum());
}

void DynamicMovementPrimitiveGUI::insertDescription(QListWidget* list_widget,
                                                    const task_recorder2_msgs::Description& description)
{
  task_recorder2_msgs::Description::Ptr list_description(new task_recorder2_msgs::Description(description));
  std::string list_item_text = task_recorder2_utilities::getFileName(description);
  // list_item_text.append(std::string(" trial:") + usc_utilities::getString(description.trial));
  QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(list_item_text));
  item->setData(DescriptionRole, QVariant::fromValue(list_description));
  list_widget->addItem(item);
}

void DynamicMovementPrimitiveGUI::insertDescriptions(QListWidget* list_widget,
                                        const std::vector<task_recorder2_msgs::Description>& descriptions)
{
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    insertDescriptionSignal(list_widget, descriptions[i]);
  }
}

void DynamicMovementPrimitiveGUI::removeSelectedItems(QListWidget* list_widget)
{
  QList<QListWidgetItem*> items = list_widget->selectedItems ();
  for (int i = 0; i < (int)items.size(); ++i)
  {
     delete items[i];
  }
}

void DynamicMovementPrimitiveGUI::getAllDescriptions(QListWidget* list_widget,
                                        std::vector<task_recorder2_msgs::Description>& descriptions)
{
  descriptions.clear();
  for (int i = 0; i < list_widget->count(); ++i)
  {
    task_recorder2_msgs::Description::Ptr description_ptr = list_widget->item(i)->data(DescriptionRole).value<task_recorder2_msgs::Description::Ptr>();
    descriptions.push_back(*description_ptr);
  }
}

void DynamicMovementPrimitiveGUI::getSelectedDescriptions(QListWidget* list_widget,
                                             std::vector<task_recorder2_msgs::Description>& descriptions)
{
  QList<QListWidgetItem*> items = list_widget->selectedItems ();
  QList<QListWidgetItem*>::const_iterator ci;
  for(ci = items.begin(); ci != items.end(); ++ci)
  {
    task_recorder2_msgs::Description::Ptr description_ptr = (*ci)->data(DescriptionRole).value<task_recorder2_msgs::Description::Ptr>();
    descriptions.push_back(*description_ptr);
  }
}


}
