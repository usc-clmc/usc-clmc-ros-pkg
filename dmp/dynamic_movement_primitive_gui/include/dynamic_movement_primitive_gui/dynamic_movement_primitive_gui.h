/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dynamic_movement_primitive_gui.h

  \author	Peter Pastor
  \date		Jun 18, 2011

 *********************************************************************/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include <task_recorder2/task_recorder.h>
#include <task_recorder2/task_recorder_manager_client.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/Description.h>

#include <skill_library/skill_library_client.h>

#include <inverse_kinematics/inverse_kinematics_with_nullspace_optimization.h>

#include <gui_utilities/description_list.h>

// local includes
#include <dynamic_movement_primitive_gui/dynamic_movement_primitive_gui_main_window.h>

namespace dynamic_movement_primitive_gui
{

class DynamicMovementPrimitiveGUI : public QMainWindow, public Ui::dynamic_movement_primitive_gui
{
  Q_OBJECT

public:

  typedef std::pair<QListWidget*, gui_utilities::DescriptionList> WidgetDescriptionListPair;
  typedef std::pair<int, QWidget*> LabelToWidgetPair;

  /*! Constructor
   * @param node_handle
   * @param parent
   * @param flags
   */
  DynamicMovementPrimitiveGUI(ros::NodeHandle node_handle, QWidget* parent = 0, Qt::WFlags flags = 0);
  /*! Destructor
   */
  virtual ~DynamicMovementPrimitiveGUI() {};

  enum StatusReportMode
  {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL
  };
  void setStatusReport(const std::string& status_report, const StatusReportMode mode);
  void clearStatusReport();

public Q_SLOTS:

  void labelSelected(QListWidgetItem* current, QListWidgetItem* previous = NULL);
  void record();
  void learn();
  void simulate();

  bool getDMPName(std::string& dmp_name);

private:
  ros::NodeHandle node_handle_;

  std::map<QListWidget*, gui_utilities::DescriptionList> widget_list_map_;
  std::map<QListWidget*, gui_utilities::DescriptionList>::iterator widget_list_map_iterator_;

  std::vector<std::string> description_names_;

  task_recorder2_msgs::Description description_;
  std::vector<task_recorder2_msgs::DataSample> data_samples_;

  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client_;

  bool recording_joint_states_;
  double num_samples_per_second_;
  ros::Time start_time_;

  std::string recorder_data_directory_name_;
  std::string dmp_data_directory_name_;
  bool readParams();

  // void loadDemonstrations();
  void load(const std::string directory_name, QListWidget *list);

  boost::shared_ptr<inverse_kinematics::InverseKinematicsWithNullspaceOptimization> left_arm_ik_;
  boost::shared_ptr<inverse_kinematics::InverseKinematicsWithNullspaceOptimization> right_arm_ik_;

  skill_library::SkillLibraryClient skill_library_client_;

  void simulate(dmp_lib::Trajectory& endeffector, dmp_lib::Trajectory& restposture,
                boost::shared_ptr<inverse_kinematics::InverseKinematicsWithNullspaceOptimization> ik);

};

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_ */
