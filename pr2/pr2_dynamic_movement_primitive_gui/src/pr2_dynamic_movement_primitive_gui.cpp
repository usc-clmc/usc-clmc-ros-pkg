/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_dynamic_movement_primitive_gui.cpp

  \author	Peter Pastor
  \date		Jul 28, 2011

 *********************************************************************/

// system includes

// local includes
#include <pr2_dynamic_movement_primitive_gui/pr2_dynamic_movement_primitive_gui.h>
#include <pr2_dynamic_movement_primitive_gui/pr2_dynamic_movement_primitive_gui_main_window.h>

using namespace dynamic_movement_primitive_gui;

namespace pr2_dynamic_movement_primitive_gui
{

static const std::string RUNNING_CONTROLLER_LIST_NAME = std::string("RunningControllerList");
static const std::string STOPPED_CONTROLLER_LIST_NAME = std::string("StoppedControllerList");

PR2DynamicMovementPrimitiveGUI::PR2DynamicMovementPrimitiveGUI(ros::NodeHandle node_handle,
                                                               QWidget* parent,
                                                               Qt::WFlags flags) :
                                                               node_handle_(node_handle), dynamic_movement_primitive_gui_(node_handle), skill_library_client_(node_handle), pr2_controller_manager_client_(node_handle)
{
  ROS_VERIFY(readParams());

  // this sets up PR2 GUI
  Ui_pr2_dynamic_movement_primitive_gui::setupUi(this);

  dynamic_movement_primitive_gui_.show();

  // signals/slots mechanism in action (using DMP gui)
  connect(dynamic_movement_primitive_gui_.execute_button_, SIGNAL(clicked()), this, SLOT(execute()));

  connect(running_controller_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*)));
  connect(stopped_controller_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*)));

  connect(update_button_, SIGNAL(clicked()), this, SLOT(listControllers()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(startController()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopController()));
  connect(reload_button_, SIGNAL(clicked()), this, SLOT(reloadControllers()));

	gui_utilities::DescriptionList running_controller_list(running_controller_list_, RUNNING_CONTROLLER_LIST_NAME, false);
  widget_list_map_.insert(DynamicMovementPrimitiveGUI::WidgetDescriptionListPair(running_controller_list_, running_controller_list));

  gui_utilities::DescriptionList stopped_controller_list(stopped_controller_list_, STOPPED_CONTROLLER_LIST_NAME, false);
  widget_list_map_.insert(DynamicMovementPrimitiveGUI::WidgetDescriptionListPair(stopped_controller_list_, stopped_controller_list));

  right_arm_dmp_publisher_ = node_handle_.advertise<dmp::ICRA2009DynamicMovementPrimitive::DMPMsg>(right_arm_dmp_ik_controller_name_, 10);
  left_arm_dmp_publisher_ = node_handle_.advertise<dmp::ICRA2009DynamicMovementPrimitive::DMPMsg>(left_arm_dmp_ik_controller_name_, 10);
  dual_arm_dmp_publisher_ = node_handle_.advertise<dmp::ICRA2009DynamicMovementPrimitive::DMPMsg>(dual_arm_dmp_ik_controller_name_, 10);

  ROS_VERIFY(right_arm_dmp_controller_client_.initialize(right_arm_dmp_ik_controller_name_));
  right_arm_dmp_controller_client_.setSingleThreadedMode(false);
  ROS_VERIFY(left_arm_dmp_controller_client_.initialize(left_arm_dmp_ik_controller_name_));
  left_arm_dmp_controller_client_.setSingleThreadedMode(false);
  ROS_VERIFY(dual_arm_dmp_controller_client_.initialize(dual_arm_dmp_ik_controller_name_));
  dual_arm_dmp_controller_client_.setSingleThreadedMode(false);

  listControllers();
}

bool PR2DynamicMovementPrimitiveGUI::sendDMP(const dmp_lib::DMPPtr& dmp, bool wait_for_success)
{
  std::vector<std::string> variable_names = dmp->getVariableNames();
  bool right_arm = robot_info::RobotInfo::containsRightArm(variable_names);
  bool left_arm = robot_info::RobotInfo::containsLeftArm(variable_names);
  if(right_arm && left_arm)
  {
    if(!dual_arm_dmp_controller_client_.sendCommand(dmp, dual_arm_dmp_ik_controller_name_, wait_for_success))
    {
      std::string info("Problem when sending DMP on controller " + dual_arm_dmp_ik_controller_name_ + ".");
      dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::ERROR);
      return false;
    }
  }
  else if(right_arm)
  {
    if(!right_arm_dmp_controller_client_.sendCommand(dmp, right_arm_dmp_ik_controller_name_, wait_for_success))
    {
      std::string info("Problem when sending DMP on controller " + right_arm_dmp_ik_controller_name_ + ".");
      dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::ERROR);
      return false;
    }
  }
  else if(left_arm)
  {
    if(!left_arm_dmp_controller_client_.sendCommand(dmp, left_arm_dmp_ik_controller_name_, wait_for_success))
    {
      std::string info("Problem when sending DMP on controller " + left_arm_dmp_ik_controller_name_ + ".");
      dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::ERROR);
      return false;
    }
  }
  else
  {
    std::string info("DMP does not contain variables for the arms.");
    dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::ERROR);
    return false;
  }
  return true;
}

void PR2DynamicMovementPrimitiveGUI::labelSelected(QListWidgetItem* current, QListWidgetItem* previous)
{
  start_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  if(widget_list_map_.at(current->listWidget()).getName().compare(RUNNING_CONTROLLER_LIST_NAME) == 0)
  {
    stop_button_->setEnabled(true);
  }
  else if(widget_list_map_.at(current->listWidget()).getName().compare(STOPPED_CONTROLLER_LIST_NAME) == 0)
  {
    start_button_->setEnabled(true);
  }
  else
  {
    ROS_ASSERT_MSG(false, "Unknown list name >%s<. This should never happen.",
                   widget_list_map_.at(current->listWidget()).getName().c_str());
  }
}

bool PR2DynamicMovementPrimitiveGUI::getControllerName(QListWidget* list, std::string& controller_name)
{
  std::vector<task_recorder2_msgs::Description> descriptions;
  if(!widget_list_map_.at(list).getDescriptions(descriptions))
  {
    dynamic_movement_primitive_gui_.setStatusReport("No controller selected... This should never happen.", DynamicMovementPrimitiveGUI::ERROR);
    return false;
  }
  ROS_ASSERT_MSG(descriptions.size() == 1, "There should only be 1 controller selected. This should never happen.");
  controller_name = task_recorder2_utilities::getDescription(descriptions[0]);
  return true;
}

void PR2DynamicMovementPrimitiveGUI::stopController()
{
  std::string controller_name;
  ROS_VERIFY(getControllerName(running_controller_list_, controller_name));

  std::string info("Stopping controller " + controller_name + ".");
  dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::INFO);

  ROS_VERIFY(pr2_controller_manager_client_.stopController(controller_name));

  listControllers();
}

void PR2DynamicMovementPrimitiveGUI::startController()
{
  std::string controller_name;
  ROS_VERIFY(getControllerName(stopped_controller_list_, controller_name));

  std::string info("Starting controller " + controller_name + ".");
  dynamic_movement_primitive_gui_.setStatusReport(info, DynamicMovementPrimitiveGUI::INFO);

  ROS_VERIFY(pr2_controller_manager_client_.startController(controller_name));

  listControllers();
}

void PR2DynamicMovementPrimitiveGUI::listControllers()
{
  std::vector<std::string> controller_names;
  std::vector<PR2ControllerManagerClient::ControllerState> controller_states;
  ROS_VERIFY(pr2_controller_manager_client_.getControllerList(controller_names, controller_states));
  if(!widget_list_map_.at(running_controller_list_).isEmpty())
  {
    widget_list_map_.at(running_controller_list_).clearList();
  }
  if(!widget_list_map_.at(stopped_controller_list_).isEmpty())
  {
    widget_list_map_.at(stopped_controller_list_).clearList();
  }

  for (int i = 0; i < (int)controller_names.size(); ++i)
  {
    task_recorder2_msgs::Description description;
    description.description = controller_names[i];
    if(controller_states[i] == PR2ControllerManagerClient::RUNNING)
    {
      widget_list_map_.at(running_controller_list_).insert(description);
    }
    else if(controller_states[i] == PR2ControllerManagerClient::STOPPED)
    {
      widget_list_map_.at(stopped_controller_list_).insert(description);
    }
    else
    {
      ROS_ASSERT_MSG(false, "Invalid controller status.");
    }
  }
}

void PR2DynamicMovementPrimitiveGUI::reloadControllers()
{
  ROS_VERIFY(pr2_controller_manager_client_.reloadControllers());
  listControllers();
}

void PR2DynamicMovementPrimitiveGUI::execute()
{
  std::string dmp_name;
  if(!dynamic_movement_primitive_gui_.getDMPName(dmp_name))
  {
    return;
  }
  ROS_INFO("Executing DMP with name >%s<", dmp_name.c_str());

  dmp_lib::DMPPtr dmp;
  ROS_VERIFY(skill_library_client_.get(dmp_name, dmp));
  ROS_VERIFY(dmp->setup());

  if(!sendDMP(dmp))
  {
    dynamic_movement_primitive_gui_.setStatusReport("Error when sending DMP.", DynamicMovementPrimitiveGUI::ERROR);
    return;
  }
  dynamic_movement_primitive_gui_.setStatusReport("Send DMP.", DynamicMovementPrimitiveGUI::INFO);
}

bool PR2DynamicMovementPrimitiveGUI::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "right_arm_dmp_ik_controller_name", right_arm_dmp_ik_controller_name_));
  usc_utilities::appendLeadingSlash(right_arm_dmp_ik_controller_name_);
  ROS_VERIFY(usc_utilities::read(node_handle_, "left_arm_dmp_ik_controller_name", left_arm_dmp_ik_controller_name_));
  usc_utilities::appendLeadingSlash(left_arm_dmp_ik_controller_name_);
  ROS_VERIFY(usc_utilities::read(node_handle_, "dual_arm_dmp_ik_controller_name", dual_arm_dmp_ik_controller_name_));
  usc_utilities::appendLeadingSlash(dual_arm_dmp_ik_controller_name_);
  return true;
}

}
