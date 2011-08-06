/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		pr2_dynamic_movement_primitive_gui.h

  \author	Peter Pastor
  \date		Jul 28, 2011

 *********************************************************************/

#ifndef PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_
#define PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <map>

#include <dynamic_movement_primitive_gui/dynamic_movement_primitive_gui.h>
#include <skill_library/skill_library_client.h>
#include <gui_utilities/description_list.h>

#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_base_client.h>
#include <dynamic_movement_primitive_utilities/dynamic_movement_primitive_controller_client.h>

// local includes
#include <pr2_dynamic_movement_primitive_gui/pr2_dynamic_movement_primitive_gui_main_window.h>
#include <pr2_dynamic_movement_primitive_gui/pr2_controller_manager_client.h>

namespace pr2_dynamic_movement_primitive_gui
{

class PR2DynamicMovementPrimitiveGUI : public QMainWindow, private Ui::pr2_dynamic_movement_primitive_gui
{
  Q_OBJECT

public:

  /*! Constructor
   */
  PR2DynamicMovementPrimitiveGUI(ros::NodeHandle node_handle, QWidget* parent = 0, Qt::WFlags flags = Qt::Window);
  /*! Destructor
   */
  virtual ~PR2DynamicMovementPrimitiveGUI() {};

public Q_SLOTS:

  void labelSelected(QListWidgetItem* current, QListWidgetItem* previous = NULL);
  void listControllers();
  void startController();
  void stopController();
  void reloadControllers();
  void execute();

private:

  ros::NodeHandle node_handle_;

  bool readParams();

  /*!
   */
  dynamic_movement_primitive_gui::DynamicMovementPrimitiveGUI dynamic_movement_primitive_gui_;

  /*!
   */
  skill_library::SkillLibraryClient skill_library_client_;

  /*!
   */
  ros::Publisher right_arm_dmp_publisher_;
  ros::Publisher left_arm_dmp_publisher_;
  ros::Publisher dual_arm_dmp_publisher_;
  std::string right_arm_dmp_ik_controller_name_;
  std::string left_arm_dmp_ik_controller_name_;
  std::string dual_arm_dmp_ik_controller_name_;

  /*!
   */
  PR2ControllerManagerClient pr2_controller_manager_client_;

  std::map<QListWidget*, gui_utilities::DescriptionList> widget_list_map_;
  std::map<QListWidget*, gui_utilities::DescriptionList>::iterator widget_list_map_iterator_;

  bool getControllerName(QListWidget* list, std::string& controller_name);

  /*!
   */
  dmp_utilities::DynamicMovementPrimitiveControllerClient right_arm_dmp_controller_client_;
  dmp_utilities::DynamicMovementPrimitiveControllerClient left_arm_dmp_controller_client_;
  dmp_utilities::DynamicMovementPrimitiveControllerClient dual_arm_dmp_controller_client_;

  bool sendDMP(const dmp_lib::DMPPtr& dmp, bool wait_for_success = false);

};

}

#endif /* PR2_DYNAMIC_MOVEMENT_PRIMITIVE_GUI_H_ */
