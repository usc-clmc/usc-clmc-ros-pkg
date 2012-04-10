/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		label_gui.h

  \author	Peter Pastor
  \date		Jun 18, 2011

 *********************************************************************/

#ifndef LABEL_GUI_H_
#define LABEL_GUI_H_

// system includes
#include <QtGui>
#include <QCloseEvent>

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>

#include <task_recorder2/task_labeler.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/SetLabel.h>
#include <task_recorder2_msgs/SetLastTrialIds.h>

#include <gui_utilities/description_list.h>
#include <gui_utilities/tab_map.h>

#include <task_recorder2/task_recorder_manager_client.h>

// local includes
#include <task_label_gui/label_gui_main_window.h>

namespace task_label_gui
{

class LabelGui : public QMainWindow, private Ui::label_gui
{
Q_OBJECT

public:

  /*! Constructor
   * @param node_handle
   * @param parent
   * @param flags
   */
  LabelGui(ros::NodeHandle node_handle,
           QWidget* parent = 0,
           Qt::WFlags flags = Qt::Window /*Qt::WindowStaysOnTopHint & Qt::Dialog*/);
  /*! Destructor
   */
  virtual ~LabelGui() {};

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool setLabel(task_recorder2_msgs::SetLabel::Request& request,
                task_recorder2_msgs::SetLabel::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool setLastTrialIds(task_recorder2_msgs::SetLastTrialIds::Request& request,
                       task_recorder2_msgs::SetLastTrialIds::Response& response);

  /*!
   * @param request
   * @param response
   * @return True on success, otherwise False
   */
  bool recordAndLabel(task_recorder2_msgs::SetLabel::Request& request,
                      task_recorder2_msgs::SetLabel::Response& response);

public Q_SLOTS:

  void closeEvent(QCloseEvent *event);
  void setLabel();
  void cancel();
  void showLabelGui(bool visible);

  void insertDescription(QListWidget* list_widget,
                         const task_recorder2_msgs::Description& description);
  void insertDescriptions(QListWidget* list_widget,
                          const std::vector<task_recorder2_msgs::Description>& descriptions);

Q_SIGNALS:
  void setLabelGuiVisibility(bool visible);
  void insertDescriptionSignal(QListWidget* list_widget,
                               const task_recorder2_msgs::Description& description);
  void insertDescriptionsSignal(QListWidget* list_widget,
                                const std::vector<task_recorder2_msgs::Description>& descriptions);

private:

  ros::NodeHandle node_handle_;
  task_recorder2::TaskLabeler<task_recorder2_msgs::DataSampleLabel> task_labeler_;

  // gui_utilities::DescriptionListPtr description_list_;
  gui_utilities::TabMap tab_map_;

  ros::ServiceServer set_last_trial_ids_service_server_;
  ros::ServiceServer set_label_service_server_;
  ros::ServiceServer record_and_label_service_server_;
  boost::mutex wait_for_label_mutex_;

  task_recorder2_msgs::DataSampleLabel getLabel(const int label_type = task_recorder2_msgs::DataSampleLabel::BINARY_LABEL);

  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client_;

  void enableTab(const int label_type);
  void done();
};

}

#endif /* TASK_MONITOR_GUI_H_ */
