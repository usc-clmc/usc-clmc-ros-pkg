/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		task_monitor_gui.h

  \author	Peter Pastor
  \date		Jun 18, 2011

 *********************************************************************/

#ifndef TASK_MONITOR_GUI_H_
#define TASK_MONITOR_GUI_H_

// system includes
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

#include <QProcess>

#include <task_recorder2/task_labeler.h>
#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/Notification.h>

#include <task_event_detector/detector.h>
#include <task_event_detector/task_event_detector_client.h>

#include <gui_utilities/tab_map.h>

#include <task_label_gui/label_gui_client.h>

// local includes
#include <task_monitor_gui/task_monitor_gui_main_window.h>
#include <task_monitor_gui/modelselection_grid_search_kernel_gui.h>

Q_DECLARE_METATYPE(task_recorder2_msgs::Description::Ptr);

namespace task_monitor_gui
{

class TaskMonitorGui : public QMainWindow, private Ui::task_monitor_gui
{
Q_OBJECT

public:

  /*! Constructor
   * @param node_handle
   * @param parent
   * @param flags
   */
  TaskMonitorGui(ros::NodeHandle node_handle,
                 QWidget* parent = 0,
                 Qt::WFlags flags = 0);
  /*! Destructor
   */
  virtual ~TaskMonitorGui() {};

private:
  /*!
   */
  enum StatusReportMode
  {
    DEBUG = 0,//!< DEBUG
    INFO,     //!< INFO
    WARN,     //!< WARN
    ERROR,    //!< ERROR
    FATAL     //!< FATAL
  };

public Q_SLOTS:

  void labelSelected(QListWidgetItem* current, QListWidgetItem* previous = NULL);
  void setLabel();

  void add();
  void addAll();
  void labelLastSample();

  void clearSVM();
  void train();
  void crossvalidate();
  void doneCrossvalidating(const task_recorder2_msgs::Description& svm_description,
                           const task_event_detector::SVMParametersMsg& parameters);
  void save();
  void load(const QString&);

  void stream();
  void record();
  void recordSample();
  void interrupt();

  void check();
  void detect(int detect);
  void setClassificationBoundary(double value);

  void loadDefault();

  void dataSampleDescriptionChanged(const QString& description_qstring);
  bool loadDataSampleDescriptions(const bool search = false, const QString& description_qstring = "null");

  void setDescriptionTextFromDefault(const QString& description_qstring);
  void setDescriptionTextFromDataSampleBox(const QString& description_qstring);
  void setIdFromSpinBox(int id);

  void clearStatusReport();
  void setStatusReport(const std::string& status_report,
                       const StatusReportMode mode);

  void insertDescription(QListWidget* list_widget,
                         const task_recorder2_msgs::Description& description);
  void insertDescriptions(QListWidget* list_widget,
                          const std::vector<task_recorder2_msgs::Description>& descriptions);
  void getAllDescriptions(QListWidget* list_widget,
                          std::vector<task_recorder2_msgs::Description>& descriptions);
  void getSelectedDescriptions(QListWidget* list_widget,
                               std::vector<task_recorder2_msgs::Description>& descriptions);
  void removeSelectedItems(QListWidget* list_widget);
  void focusDataSampleDescription();

  void openBagFile();
  void extract();

Q_SIGNALS:

  void clearListSignal();

  void dataSampleDescriptionChangedSignal(const QString& description_qstring);
  bool loadDataSampleDescriptionsSignal(const bool search = false,
                                        const QString& description_qstring = "null");

  void insertDescriptionSignal(QListWidget* list_widget,
                               const task_recorder2_msgs::Description& description);
  void insertDescriptionsSignal(QListWidget* list_widget,
                                const std::vector<task_recorder2_msgs::Description>& descriptions);
  void getAllDescriptionsSignal(QListWidget* list_widget,
                                std::vector<task_recorder2_msgs::Description>& descriptions);
  void getSelectedDescriptionsSignal(QListWidget* list_widget,
                                     std::vector<task_recorder2_msgs::Description>& descriptions);
  void removeSelectedItemsSignal(QListWidget* list_widget);
  void focusDataSampleDescriptionSignal();

private:

  ros::NodeHandle node_handle_;

  ros::Subscriber stop_recording_notification_subscriber_;
  void notificationCB(const task_recorder2_msgs::NotificationConstPtr& notification);

  task_recorder2::TaskLabeler<task_recorder2_msgs::DataSampleLabel> task_labeler_;
  task_label_gui::LabelGuiClient label_gui_client_;
  gui_utilities::TabMap tab_map_;

//  std::map<QListWidget*, boost::shared_ptr<gui_utilities::DescriptionList> > widget_list_map_;
//  std::map<QListWidget*, boost::shared_ptr<gui_utilities::DescriptionList> >::iterator widget_list_map_iterator_;

  enum DescriptionRoles
  {
    DescriptionRole = Qt::UserRole + 1
  };

  task_event_detector::TaskEventDetectorClientPtr task_event_detector_client_;

  bool loadSVMDescriptions(const bool search = false, const QString& description_qstring = "null");

  bool getSVMName(task_recorder2_msgs::Description& svm_description);

  task_recorder2_msgs::Description recorded_description_;

  std::vector<task_recorder2_msgs::Description> added_descriptions_;
  bool readIntoSVM(const std::vector<task_recorder2_msgs::Description>& descriptions);
  void addToSVM(const std::vector<task_recorder2_msgs::Description>& descriptions);

  /*! utility functions
   */
  task_recorder2_msgs::DataSampleLabel getLabel();
  void select(QListWidgetItem* item);
  QWidget* getLabelTab(const int& label_type);

  std::vector<std::string> description_names_;

  bool streaming_;
  bool recording_;
  ros::Time start_time_;
  ros::Time end_time_;

  double task_recorder_manager_sampling_rate_;
  double recording_duration_;

  typedef std::pair<std::string, int> MapPair;
  std::map<std::string, int> kernel_type_map_;
  std::map<std::string, int> svm_lib_map_;

  void setup();
  bool loadSVMInfo();
  bool getSVMInfo(task_event_detector::SVMParametersMsg& msg);
  bool setSVMInfo();
  void setSVMInfo(ros::NodeHandle node_handle);
  bool setLibAndKernel(const int svm_lib, const int kernel_type);
  bool getLibAndKernel(int& svm_lib, int& kernel_type);
  QString current_data_sample_description_;
  bool setDetectionVariableNames(const std::vector<std::string>& variable_names);
  void getDetectionVariableNames(std::vector<std::string>& detection_variable_names);

  ModelSelectionGridSearchKernelGuiPtr model_selection_gui_;
  std::vector<boost::shared_ptr<QProcess> > rxbag_processes_;

};

}

#endif /* TASK_MONITOR_GUI_H_ */
