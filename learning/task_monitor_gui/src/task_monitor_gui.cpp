/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks      ...
 
  \file     task_monitor_gui.cpp

  \author   Peter Pastor
  \date     Jun 18, 2011

 *********************************************************************/

// system includes
#include <QtGui>
#include <iomanip>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>
#include <task_recorder2_utilities/task_description_utilities.h>

#include <task_recorder2_msgs/DetectedEvents.h>
#include <task_event_detector/SVMParametersMsg.h>

// local includes
#include <task_monitor_gui/task_monitor_gui.h>

namespace task_monitor_gui
{

static const std::string LABELED_LIST_NAME = std::string("LabeledList");
static const std::string UNLABELED_LIST_NAME = std::string("UnLabeledList");

static const QString COLOR_STYLE("* { color : %1; }");
static const QColor COLOR_RED(255, 0, 0, 255);
static const QColor COLOR_BLACK(0, 0, 0, 255);

TaskMonitorGui::TaskMonitorGui(ros::NodeHandle node_handle,
                               QWidget* parent,
                               Qt::WFlags flags) :
  QMainWindow(parent, flags), node_handle_(node_handle), task_labeler_(node_handle), streaming_(false), recording_(false)
{
  task_event_detector_client_.reset(new task_event_detector::TaskEventDetectorClient);
  stop_recording_notification_subscriber_ = node_handle_.subscribe<task_recorder2_msgs::Notification> (std::string("/TaskRecorderManager/notification"), 10,
                                                                                                       &TaskMonitorGui::notificationCB, this);
  // this sets up GUI
  setupUi(this);
  setup();

  model_selection_gui_.reset(new ModelSelectionGridSearchKernelGui(node_handle, parent));
  // model_selection_gui_->show();

  // signals/slots mechanism in action
  ROS_VERIFY(connect(labeled_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*))));
  ROS_VERIFY(connect(unlabeled_list_, SIGNAL(itemPressed(QListWidgetItem*)), this, SLOT(labelSelected(QListWidgetItem*))));
  ROS_VERIFY(connect(label_button_, SIGNAL(clicked()), this, SLOT(setLabel())));
  ROS_VERIFY(connect(add_button_, SIGNAL(clicked()), this, SLOT(add())));
  ROS_VERIFY(connect(add_all_button_, SIGNAL(clicked()), this, SLOT(addAll())));
  ROS_VERIFY(connect(label_last_button_, SIGNAL(clicked()), this, SLOT(labelLastSample())));
  ROS_VERIFY(connect(train_button_, SIGNAL(clicked()), this, SLOT(train())));
  ROS_VERIFY(connect(crossvalidate_svm_button_, SIGNAL(clicked()), this, SLOT(crossvalidate())));
  ROS_VERIFY(connect(save_button_, SIGNAL(clicked()), this, SLOT(save())));
  ROS_VERIFY(connect(svm_box_, SIGNAL(activated(const QString&)), this, SLOT(load(const QString&))));
  ROS_VERIFY(connect(record_button_, SIGNAL(clicked()), this, SLOT(record())));
  ROS_VERIFY(connect(record_sample_button_, SIGNAL(clicked()), this, SLOT(recordSample())));
  ROS_VERIFY(connect(stream_button_, SIGNAL(clicked()), this, SLOT(stream())));
  ROS_VERIFY(connect(interrupt_button_, SIGNAL(clicked()), this, SLOT(interrupt())));
  ROS_VERIFY(connect(check_button_, SIGNAL(clicked()), this, SLOT(check())));
  ROS_VERIFY(connect(svm_boundary_spin_box_, SIGNAL(valueChanged(double)), this, SLOT(setClassificationBoundary(double))));
  ROS_VERIFY(connect(detect_box_, SIGNAL(stateChanged(int)), this, SLOT(detect(int))));
  ROS_VERIFY(connect(description_cbox_, SIGNAL(activated(const QString&)), this, SLOT(setDescriptionTextFromDefault(const QString&))));
  ROS_VERIFY(connect(id_spin_box_, SIGNAL(valueChanged(int)), this, SLOT(setIdFromSpinBox(int))));
  ROS_VERIFY(connect(data_sample_description_box_, SIGNAL(activated(const QString&)), this, SLOT(dataSampleDescriptionChanged(const QString&))));
  ROS_VERIFY(connect(data_sample_description_box_, SIGNAL(activated(const QString&)), this, SLOT(setDescriptionTextFromDataSampleBox(const QString&))));
  ROS_VERIFY(connect(default_button_, SIGNAL(clicked()), this, SLOT(loadDefault())));
  ROS_VERIFY(connect(clear_svm_button_, SIGNAL(clicked()), this, SLOT(clearSVM())));

  ROS_VERIFY(connect(extract_button_, SIGNAL(clicked()), this, SLOT(extract())));
  ROS_VERIFY(connect(open_button_, SIGNAL(clicked()), this, SLOT(openBagFile())));

  ROS_VERIFY(connect(this, SIGNAL(dataSampleDescriptionChangedSignal(const QString&)),
                     this, SLOT(dataSampleDescriptionChanged(const QString&))));
  ROS_VERIFY(connect(this, SIGNAL(loadDataSampleDescriptionsSignal(const bool, const QString&)),
                     this, SLOT(loadDataSampleDescriptions(const bool, const QString&))));

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
  ROS_VERIFY(connect(this, SIGNAL(focusDataSampleDescriptionSignal()),
                     this, SLOT(focusDataSampleDescription())));

  default_button_->click();
  ROS_VERIFY(loadSVMDescriptions());
  // ROS_VERIFY(loadDataSampleDescriptions());

  svm_box_->setDuplicatesEnabled(false);
  if(svm_box_->count() > 0)
  {
    load(svm_box_->currentText());
  }

  ROS_VERIFY(task_recorder2_utilities::readDescriptionLabels(node_handle, description_names_));
  description_cbox_->setDuplicatesEnabled(false);
  for (int i = 0; i < (int)description_names_.size(); ++i)
  {
    description_cbox_->addItem(QString(description_names_[i].c_str()));
    description_cbox_->setEnabled(true);
  }
  if(description_cbox_->count() > 0)
  {
    setDescriptionTextFromDefault(description_cbox_->currentText());
  }

  duration_spin_box_->setMaximum(15.0);
  duration_spin_box_->setMinimum(1.0);

  if(data_sample_description_box_->count() > 0)
  {
    dataSampleDescriptionChangedSignal(data_sample_description_box_->currentText());
  }

  ros::NodeHandle task_recorder_manager_node_handle("/TaskRecorderManager");
  ROS_VERIFY(usc_utilities::read(task_recorder_manager_node_handle, "sampling_rate", task_recorder_manager_sampling_rate_));
}

void TaskMonitorGui::setup()
{
  unlabeled_list_->setObjectName(QString(UNLABELED_LIST_NAME.c_str()));
  labeled_list_->setObjectName(QString(LABELED_LIST_NAME.c_str()));

  kernel_type_map_.insert(MapPair("K_GAUSSIAN", task_event_detector::SVMParametersMsg::K_GAUSSIAN));
  kernel_type_map_.insert(MapPair("K_LINEAR", task_event_detector::SVMParametersMsg::K_LINEAR));
  std::map<std::string, int>::const_iterator mi;
  for (mi = kernel_type_map_.begin(); mi != kernel_type_map_.end(); ++mi)
  {
    kernel_type_box_->addItem(QString(mi->first.c_str()));
  }
  svm_lib_map_.insert(MapPair("CT_LIGHT", task_event_detector::SVMParametersMsg::CT_LIGHT));
  svm_lib_map_.insert(MapPair("CT_LIBSVM", task_event_detector::SVMParametersMsg::CT_LIBSVM));
  for (mi = svm_lib_map_.begin(); mi != svm_lib_map_.end(); ++mi)
  {
    svm_lib_box_->addItem(QString(mi->first.c_str()));
  }

  std::vector<task_recorder2_msgs::TaskRecorderSpecification> specifications;
  ROS_VERIFY(task_recorder2_utilities::readTaskRecorderSpecification(specifications));
  std::vector<std::string> all_variable_names;
  ROS_VERIFY(task_recorder2_utilities::getAllVariableNames(specifications, all_variable_names));
  task_recorder2_msgs::DataSample data_sample;
  data_sample.names = all_variable_names;
  data_sample.data.resize(all_variable_names.size(), 0.0);
  ROS_VERIFY(task_event_detector_client_->filter(data_sample));
  for (int i = 0; i < (int)data_sample.names.size(); ++i)
  {
    detection_variable_name_list_->addItem(QString(data_sample.names[i].c_str()));
    detection_variable_name_list_->setEnabled(true);
  }
  std::vector<std::string> default_detection_variable_names;
  ROS_VERIFY(usc_utilities::read(node_handle_, "default_detection_variable_names", default_detection_variable_names));
  ROS_VERIFY(setDetectionVariableNames(default_detection_variable_names));
}

void TaskMonitorGui::getDetectionVariableNames(std::vector<std::string>& detection_variable_names)
{
  QList<QListWidgetItem*> items = detection_variable_name_list_->selectedItems();
  detection_variable_names.clear();
  QList<QListWidgetItem*>::iterator ci;
  for(ci = items.begin(); ci != items.end(); ++ci)
  {
    detection_variable_names.push_back((*ci)->text().toStdString());
  }
}

bool TaskMonitorGui::setDetectionVariableNames(const std::vector<std::string>& variable_names)
{
  detection_variable_name_list_->clearSelection();
  for (int i = 0; i < (int)variable_names.size(); ++i)
  {
    QList<QListWidgetItem*> items = detection_variable_name_list_->findItems(QString(variable_names[i].c_str()), Qt::MatchExactly);
    ROS_VERIFY_MSG(items.count() == 1, "Could not find variable named >%s<.", variable_names[i].c_str());
    detection_variable_name_list_->setCurrentItem(items.front(), QItemSelectionModel::Select);
  }
  return true;
}

void TaskMonitorGui::setDescriptionTextFromDefault(const QString& description_qstring)
{
  if(!description_qstring.isEmpty())
  {
    description_line_->setText(description_qstring);
    setIdFromSpinBox(id_spin_box_->value());
  }
}

void TaskMonitorGui::setDescriptionTextFromDataSampleBox(const QString& description_qstring)
{
  if(!description_qstring.isEmpty())
  {
    std::string description;
    int id = 0;
    ROS_VERIFY(task_recorder2_utilities::parseDescriptionString(description_qstring.toStdString(), description, id));
    description_line_->setText(QString(description.c_str()));
    id_spin_box_->setValue(id);
    loadDataSampleDescriptionsSignal(true, description_qstring);
  }
}

void TaskMonitorGui::setIdFromSpinBox(int id)
{
  task_recorder2_msgs::Description description;
  description.description = description_line_->text().toStdString();
  description.id = id;
  loadDataSampleDescriptionsSignal(true, QString(task_recorder2_utilities::getFileName(description).c_str()));
}

bool TaskMonitorGui::loadDataSampleDescriptions(const bool search, const QString& description_qstring)
{
  current_data_sample_description_ = data_sample_description_box_->currentText();
  std::vector<std::string> descriptions;
  task_event_detector_client_->getListOfDataSamples(descriptions);
  data_sample_description_box_->clear();
  data_sample_description_box_->setInsertPolicy(QComboBox::InsertAlphabetically);
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    std::string description;
    int id = 0;
    ROS_VERIFY(task_recorder2_utilities::parseDescriptionString(descriptions[i], description, id));
    data_sample_description_box_->addItem(QString(descriptions[i].c_str()));
  }
  if (!descriptions.empty())
  {
    data_sample_description_box_->setEnabled(true);
    int index = data_sample_description_box_->findText(current_data_sample_description_);
    if (index >= 0)
    {
      data_sample_description_box_->setCurrentIndex(index);
      dataSampleDescriptionChangedSignal(current_data_sample_description_);
    }
    if(search)
    {
      index = data_sample_description_box_->findText(description_qstring);
      if (index >= 0)
      {
        data_sample_description_box_->setCurrentIndex(index);
        dataSampleDescriptionChangedSignal(description_qstring);
      }
    }
  }
  return true;
}

void TaskMonitorGui::insertDescription(QListWidget* list_widget,
                                       const task_recorder2_msgs::Description& description)
{
  task_recorder2_msgs::Description::Ptr list_description(new task_recorder2_msgs::Description(description));
  std::string list_item_text = task_recorder2_utilities::getFileName(description);
  list_item_text.append(std::string(" trial:") + usc_utilities::getString(description.trial));
  QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(list_item_text));
  item->setData(DescriptionRole, QVariant::fromValue(list_description));
  list_widget->addItem(item);
}

void TaskMonitorGui::insertDescriptions(QListWidget* list_widget,
                                        const std::vector<task_recorder2_msgs::Description>& descriptions)
{
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    insertDescriptionSignal(list_widget, descriptions[i]);
  }
}

void TaskMonitorGui::removeSelectedItems(QListWidget* list_widget)
{
  QList<QListWidgetItem*> items = list_widget->selectedItems ();
  for (int i = 0; i < (int)items.size(); ++i)
  {
     delete items[i];
  }
}

void TaskMonitorGui::getAllDescriptions(QListWidget* list_widget,
                                        std::vector<task_recorder2_msgs::Description>& descriptions)
{
  descriptions.clear();
  for (int i = 0; i < list_widget->count(); ++i)
  {
    task_recorder2_msgs::Description::Ptr description_ptr = list_widget->item(i)->data(DescriptionRole).value<task_recorder2_msgs::Description::Ptr>();
    descriptions.push_back(*description_ptr);
  }
}

void TaskMonitorGui::getSelectedDescriptions(QListWidget* list_widget,
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

void TaskMonitorGui::dataSampleDescriptionChanged(const QString& description_qstring)
{
  std::vector<task_recorder2_msgs::Description> labeled_descriptions;
  std::vector<task_recorder2_msgs::Description> unlabeled_descriptions;
  ROS_VERIFY(task_event_detector_client_->getListOfRecordedDataSamples(description_qstring.toStdString(), labeled_descriptions, unlabeled_descriptions));

  if(unlabeled_list_->count() > 0)
  {
    unlabeled_list_->clear();
  }
  insertDescriptionsSignal(unlabeled_list_, unlabeled_descriptions);

  if (labeled_list_->count() > 0)
  {
    labeled_list_->clear();
  }
  insertDescriptionsSignal(labeled_list_, labeled_descriptions);

  if(description_qstring.compare(current_data_sample_description_) != 0)
  {
    loadDataSampleDescriptionsSignal(true, current_data_sample_description_);
  }
}

void TaskMonitorGui::notificationCB(const task_recorder2_msgs::NotificationConstPtr& notification)
{
  insertDescriptionSignal(unlabeled_list_, notification->description);
  loadDataSampleDescriptionsSignal();
  std::string description_string = data_sample_description_box_->currentText().toStdString();
  int id;
  std::string description;
  if(!description_string.empty())
  {
    ROS_VERIFY(task_recorder2_utilities::parseDescriptionString(description_string, description, id));
    if ((notification->description.description.compare(description) != 0) || (notification->description.id != id))
    {
      dataSampleDescriptionChangedSignal(QString(task_recorder2_utilities::getFileName(notification->description).c_str()));
      data_sample_description_box_->setCurrentIndex(data_sample_description_box_->findText(QString(task_recorder2_utilities::getFileName(notification->description).c_str())));
    }
  }
}

void TaskMonitorGui::labelSelected(QListWidgetItem* current, QListWidgetItem* previous)
{
  if(current->listWidget() == labeled_list_)
  {
    add_button_->setEnabled(true);
    add_all_button_->setEnabled(true);
  }
  else if(current->listWidget() == unlabeled_list_)
  {
    std::vector<task_recorder2_msgs::Description> descriptions;
    getSelectedDescriptions(unlabeled_list_, descriptions);
    if(!task_labeler_.getLabelType(descriptions))
    {
      setStatusReport("Selected items do not have same label type. Please deselect some of them.", ERROR);
      label_button_->setEnabled(false);
      return;
    }
    clearStatusReport();
    label_button_->setEnabled(true);
  }
  else
  {
    setStatusReport("Unknown list " + current->listWidget()->objectName().toStdString() + ".", ERROR);
  }
}

void TaskMonitorGui::detect(int detect)
{
  // ROS_VERIFY(task_event_detector_client_->detect(static_cast<bool>(detect)));
}

void TaskMonitorGui::stream()
{
  if(streaming_)
  {
    setStatusReport("Already streaming. Cannot start streaming. This should never happen.", ERROR);
    return;
  }
  if(recording_)
  {
    setStatusReport("Already recording. Cannot start streaming. This should never happen.", ERROR);
    return;
  }
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  ROS_VERIFY(task_recorder_manager_client.startStreaming());

  stream_button_->setEnabled(false);
  record_sample_button_->setEnabled(false);
  record_button_->setEnabled(false);
  interrupt_button_->setEnabled(true);
  streaming_ = true;
}

void TaskMonitorGui::recordSample()
{
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  task_recorder2_msgs::Description description;
  description.description = description_line_->text().toStdString();
  description.id = id_spin_box_->value();
  std::stringstream ss;
  ss << description.id;
  setStatusReport("Recorded sample " + description.description + " with id " + ss.str() + ".", INFO);
  ROS_VERIFY(task_recorder_manager_client.getDataSample(description));
  loadDataSampleDescriptionsSignal(true, QString(task_recorder2_utilities::getFileName(description).c_str()));
  focusDataSampleDescriptionSignal();
}

void TaskMonitorGui::record()
{
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  if (!recording_) // start recording
  {
    record_button_->setStyleSheet(COLOR_STYLE.arg(COLOR_RED.name()));

    //    // TODO: why is this neccessary ?
    //    std::vector<task_recorder2_msgs::TaskRecorderSpecification> specifications;
    //    ROS_VERIFY(task_recorder2_utilities::readTaskRecorderSpecification(specifications));
    //    std::vector<std::string> variable_names;
    //    ROS_VERIFY(task_recorder2_utilities::getAllVariableNames(specifications, variable_names));

    recorded_description_.description = description_line_->text().toStdString();
    recorded_description_.id = id_spin_box_->value();

    std::stringstream ss;
    ss << recorded_description_.id;
    setStatusReport("Start recording " + description_line_->text().toStdString() + " with id " + ss.str() + ".", INFO);

    ROS_VERIFY(task_recorder_manager_client.startRecording(recorded_description_, start_time_));
    start_time_ = start_time_ + ros::Duration(0.1);

    recording_duration_ = duration_spin_box_->value();
    end_time_ = start_time_ + ros::Duration(recording_duration_);

    record_sample_button_->setEnabled(false);
    stream_button_->setEnabled(false);
    interrupt_button_->setEnabled(true);
  }
  else // stop recording
  {
    record_button_->setStyleSheet(COLOR_STYLE.arg(COLOR_BLACK.name()));

    ros::Time now = ros::Time::now();
    if (now < end_time_)
    {
      ros::Duration wait_duration = end_time_ - now;
      std::stringstream ss;
      ss << std::setprecision(2) << wait_duration.toSec();
      setStatusReport("Not done recording yet... waiting for " + ss.str() + " seconds.", WARN);
      wait_duration.sleep();
    }
    setStatusReport("Stopped recording.", INFO);
    ros::Duration(0.2).sleep(); // TODO: is this neccessary ?
    // TODO: filtered_and_cropped_messages are not used, remove them ?
    std::vector<task_recorder2_msgs::DataSample> filtered_and_cropped_messages;
    ROS_VERIFY(task_recorder_manager_client.stopRecording(start_time_, end_time_, recording_duration_ * task_recorder_manager_sampling_rate_, filtered_and_cropped_messages));

    record_sample_button_->setEnabled(true);
    stream_button_->setEnabled(true);
    interrupt_button_->setEnabled(false);

    loadDataSampleDescriptionsSignal(true, QString(task_recorder2_utilities::getFileName(recorded_description_).c_str()));
    focusDataSampleDescriptionSignal();
  }
  recording_ = !recording_;
}

void TaskMonitorGui::focusDataSampleDescription()
{
  task_recorder2_msgs::Description description;
  description.description = description_line_->text().toStdString();
  description.id = id_spin_box_->value();
  dataSampleDescriptionChangedSignal(QString(task_recorder2_utilities::getFileName(description).c_str()));
}

void TaskMonitorGui::interrupt()
{
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  ROS_VERIFY(task_recorder_manager_client.interruptRecording());
  if(recording_)
  {
    recording_ = false;
    record_button_->toggle();
  }
  if(streaming_)
  {
    streaming_ = false;
  }
  interrupt_button_->setEnabled(false);
  stream_button_->setEnabled(true);
  record_sample_button_->setEnabled(true);
  record_button_->setEnabled(true);
  record_button_->setStyleSheet(COLOR_STYLE.arg(COLOR_BLACK.name()));
}

void TaskMonitorGui::loadDefault()
{
  double svm_c;
  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_c", svm_c));
  svm_c_sbox_->setValue(svm_c);
  double svm_eps;
  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_eps", svm_eps));
  svm_eps_sbox_->setValue(svm_eps);
  double kernel_width;
  ROS_VERIFY(usc_utilities::read(node_handle_, "kernel_width", kernel_width));
  kernel_width_sbox_->setValue(kernel_width);
  int kernel_type;
  ROS_VERIFY(usc_utilities::read(node_handle_, "kernel_type", kernel_type));
  int svm_lib;
  ROS_VERIFY(usc_utilities::read(node_handle_, "svm_lib", svm_lib));
  setLibAndKernel(svm_lib, kernel_type);
}

bool TaskMonitorGui::setLibAndKernel(const int svm_lib, const int kernel_type)
{
  std::map<std::string, int>::const_iterator mi;
  bool found = false;
  for (mi = kernel_type_map_.begin(); !found && mi != kernel_type_map_.end(); ++mi)
  {
    if(mi->second == kernel_type)
    {
      kernel_type_box_->setCurrentIndex(kernel_type_box_->findText(QString(mi->first.c_str())));
      kernel_type_box_->setEnabled(true);
      found = true;
    }
  }
  if(!found)
  {
    std::stringstream ss;
    ss << kernel_type;
    setStatusReport("SVM kernel type " + ss.str() + " is invalid.", ERROR);
  }
  found = false;
  for (mi = svm_lib_map_.begin(); !found && mi != svm_lib_map_.end(); ++mi)
  {
    if(mi->second == svm_lib)
    {
      svm_lib_box_->setCurrentIndex(svm_lib_box_->findText(QString(mi->first.c_str())));
      svm_lib_box_->setEnabled(true);
      found = true;
    }
  }
  if(!found)
  {
    std::stringstream ss;
    ss << svm_lib;
    setStatusReport("SVM lib " + ss.str() + " is invalid.", ERROR);
  }
  QCoreApplication::processEvents();
  repaint();
  return true;
}

bool TaskMonitorGui::getLibAndKernel(int& svm_lib, int& kernel_type)
{
  std::map<std::string, int>::const_iterator mi;
  mi = svm_lib_map_.find(svm_lib_box_->currentText().toStdString());
  ROS_ASSERT_MSG(mi != svm_lib_map_.end(), "SVM lib >%s< is invalid. This should never happen.", svm_lib_box_->currentText().toStdString().c_str());
  svm_lib = mi->second;
  mi = kernel_type_map_.find(kernel_type_box_->currentText().toStdString());
  ROS_ASSERT_MSG(mi != kernel_type_map_.end(), "Kernel type >%s< is invalid. This should never happen.", kernel_type_box_->currentText().toStdString().c_str());
  kernel_type = mi->second;
  return true;
}

bool TaskMonitorGui::loadSVMInfo()
{
  task_event_detector::SVMParametersMsg msg;
  ROS_VERIFY(task_event_detector_client_->getSVMParametersMsg(msg));
  svm_c_sbox_->setValue(msg.svm_c);
  svm_eps_sbox_->setValue(msg.svm_eps);
  kernel_width_sbox_->setValue(msg.kernel_width);
  setLibAndKernel(msg.svm_lib, msg.kernel_type);
  if(msg.variable_names.empty())
  {
    setStatusReport("No detection variable names read from SVM. This should never happen.", ERROR);
    return false;
  }
  else
  {
    setDetectionVariableNames(msg.variable_names);
  }
  return true;
}

bool TaskMonitorGui::getSVMInfo(task_event_detector::SVMParametersMsg& msg)
{
  if(!getLibAndKernel(msg.svm_lib, msg.kernel_type))
  {
    setStatusReport("Could not get classifier and kernel type.", ERROR);
    return false;
  }
  msg.svm_c = svm_c_sbox_->value();
  msg.svm_eps = svm_eps_sbox_->value();
  msg.kernel_width = kernel_width_sbox_->value();
  msg.classification_boundary = svm_boundary_spin_box_->value();
  return true;
}

bool TaskMonitorGui::setSVMInfo()
{
  task_event_detector::SVMParametersMsg new_parameter_msg;
  ROS_VERIFY(getSVMInfo(new_parameter_msg));
  task_event_detector::SVMParametersMsg current_parameter_msg;
  ROS_VERIFY(task_event_detector_client_->getSVMParametersMsg(current_parameter_msg));
  new_parameter_msg.num_data_samples = current_parameter_msg.num_data_samples;
  new_parameter_msg.num_variables = current_parameter_msg.num_variables;
  new_parameter_msg.variable_names = current_parameter_msg.variable_names;
  new_parameter_msg.indices = current_parameter_msg.indices;
  return task_event_detector_client_->setSVMParametersMsg(new_parameter_msg);
}

void TaskMonitorGui::crossvalidate()
{
  task_recorder2_msgs::Description svm_description;
  if(!getSVMName(svm_description))
  {
    return;
  }
  task_event_detector::SVMParametersMsg msg;
  if(!task_event_detector_client_->doesSVMExist(svm_description))
  {
    model_selection_gui_->loadDefault();
    if(!model_selection_gui_->getSVMParametersMsg(msg))
    {
      setStatusReport("Could not get SVM parameters.", ERROR);
      return;
    }
    getDetectionVariableNames(msg.variable_names);
    if(msg.variable_names.empty())
    {
      setStatusReport("No detection variable names selected.", ERROR);
      return;
    }
    if(!getSVMInfo(msg))
    {
      setStatusReport("Could not get SVM parameters.", ERROR);
      return;
    }
  }
  else // reload cross validation parameters
  {
    if(!task_event_detector_client_->load(svm_description))
    {
      setStatusReport("Could not load SVM " + task_recorder2_utilities::getFileName(svm_description) + "." , ERROR);
      return;
    }

    // TODO: This is horrible... we are using 2 SVMs. This will be changed after the next test.
    if (!task_event_detector_client_->setDescription(svm_description))
    {
      setStatusReport("Could not load SVM " + task_recorder2_utilities::getFileName(svm_description) + "." , ERROR);
      return;
    }

    if(!task_event_detector_client_->getSVMParametersMsg(msg))
    {
      setStatusReport("Could not get SVM parameters." , ERROR);
      return;
    }
    setDetectionVariableNames(msg.variable_names);
  }
  if(!model_selection_gui_->setup(svm_description, added_descriptions_, msg, this))
  {
    setStatusReport("Could not setup cross validation gui." , ERROR);
    return;
  }
}

void TaskMonitorGui::doneCrossvalidating(const task_recorder2_msgs::Description& svm_description,
                                         const task_event_detector::SVMParametersMsg& parameters)
{
  svm_name_->setText(QString(task_recorder2_utilities::getFileName(svm_description).c_str()));
  ROS_VERIFY(loadSVMDescriptions(true, svm_name_->text()));
  train_button_->setEnabled(false);
  crossvalidate_svm_button_->setEnabled(false);

}

void TaskMonitorGui::train()
{
  if(added_descriptions_.empty())
  {
    setStatusReport("No data added. This should never happen.", ERROR);
    return;
  }
  if(!readIntoSVM(added_descriptions_))
  {
    return;
  }
  ROS_VERIFY(setSVMInfo());
  ROS_VERIFY(task_event_detector_client_->train());
  add_button_->setEnabled(false);
  add_all_button_->setEnabled(false);
  train_button_->setEnabled(false);
  crossvalidate_svm_button_->setEnabled(false);
  // detect_box_->setEnabled(true);
  save_button_->setEnabled(true);
}

void TaskMonitorGui::clearSVM()
{
  added_descriptions_.clear();
  ROS_VERIFY(task_event_detector_client_->reset());
  task_event_detector::SVMParametersMsg msg;
  ROS_VERIFY(getSVMInfo(msg));
  ROS_VERIFY(task_event_detector_client_->setSVMParametersMsg(msg));
  check_button_->setEnabled(false);
  train_button_->setEnabled(false);
  crossvalidate_svm_button_->setEnabled(false);
  detect_box_->setEnabled(false);
  save_button_->setEnabled(false);
}

bool TaskMonitorGui::loadSVMDescriptions(const bool search, const QString& description_qstring)
{
  std::vector<std::string> descriptions;
  task_event_detector_client_->getListOfSVMs(descriptions);
  svm_box_->clear();
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    svm_box_->addItem(QString(descriptions[i].c_str()));
  }
  if(svm_box_->count() > 0)
  {
    svm_box_->setEnabled(true);
    if(search)
    {
      int index = svm_box_->findText(description_qstring);
      if (index >= 0)
      {
        svm_box_->setCurrentIndex(index);
        load(description_qstring);
      }
    }
  }
  return true;
}

bool TaskMonitorGui::getSVMName(task_recorder2_msgs::Description& svm_description)
{
  if(!task_recorder2_utilities::parseDescriptionString(svm_name_->text().toStdString(), svm_description.description, svm_description.id))
  {
    setStatusReport("Invalid SVM name " + svm_name_->text().toStdString() + ". It should be of the form STRING_INT.", ERROR);
    return false;
  }
  return true;
}

void TaskMonitorGui::save()
{
  task_recorder2_msgs::Description description;
  if (!getSVMName(description))
  {
    return;
  }
  if (!task_event_detector_client_->save(description))
  {
    setStatusReport("Problems when loading SVM " + svm_name_->text().toStdString() + ".", ERROR);
    return;
  }
  ROS_VERIFY(loadSVMDescriptions(true, svm_name_->text()));
  save_button_->setEnabled(false);
}

void TaskMonitorGui::load(const QString& description_string)
{
  task_recorder2_msgs::Description description;
  ROS_VERIFY(task_recorder2_utilities::parseDescriptionString(description_string.toStdString(), description.description, description.id));

  if(!task_event_detector_client_->load(description))
  {
    setStatusReport("Problems when loading SVM " + description_string.toStdString() + ".", ERROR);
    check_button_->setEnabled(false);
    detect_box_->setEnabled(false);
    return;
  }

  // TODO: This is horrible... we are using 2 SVMs. This will be changed after the next test.
  if (!task_event_detector_client_->setDescription(description))
    {
    setStatusReport("Problems when loading SVM " + description_string.toStdString() + ".", ERROR);
    check_button_->setEnabled(false);
    detect_box_->setEnabled(false);
    return;
  }
  else
  {
    check_button_->setEnabled(true);
    detect_box_->setEnabled(true);
  }
  ROS_VERIFY(loadSVMInfo());
  save_button_->setEnabled(true);
  svm_boundary_spin_box_->setValue(task_event_detector_client_->getClassificationBoundary());
}

void TaskMonitorGui::check()
{
  task_recorder2_msgs::DataSampleLabel label;
  ROS_VERIFY(task_event_detector_client_->checkForEvent(label));
  std::stringstream ss;
  ss << std::setprecision(2) << label.cost_label.cost;
  if(label.binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED)
  {
    setStatusReport("Yes, event occured (" + ss.str() + ").", INFO);
  }
  else
  {
    setStatusReport("No, event did not occure (" + ss.str() + ").", WARN);
  }
}

bool TaskMonitorGui::readIntoSVM(const std::vector<task_recorder2_msgs::Description>& descriptions)
{
  if(descriptions.empty())
  {
    setStatusReport("No description provided. Cannot add anything to SVM. This should never happen.", ERROR);
    return false;
  }
  std::vector<std::string> detection_variable_names;
  getDetectionVariableNames(detection_variable_names);
  if(detection_variable_names.empty())
  {
    setStatusReport("No detection variable names selected.", ERROR);
    return false;
  }
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    ROS_INFO("Adding >%s< with id >%i< and trial >%i< to SVM.",
              descriptions[i].description.c_str(), descriptions[i].id, descriptions[i].trial);
    ROS_VERIFY(task_event_detector_client_->add(descriptions[i], detection_variable_names));
  }
  return true;
}

void TaskMonitorGui::addToSVM(const std::vector<task_recorder2_msgs::Description>& descriptions)
{
  svm_name_->setText(QString(task_recorder2_utilities::getFileName(descriptions.front()).c_str()));
  detect(0);
  detect_box_->setEnabled(false);
  if (task_event_detector_client_->isTrained())
  {
    clearSVM();
  }
  added_descriptions_.insert(added_descriptions_.end(), descriptions.begin(), descriptions.end());
  add_button_->setEnabled(false);
  train_button_->setEnabled(true);
  crossvalidate_svm_button_->setEnabled(true);
}

void TaskMonitorGui::add()
{
  std::vector<task_recorder2_msgs::Description> descriptions;
  getSelectedDescriptions(labeled_list_, descriptions);
  addToSVM(descriptions);
  removeSelectedItems(labeled_list_);
  if(labeled_list_->count() == 0)
  {
    add_all_button_->setEnabled(false);
  }
  clear_svm_button_->setEnabled(true);
}

void TaskMonitorGui::addAll()
{
  std::vector<task_recorder2_msgs::Description> descriptions;
  getAllDescriptions(labeled_list_, descriptions);
  addToSVM(descriptions);
  add_all_button_->setEnabled(false);
  clear_svm_button_->setEnabled(true);
  labeled_list_->clear();
}

void TaskMonitorGui::labelLastSample()
{
  std::vector<task_recorder2_msgs::DataSample> data_samples;
  if(!usc_utilities::FileIO<task_recorder2_msgs::DataSample>::readFromBagFile(data_samples, task_event_detector::DATA_SAMPLE_TOPIC_NAME, task_event_detector::LAST_DATA_SAMPLES_BAG_FILE_NAME, false))
  {
    setStatusReport("Could not read file " + task_event_detector::LAST_DATA_SAMPLES_BAG_FILE_NAME + ".", ERROR);
    return;
  }

  task_recorder2_msgs::Description description;
  if(!usc_utilities::FileIO<task_recorder2_msgs::Description>::readFromBagFile(description, task_event_detector::DESCRIPTION_TOPIC_NAME, task_event_detector::LAST_DESCRIPTION_BAG_FILE_NAME, false))
  {
    setStatusReport("Could not read file " + task_event_detector::LAST_DESCRIPTION_BAG_FILE_NAME + ".", ERROR);
    return;
  }

  ROS_VERIFY(task_event_detector_client_->addDataSamples(description, data_samples));
  dataSampleDescriptionChangedSignal(QString(task_recorder2_utilities::getFileName(description).c_str()));
}

void TaskMonitorGui::setLabel()
{
  std::vector<task_recorder2_msgs::Description> descriptions;
  getSelectedDescriptions(unlabeled_list_, descriptions);
  if(!label_gui_client_.setLabel(descriptions, true))
  {
    setStatusReport("Aborted labeling.", WARN);
    return;
  }

  insertDescriptionsSignal(labeled_list_, descriptions);
  removeSelectedItems(unlabeled_list_);
  label_button_->setEnabled(false);
  add_all_button_->setEnabled(true);
}

void TaskMonitorGui::setClassificationBoundary(double value)
{
  task_event_detector_client_->setClassificationBoundary(value);
}

void TaskMonitorGui::clearStatusReport()
{
  status_text_edit_->clear();
}

void TaskMonitorGui::openBagFile()
{
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  std::vector<task_recorder2_msgs::Description> descriptions;
  getSelectedDescriptionsSignal(labeled_list_, descriptions);
  if(descriptions.empty())
  {
    setStatusReport("No labeled description selected.", ERROR);
    return;
  }

  for (int i = 0; i < (int)rxbag_processes_.size(); ++i)
  {
    rxbag_processes_[i]->kill();
  }
  rxbag_processes_.clear();

  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    std::string abs_file_name;
    if(!task_recorder_manager_client.getInfo(descriptions[i], abs_file_name))
    {
      setStatusReport("Could not get file name of description " + task_recorder2_utilities::getFileName(descriptions[i]) + ".", ERROR);
      return;
    }
    QStringList cmd_list;
    cmd_list.append(QString(abs_file_name.c_str()));

    boost::shared_ptr<QProcess> rxbag_process(new QProcess(this));
    rxbag_process->start(QString("rxbag"), cmd_list);

    if(rxbag_process->state() != QProcess::Running && rxbag_process->error() == QProcess::FailedToStart)
    {
      setStatusReport("Problems when opening " + abs_file_name + ".", ERROR);
      rxbag_process->kill();
      return;
    }
    rxbag_processes_.push_back(rxbag_process);
  }
}

void TaskMonitorGui::extract()
{
  task_recorder2::TaskRecorderManagerClient task_recorder_manager_client;
  std::vector<task_recorder2_msgs::Description> descriptions;
  getSelectedDescriptionsSignal(labeled_list_, descriptions);
  if(descriptions.empty())
  {
    setStatusReport("No labeled description selected.", ERROR);
    return;
  }

  const int INDEX = extract_index_spin_box_->value();
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    std::vector<task_recorder2_msgs::DataSample> data_samples;
    if(!task_recorder_manager_client.readDataSamples(descriptions[i], data_samples))
    {
      setStatusReport("Could not read " + task_recorder2_utilities::getFileName(descriptions[i]) + ".", ERROR);
      return;
    }
    if(INDEX < 0 || INDEX >= (int)data_samples.size())
    {
      setStatusReport("Selected index " + boost::lexical_cast<std::string>(INDEX) + " is invalid. There are "
                      + boost::lexical_cast<std::string>((int)data_samples.size()) + " data samples.", ERROR);
      return;
    }

    task_recorder2_msgs::DataSample data_sample = data_samples[INDEX];
    if(!task_recorder_manager_client.addDataSample(descriptions[i], data_sample))
    {
      setStatusReport("Could not add data sample with description " + task_recorder2_utilities::getFileName(descriptions[i]) + ".", ERROR);
      return;
    }
  }
}

void TaskMonitorGui::setStatusReport(const std::string& status_report, const StatusReportMode mode)
{
  ROS_DEBUG_STREAM_COND(mode==DEBUG, status_report);
  ROS_INFO_STREAM_COND(mode==INFO, status_report);
  ROS_WARN_STREAM_COND(mode==WARN, status_report);
  ROS_ERROR_STREAM_COND(mode==ERROR, status_report);
  ROS_FATAL_STREAM_COND(mode==FATAL, status_report);

  // ROS_INFO_STREAM(status_report);
  std::string status_string;
  const std::string debug_color = "#000000"; // black
  const std::string info_color = "#006400"; // dark green
  const std::string warn_color = "#FF6600"; // orange
  const std::string error_color = "#FF0000"; // red
  const std::string fatal_color = "#9400D3"; // dark violoet
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
  QCoreApplication::processEvents();
  repaint();
}

}
