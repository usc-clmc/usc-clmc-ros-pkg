/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file     label_gui.cpp

  \author   Peter Pastor
  \date     Jun 18, 2011

 *********************************************************************/

// system includes
#include <boost/thread.hpp>

// local includes
#include <task_label_gui/label_gui.h>

namespace task_label_gui
{

LabelGui::LabelGui(ros::NodeHandle node_handle, QWidget* parent, Qt::WFlags flags) :
    QMainWindow(parent, flags), node_handle_(node_handle), task_labeler_(node_handle)
{
  // this sets up GUI
  setupUi(this);

  label_button_->setEnabled(false);
  wait_for_label_mutex_.lock();

  // signals/slots mechanism in action
  ROS_VERIFY(connect(label_button_, SIGNAL(clicked()), this, SLOT(setLabel())));
  ROS_VERIFY(connect(cancel_button_, SIGNAL(clicked()), this, SLOT(cancel())));
  ROS_VERIFY(connect(this, SIGNAL(setLabelGuiVisibility(bool)), this, SLOT(showLabelGui(bool))));
  ROS_VERIFY(connect(this, SIGNAL(insertDescriptionsSignal(QListWidget*, const std::vector<task_recorder2_msgs::Description>&)),
                     this, SLOT(insertDescriptions(QListWidget*, const std::vector<task_recorder2_msgs::Description>&))));
  ROS_VERIFY(connect(this, SIGNAL(insertDescriptionSignal(QListWidget*, const task_recorder2_msgs::Description&)),
                     this, SLOT(insertDescription(QListWidget*, const task_recorder2_msgs::Description&))));

  set_last_trial_ids_service_server_ = node_handle_.advertiseService("setLastTrialIds", &LabelGui::setLastTrialIds, this);
  set_label_service_server_ = node_handle_.advertiseService("setLabel", &LabelGui::setLabel, this);
  record_and_label_service_server_ = node_handle_.advertiseService("recordAndLabel", &LabelGui::recordAndLabel, this);

  unlabeled_list_->setObjectName("UnLabeledList");
  tab_map_.insert(task_recorder2_msgs::DataSampleLabel::BINARY_LABEL, binary_tab_);
  tab_map_.insert(task_recorder2_msgs::DataSampleLabel::COST_LABEL, cost_tab_);

  setLabelGuiVisibility(false);
  clearFocus();
}

void LabelGui::closeEvent(QCloseEvent *event)
{
  if (wait_for_label_mutex_.try_lock())
  {
    wait_for_label_mutex_.unlock();
    event->accept();
  }
  event->ignore();
}

void LabelGui::setLabel()
{
  label_button_->setEnabled(false);
  wait_for_label_mutex_.unlock();
}

void LabelGui::cancel()
{
  cancel_button_->setEnabled(false);
  wait_for_label_mutex_.unlock();
}

void LabelGui::showLabelGui(bool visible)
{
  if(!visible)
  {
    clearFocus();
    setVisible(false);
    return;
  }

  move(QCursor::pos().x() - (width()/2), QCursor::pos().y() - (height()/2));
  setVisible(true);
  raise();
  activateWindow();
  setFocus(Qt::ActiveWindowFocusReason);
  QCoreApplication::processEvents();
  repaint();
}

bool LabelGui::recordAndLabel(task_recorder2_msgs::SetLabel::Request& request,
                              task_recorder2_msgs::SetLabel::Response& response)
{
  if(!task_recorder_manager_client_.checkForServices())
  {
    response.info.assign("Task recorder manager is not ready, cannot record data sample.");
    response.return_code = task_recorder2_msgs::SetLabel::Response::FAILED;
    return true;
  }

  // recording samples
  for (int i = 0; i < (int)request.descriptions.size(); ++i)
  {
    ROS_DEBUG("Recording >%s< with id >%i< and trial >%i<.",
              request.descriptions[i].description.c_str(), request.descriptions[i].id, request.descriptions[i].trial);
    if(!task_recorder_manager_client_.getDataSample(request.descriptions[i]))
    {
      response.info.assign("Problem when recording sample for description " + request.descriptions[i].description + ".");
      response.return_code = task_recorder2_msgs::SetLabel::Response::FAILED;
      return true;
    }
  }

  task_recorder2_msgs::SetLastTrialIds::Request set_last_trials_request;
  set_last_trials_request.descriptions = request.descriptions;
  task_recorder2_msgs::SetLastTrialIds::Response set_last_trials_response;
  if (!setLastTrialIds(set_last_trials_request, set_last_trials_response))
  {
    response.return_code = task_recorder2_msgs::SetLabel::Response::FAILED;
    return true;
  }

  task_recorder2_msgs::SetLabel::Request new_request = request;
  new_request.descriptions = set_last_trials_response.descriptions;
  return setLabel(new_request, response);
}

bool LabelGui::setLastTrialIds(task_recorder2_msgs::SetLastTrialIds::Request& request,
                               task_recorder2_msgs::SetLastTrialIds::Response& response)
{
  ROS_WARN_COND(request.descriptions.empty(), "There are no descriptions provided, not returning any trial ids.");
  response.descriptions = request.descriptions;
  for (int i = 0; i < (int)response.descriptions.size(); ++i)
  {
    if (!task_labeler_.setLastTrialId(response.descriptions[i]))
    {
      ROS_ERROR("Could not set last trial id of >%s<. This should never happen!", response.descriptions[i].description.c_str());
      response.return_code = task_recorder2_msgs::SetLastTrialIds::Response::FAILED;
      return true;
    }
  }
  return true;
}

void LabelGui::done()
{
  tabs_->setEnabled(false);
  // TODO: only remove those that have been labeled.
  unlabeled_list_->clear();
  setLabelGuiVisibility(false);
}

bool LabelGui::setLabel(task_recorder2_msgs::SetLabel::Request& request,
                        task_recorder2_msgs::SetLabel::Response& response)
{
  insertDescriptionsSignal(unlabeled_list_, request.descriptions);
  setLabelGuiVisibility(true);

  // get label type
  int label_type = task_recorder2_msgs::DataSampleLabel::BINARY_LABEL;
  if(!task_labeler_.getLabelType(label_type, request.descriptions, task_recorder2_msgs::DataSampleLabel::BINARY_LABEL))
  {
    response.info.assign("Labels do not have same type. This should never happen!");
    response.return_code = task_recorder2_msgs::SetLabel::Response::FAILED;
    done();
    return true;
  }
  tab_map_.enableTab(label_type, tabs_);

  label_button_->setEnabled(true);
  cancel_button_->setEnabled(true);
  ROS_DEBUG("Waiting for user input...");
  wait_for_label_mutex_.lock();

  if(label_button_->isEnabled()) // cancel button was clicked
  {
    label_button_->setEnabled(false);
    response.info.assign("Aborted labeling.");
    response.return_code = task_recorder2_msgs::SetLabel::Response::ABORTED;
    done();
    return true;
  }
  else // label button was clicked
  {
    cancel_button_->setEnabled(false);
  }

  for (int i = 0; i < (int)request.descriptions.size(); ++i)
  {
    ROS_DEBUG("Labeling >%s< with id >%i< and trial >%i<.",
              request.descriptions[i].description.c_str(), request.descriptions[i].id, request.descriptions[i].trial);
    if(!task_labeler_.label(request.descriptions[i], getLabel()))
    {
      response.info.assign("Could not label sample named " + request.descriptions[i].description + ". This should never happen!");
      response.return_code = task_recorder2_msgs::SetLabel::Response::FAILED;
      return true;
    }
  }

  done();
  response.info.assign("Labeled data sample named ");
  for (int i = 0; i < (int)request.descriptions.size(); ++i)
  {
    if (i + 1 == (int)request.descriptions.size())
    {
      response.info.append(request.descriptions[i].description + ".");
    }
    else
    {
      response.info.append(request.descriptions[i].description + ", ");
    }
  }
  response.return_code = task_recorder2_msgs::SetLabel::Response::SUCCEEDED;
  return true;
}

task_recorder2_msgs::DataSampleLabel LabelGui::getLabel(const int label_type)
{
  task_recorder2_msgs::DataSampleLabel label;
  label.type = label_type;
  if(label_type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL)
  {
    task_recorder2_msgs::BinaryLabel binary_label;
    if(radio_button_failed_->isChecked())
    {
      binary_label.label = task_recorder2_msgs::BinaryLabel::FAILED;
    }
    else if(radio_button_succeeded_->isChecked())
    {
      binary_label.label = task_recorder2_msgs::BinaryLabel::SUCCEEDED;
    }
    else
    {
      ROS_ASSERT_MSG(false, "No label selected. This should never happen.");
    }
    label.binary_label = binary_label;
  }
  else if(label_type == task_recorder2_msgs::DataSampleLabel::COST_LABEL)
  {
    task_recorder2_msgs::CostLabel cost_label;
    label.cost_label = cost_label;
  }
  else
  {
    ROS_ASSERT_MSG(false, "Unknown label type >%i<. This should never happen.", label_type);
  }
  return label;
}

void LabelGui::insertDescription(QListWidget* list_widget,
                                 const task_recorder2_msgs::Description& description)
{
  task_recorder2_msgs::Description::Ptr list_description(new task_recorder2_msgs::Description(description));
  std::string list_item_text = task_recorder2_utilities::getFileName(description);
  list_item_text.append(std::string(" trial:") + usc_utilities::getString(description.trial));
  QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(list_item_text));
  list_widget->addItem(item);
}

void LabelGui::insertDescriptions(QListWidget* list_widget,
                                  const std::vector<task_recorder2_msgs::Description>& descriptions)
{
  for (int i = 0; i < (int)descriptions.size(); ++i)
  {
    insertDescriptionSignal(list_widget, descriptions[i]);
  }
}

}
