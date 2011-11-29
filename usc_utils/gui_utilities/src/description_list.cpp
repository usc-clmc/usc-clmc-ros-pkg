/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		description_list.cpp

  \author	Peter Pastor
  \date		Jun 22, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>

#include <task_recorder2_utilities/task_description_utilities.h>

// local includes
#include <gui_utilities/description_list.h>

namespace gui_utilities
{

DescriptionList::DescriptionList(QListWidget* list, const std::string& name, const bool use_id) :
    name_(name), list_(list), use_id_(use_id)
{
  // signals/slots mechanism in action

}

std::string DescriptionList::getName() const
{
  return name_;
}

void DescriptionList::print() const
{
  ROS_INFO("DescriptionList: >%s<", name_.c_str());
  QList<QListWidgetItem*> items = list_->selectedItems ();
  for (int i = 0; i < (int)items.size(); ++i)
  {
    ROS_INFO("Item >%s< is selected.", items[i]->text().toStdString().c_str());
  }
  std::vector<task_recorder2_msgs::Description> descriptions;
  if(!isEmpty())
  {
    ROS_VERIFY(getAllDescriptions(descriptions));
    for (int i = 0; i < (int)descriptions.size(); ++i)
    {
      ROS_INFO("Item >%s< trial: >%i< is contained.", task_recorder2_utilities::getFileName(descriptions[i]).c_str(), descriptions[i].trial);
    }
  }
}

void DescriptionList::insertDescription(const task_recorder2_msgs::Description& description)
{
  std::string list_item_text = description.description;
  if(use_id_)
  {
    list_item_text = task_recorder2_utilities::getFileName(description);
    list_item_text.append(std::string(" trial:") + usc_utilities::getString(description.trial));
  }
  QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(list_item_text));
  descriptions_.insert(DescriptionPair(item, description));
  list_->addItem(item);
}

bool DescriptionList::getDescriptions(std::vector<task_recorder2_msgs::Description>& descriptions)
{
  QList<QListWidgetItem*> items = list_->selectedItems ();
  if(items.isEmpty())
  {
    return false;
  }
  for (int i = 0; i < (int)items.size(); ++i)
  {
    descriptions_iterator_ = descriptions_.find(items[i]);
    ROS_ASSERT_MSG(descriptions_iterator_ != descriptions_.end(), "List item >%s< not found. This should never happen.", items[i]->text().toStdString().c_str());
    descriptions.push_back(descriptions_iterator_->second);
  }
  return true;
}

void DescriptionList::removeSelectedItems()
{
  QList<QListWidgetItem*> items = list_->selectedItems ();
  for (int i = 0; i < (int)items.size(); ++i)
  {
    descriptions_.erase(items[i]);
    delete items[i];
  }
  ROS_ASSERT_MSG((int)descriptions_.size() == list_->count(), "Number of descriptions >%i< does not match number of list items >%i<.", (int)descriptions_.size(), list_->count());
}

bool DescriptionList::getAllDescriptions(std::vector<task_recorder2_msgs::Description>& descriptions) const
{
  if(descriptions_.empty())
  {
    ROS_ERROR("No descriptions in list, that is sad... want to return all of them. This should never happen.");
    return false;
  }
  descriptions.clear();
  std::map<QListWidgetItem*, task_recorder2_msgs::Description>::const_iterator descriptions_iterator;
  for(descriptions_iterator = descriptions_.begin(); descriptions_iterator != descriptions_.end(); ++descriptions_iterator)
  {
    descriptions.push_back(descriptions_iterator->second);
  }
  return true;
}

bool DescriptionList::isEmpty() const
{
  return (list_->count() == 0);
}

void DescriptionList::clearList()
{
  if(descriptions_.empty())
  {
    ROS_ERROR("No descriptions in list, that is sad... want to clear it. This should never happen.");
    return;
  }
  descriptions_.clear();
  list_->clear();
  if ((int)descriptions_.size() != list_->count())
  {
    ROS_ERROR("Got >%i< descriptions but >%i< list items.", (int)descriptions_.size(), list_->count());
    // ROS_ASSERT((int)descriptions_.size() == list_->count());
  }
}

}
