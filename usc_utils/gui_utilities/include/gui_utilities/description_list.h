/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		description_list.h

  \author	Peter Pastor
  \date		Jun 22, 2011

 *********************************************************************/

#ifndef DESCRIPTION_LIST_H_
#define DESCRIPTION_LIST_H_

// system includes
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

#include <QtCore>
#include <QtGui/QListWidget>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes

namespace gui_utilities
{

class DescriptionList : public QObject
{
Q_OBJECT

public:

  typedef std::pair<QListWidgetItem*, task_recorder2_msgs::Description> DescriptionPair;

  /*! Constructor
   */
  DescriptionList(QListWidget* list, const std::string& name, const bool use_id = true);
  /*! Destructor
   */
  virtual ~DescriptionList() {};

public Q_SLOTS:

  /*!
   * @param description
   */
  void insertDescription(const task_recorder2_msgs::Description& description);

  /*!
   * @param description
   */
  void insertDescription(const std::vector<task_recorder2_msgs::Description>& descriptions)
  {
    for(int i=0; i<(int)descriptions.size(); ++i)
    {
      insertDescription(descriptions[i]);
    }
  }

  /*!
   */
  void print() const;
  std::string getName() const;

  /*!
   * @param descriptions
   * @return True if items have been selected, otherwise False
   */
  bool getDescriptions(std::vector<task_recorder2_msgs::Description>& descriptions);

  /*!
   * @param descriptions
   * @return True on success, otherwise False
   */
  bool getAllDescriptions(std::vector<task_recorder2_msgs::Description>& descriptions) const;

  /*!
   * @return True if list is empty, otherwise False
   */
  bool isEmpty() const;

  /*!
   */
  void clearList();

  /*!
   */
  void removeSelectedItems();

private:

  std::string name_;
  std::map<QListWidgetItem*, task_recorder2_msgs::Description> descriptions_;
  std::map<QListWidgetItem*, task_recorder2_msgs::Description>::const_iterator descriptions_iterator_;
  QListWidget* list_;
  bool use_id_;

};

/*! Abreviations for convenience
 */
typedef boost::shared_ptr<DescriptionList> DescriptionListPtr;

}

#endif /* DESCRIPTION_LIST_H_ */
