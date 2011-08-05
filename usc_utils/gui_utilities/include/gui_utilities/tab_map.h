/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks      ...

  \file     tab_map.h

  \author   Peter Pastor
  \date     Jun 18, 2011

 *********************************************************************/

#ifndef TAB_MAP_H_
#define TAB_MAP_H_

// system includes
#include <map>
#include <QtGui/QWidget>
#include <QtGui/QTabWidget>

// local includes

namespace gui_utilities
{

class TabMap
{

  typedef std::pair<int, QWidget*> LabelToWidgetPair;

public:

  /*! Constructor
   */
  TabMap();
  /*! Destructor
   */
  virtual ~TabMap() {}

  /*!
   * @param label
   * @param tab
   */
  void insert(const int label, QWidget* tab);

  /*!
   * @param label_type
   * @param tabs
   */
  void enableTab(const int label_type, QTabWidget* tabs);

private:

  /*!
   * @param label_type
   * @return
   */
  QWidget* getLabelTab(const int& label_type);

  std::map<int, QWidget*> label_to_tab_map_;
  std::map<int, QWidget*>::const_iterator label_to_tab_map_iterator_;

};

}


#endif /* TAB_MAP_H_ */
