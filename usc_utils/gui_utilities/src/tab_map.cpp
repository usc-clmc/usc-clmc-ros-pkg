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

// system includes
#include <usc_utilities/assert.h>

// local includes
#include <gui_utilities/tab_map.h>

namespace gui_utilities
{

TabMap::TabMap()
{
}

void TabMap::insert(const int label, QWidget* tab)
{
  label_to_tab_map_.insert(LabelToWidgetPair(label, tab));
}

void TabMap::enableTab(const int label_type, QTabWidget* tabs)
{
  tabs->setEnabled(true);
  for (int i = 0; i < tabs->count(); ++i)
  {
    tabs->setTabEnabled(i, false);
  }
  QWidget* tab = getLabelTab(label_type);
  tabs->setCurrentWidget(tab);
  tabs->setTabEnabled(tabs->indexOf(tab), true);
}

QWidget* TabMap::getLabelTab(const int& label_type)
{
  label_to_tab_map_iterator_ = label_to_tab_map_.find(label_type);
  ROS_ASSERT_MSG(label_to_tab_map_iterator_ != label_to_tab_map_.end(), "Label type >%i< not found. This should never happen.", label_type);
  return label_to_tab_map_iterator_->second;
}

}

