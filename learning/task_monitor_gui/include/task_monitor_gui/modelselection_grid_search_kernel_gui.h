/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks Check:
           http://www.csie.ntu.edu.tw/~cjlin/papers/guide/guide.pdf
           http://www.csie.ntu.edu.tw/~cjlin/papers/libsvm.pdf

  \file   modelselection_grid_search_kernel_gui.h

  \author Peter Pastor
  \date   Jul 22, 2011

 *********************************************************************/

#ifndef MODELSELECTION_GRID_SEARCH_KERNEL_GUI_H_
#define MODELSELECTION_GRID_SEARCH_KERNEL_GUI_H_

// system includes
#include <ros/ros.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>
#include <task_recorder2_msgs/Description.h>

#include <task_event_detector/SVMParametersMsg.h>
#include <task_event_detector/modelselection_grid_search_kernel.h>

// local includes
#include <task_monitor_gui/model_selection_grid_search_gui_main_window.h>

namespace task_monitor_gui
{

class ModelSelectionGridSearchKernelGui : public QMainWindow, private Ui::modelselection_grid_search_kernel_gui
{
Q_OBJECT

private:

  /*!
   */
  enum StatusReportMode
  {
    DEBUG = 0, //!< DEBUG
    INFO, //!< INFO
    WARN, //!< WARN
    ERROR, //!< ERROR
    FATAL //!< FATAL
  };

public:

  /*! Constructor
   * @param node_handle
   * @param parent
   * @param flags
   */
  ModelSelectionGridSearchKernelGui(ros::NodeHandle node_handle,
                                    QWidget* parent = 0,
                                    Qt::WFlags flags = 0);

  /*! Destructor
   */
  virtual ~ModelSelectionGridSearchKernelGui() {};

  /*!
   * @param msg
   * @return
   */
  bool getSVMParametersMsg(task_event_detector::SVMParametersMsg& msg);

  /*!
   * @param msg
   * @return
   */
  bool setSVMParametersMsg(const task_event_detector::SVMParametersMsg& msg);

  /*!
   * @param svm_description
   * @param data_descriptions
   * @param parameters
   * @return True on success, otherwise False
   */
  bool setup(const task_recorder2_msgs::Description& svm_description,
             const std::vector<task_recorder2_msgs::Description>& data_descriptions,
             task_event_detector::SVMParametersMsg& parameters,
             QObject* qobject);

public Q_SLOTS:

  void loadDefault();
  void crossvalidate();
  void done();

Q_SIGNALS:

  void signalDone(const task_recorder2_msgs::Description& svm_description,
                  const task_event_detector::SVMParametersMsg& parameters);

private:

  ros::NodeHandle node_handle_;

  task_event_detector::ModelSelectionGridSearchKernel model_selection_grid_search_kernel_;

  task_recorder2_msgs::Description svm_description_;
  std::vector<task_recorder2_msgs::Description> data_descriptions_;
  task_event_detector::SVMParametersMsg parameters_;

  void clearStatusReport();
  void setStatusReport(const std::string& status_report, const StatusReportMode mode);

  bool setRange(QComboBox* box, const std::string& range);
  bool getRange(QComboBox* box, std::string& range);

};

/*! Abbreviatons for convenience
 */
typedef boost::shared_ptr<ModelSelectionGridSearchKernelGui> ModelSelectionGridSearchKernelGuiPtr;

}

#endif /* MODELSELECTION_GRID_SEARCH_KERNEL_GUI_H_ */
