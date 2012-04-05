/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    accumulator.h

 \author  Peter Pastor
 \date    Jul 28, 2010

 *********************************************************************/

#ifndef ACCUMULATOR_H_
#define ACCUMULATOR_H_

// system includes
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>

// ros includes
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

// local includes
#include <task_recorder2_msgs/AccumulatedTrialStatistics.h>
#include <task_recorder2_msgs/DataSample.h>

namespace task_recorder2_utilities
{

class Accumulator
{

public:

  typedef boost::accumulators::accumulator_set<double,
      boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance> > AccumulatorType;

  Accumulator() :
    counter_(0), num_data_traces_(0), num_data_samples_(0) {};
  virtual ~Accumulator() {};

 /*!
  * @param data_samples
  * @param headers
  */
  void getHeaders(const std::vector<task_recorder2_msgs::DataSample>& data_samples,
                  std::vector<std_msgs::Header>& headers);

  /*!
   */
  void clear();

  /*!
   * @param data_samples
   * @param accumulated_trial_statistics
   * @return True on success, otherwise False
   */
  bool accumulate(const std::vector<task_recorder2_msgs::DataSample>& data_samples);

  /*!
   * @param accumulated_trial_statistics
   * @return True on success, otherwise False
   */
  bool getAccumulatedTrialStatistics(std::vector<task_recorder2_msgs::AccumulatedTrialStatistics>& accumulated_trial_statistics);

private:

  int counter_;
  int num_data_traces_;
  int num_data_samples_;
  Eigen::MatrixXd accumulators_;
  Eigen::MatrixXd buffer_;

  // std::vector<std::vector<AccumulatorType> > accumulators_;

};

}


#endif /* ACCUMULATOR_H_ */
