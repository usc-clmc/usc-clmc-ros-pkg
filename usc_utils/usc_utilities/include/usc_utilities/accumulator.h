/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks    ...

  \file   accumulator.cpp

  \author Peter Pastor
  \date   Nov 3, 2010

 *********************************************************************/

#ifndef USC_UTILITIES_ACCUMULATOR_H_
#define USC_UTILITIES_ACCUMULATOR_H_

// system includes
#include <vector>

// ros includes
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

// local includes
#include <usc_utilities/AccumulatedTrialStatistics.h>

namespace usc_utilities
{

class Accumulator
{

public:

    typedef boost::accumulators::accumulator_set<double,boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance> > AccumulatorType;

    /*! Constructor
     */
    Accumulator() :
      initialized_(false), num_data_traces_(0), num_samples_(0) {};

    /*! Destructor
     */
    virtual ~Accumulator() {};

    /*!
     * @param num_data_traces
     * @param num_samples
     * @return
     */
    bool initialize(const int num_data_traces, const int num_samples);

    /*!
     *
     * @param data_trace_index
     * @param data_trace
     * @return
     */
    bool add(const int data_trace_index, std::vector<double>& data_trace);

    /*!
     *
     * @param trial_statistics
     * @return
     */
    bool getAccumulatedTrialStatistics(std::vector<usc_utilities::AccumulatedTrialStatistics>& accumulated_trial_statistics);

private:

    /*!
     */
    bool initialized_;

    /*!
     */
    int num_data_traces_;
    int num_samples_;

    /*!
     */
    std::vector<std::vector<AccumulatorType> > accumulators_;
};

}


#endif /* USC_UTILITIES_ACCUMULATOR_H_ */
