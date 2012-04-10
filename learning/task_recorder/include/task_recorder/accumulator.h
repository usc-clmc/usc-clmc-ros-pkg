/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

  \file    accumulator.h

  \author  Peter Pastor
  \date    Jul 28, 2010

**********************************************************************/

#ifndef TASK_RECORDER1_ACCUMULATOR_H_
#define TASK_RECORDER1_ACCUMULATOR_H_

// system includes
#include <vector>

// ros includes
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

// local includes
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
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
    bool getAccumulatedTrialStatistics(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics);

private:

    bool initialized_;

    int num_data_traces_;
    int num_samples_;

    std::vector<std::vector<AccumulatorType> > accumulators_;
};

}


#endif /* ACCUMULATOR_H_ */
