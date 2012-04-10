/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		fft_signal_processor.h

  \author	Peter Pastor
  \date		Jul 22, 2011

 *********************************************************************/

#ifndef FFT_SIGNAL_PROCESSOR_H_
#define FFT_SIGNAL_PROCESSOR_H_

// system includes
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

#include <fftw3.h>
#include <ros/ros.h>

#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

#include <task_recorder2_utilities/message_ring_buffer.h>

#include <task_recorder2_utilities/data_sample_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>

#include <task_recorder2_msgs/DataSample.h>

// local includes

namespace task_signal_processor
{

class FFTSignalProcessor
{

public:

  /*! Constructor
   */
  FFTSignalProcessor();
  /*! Destructor
   */
  virtual ~FFTSignalProcessor();

  /*!
   * @param num_frames_per_period
   * @param num_output_signals
   * @param output_sample_rate
   * @param apply_window
   * @param mel_filter_parameter_a
   * @param mel_filter_parameter_b
   * @return True on success, otherwise False
   */
  bool initialize(const int num_frames_per_period,
                  const int num_output_signals,
                  const double output_sample_rate,
                  bool apply_window = false,
                  const double mel_filter_parameter_a = 700.0,
                  const double mel_filter_parameter_b = 2590.0);

  /*!
   * @param value
   * @param data
   * @return True on success, False otherwise
   */
  bool filter(const double value, std::vector<double>& data);

private:

  bool initialized_;
  int num_frames_per_period_;
  int num_output_signals_;
  bool apply_window_;

  std::vector<double> current_data_;
  Eigen::VectorXd output_signal_spectrum_;
  Eigen::VectorXd amplitude_spectrum_;
  Eigen::VectorXd hamming_window_;

  Eigen::MatrixXd mel_filter_bank_;

  double* fftw_input_;
  fftw_complex *fftw_out_;
  fftw_plan fftw_plan_;

  boost::shared_ptr<task_recorder2_utilities::CircularMessageBuffer<double> > cb_;

  double output_sample_rate_;
  bool initMelFilterBank(const double a, const double b);
  void setupOutput();
};

}


#endif /* FFT_SIGNAL_PROCESSOR_H_ */
