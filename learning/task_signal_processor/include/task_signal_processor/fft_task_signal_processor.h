/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		fft_task_signal_processor.h

  \author	Peter Pastor
  \date		Jul 19, 2011

 *********************************************************************/

#ifndef FFT_TASK_SIGNAL_PROCESSOR_H_
#define FFT_TASK_SIGNAL_PROCESSOR_H_

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

const int MESSAGE_SUBSCRIBER_BUFFER_SIZE = 10000;

template<class MessageType>
  class FFTSignalProcessor
  {

  public:

  typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

  /*! Constructor
   */
  FFTSignalProcessor();
  /*! Destructor
   */
  virtual ~FFTSignalProcessor() {};

  /*!
   * @param num_frames_per_period
   * @param num_output_signals
   * @param apply_window
   * @param mel_filter_parameter_a
   * @param mel_filter_parameter_b
   * @return True on success, otherwise False
   */
  bool initialize(const int num_frames_per_period, const int num_output_signals,
                  bool apply_window = true,
                  const double mel_filter_parameter_a = 700.0,
                  const double mel_filter_parameter_b = 2590.0);

  /*!
   * @param message
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool filter(task_recorder2_msgs::DataSample& data_sample);

  private:

  int num_frames_per_period_;
  int num_output_signals_;

  Eigen::VectorXd output_signal_spectrum_;
  Eigen::VectorXd amplitude_spectrum_;
  Eigen::VectorXd hamming_window_;
  bool apply_window_;

  Eigen::MatrixXd mel_filter_bank_;

  double* fftw_input_;
  fftw_complex *fftw_out_;
  fftw_plan fftw_plan_;

  boost::shared_ptr<task_recorder2_utilities::CircularMessageBuffer<double> > cb_;

  double output_sample_rate_;
  bool initMelFilterBank(const double a, const double b);
  void setupOutput();

  };

template<class MessageType>
  FFTSignalProcessor<MessageType>::FFTSignalProcessor()
  {

  }

template<class MessageType>
  bool FFTSignalProcessor<MessageType>::initialize(const int window_size, const int num_output_signals,
                                                   bool apply_window,
                                                   const double mel_filter_parameter_a,
                                                   const double mel_filter_parameter_b)
  {


    ROS_VERIFY(initMelFilterBank(mel_filter_parameter_a, mel_filter_parameter_b));
    return true;
  }

template<class MessageType>
  bool FFTSignalProcessor<MessageType>::filter(task_recorder2_msgs::DataSample& data_sample)
  {

    return true;
  }

template<class MessageType>
  bool FFTSignalProcessor<MessageType>::initMelFilterBank(const double a, const double b)
  {
    // from "Mel Frequency Cepstral Coefficients: An Evaluation of Robustness of MP3 Encoded Music"
    // Authors: Sigurdur Sigurdsson and Kaare Brandt Petersen and Tue Lehn-Schiøler
    mel_filter_bank_ = Eigen::MatrixXd::Zero((Eigen::DenseIndex)num_frames_per_period_, (Eigen::DenseIndex)num_output_signals_);

    double bandwidth = static_cast<double> (output_sample_rate_) / 2.0;
    double frequency_step = bandwidth / static_cast<double> (mel_filter_bank_.rows());
    Eigen::VectorXd f = Eigen::VectorXd::Zero(mel_filter_bank_.rows() + 1);
    for (int k = 0; k < (int)f.size(); ++k)
    {
      f(k) = static_cast<double> (k) * frequency_step;
    }

    //  std::ofstream outfile_f;
    //  outfile_f.open(std::string(std::string("/tmp/f.txt")).c_str());
    //  outfile_f << f;
    //  outfile_f.close();

    double f_max = static_cast<double> (output_sample_rate_) / 2.0;
    double phi_max = b * log10((f_max / a) + 1.0);
    double phi_min = 0.0;

    double mel_frequency_step = (phi_max - phi_min) / static_cast<double> (mel_filter_bank_.cols() - 1);
    Eigen::VectorXd phi_c = Eigen::VectorXd::Zero(mel_filter_bank_.cols() + 1);
    for (int m = 0; m < (int)phi_c.size(); ++m)
    {
      phi_c(m) = static_cast<double> (m) * mel_frequency_step;
    }
    // ROS_INFO("Max MEL frequency is >%f< Hz.", phi_c(phi_c.size()-1));

    //  std::ofstream outfile_phi_c;
    //  outfile_phi_c.open(std::string(std::string("/tmp/phi_c.txt")).c_str());
    //  outfile_phi_c << phi_c;
    //  outfile_phi_c.close();

    Eigen::VectorXd f_c = Eigen::VectorXd::Zero(phi_c.size());
    for (int i = 0; i < (int)f_c.size(); ++i)
    {
      f_c(i) = a * (pow(10.0, (phi_c(i) / b)) - 1.0);
    }

    //  std::ofstream outfile_f_c;
    //  outfile_f_c.open(std::string(std::string("/tmp/f_c.txt")).c_str());
    //  outfile_f_c << f_c;
    //  outfile_f_c.close();

    for (int k = 0; k < (int)f.size() - 1; ++k)
    {
      for (int m = 0; m < (int)f_c.size() - 1; ++m)
      {
        if (m == 0)
        {
          if (f(k) < f_c(m + 1))
          {
            mel_filter_bank_(k, m) = (f(k) - f_c(m + 1)) / (f_c(m) - f_c(m + 1));
          }
          else
          {
            mel_filter_bank_(k, m) = 0.0;
          }
        }
        else
        {
          if (f(k) < f_c(m - 1))
          {
            mel_filter_bank_(k, m) = 0.0;
          }
          else if (f_c(m - 1) <= f(k) && f(k) < f_c(m))
          {
            mel_filter_bank_(k, m) = (f(k) - f_c(m - 1)) / (f_c(m) - f_c(m - 1));
          }
          else if (f_c(m) <= f(k) && f(k) < f_c(m + 1))
          {
            mel_filter_bank_(k, m) = (f(k) - f_c(m + 1)) / (f_c(m) - f_c(m + 1));
          }
          else if (f(k) >= f_c(m + 1))
          {
            mel_filter_bank_(k, m) = 0.0;
          }
          else
          {
            ROS_ERROR("This should never happen. f_c(%i) = %f, f_k(%i) = %f.", m, f_c(m), k, f(k));
            ROS_BREAK();
          }
        }
      }
    }

    // log data
    //  std::ofstream outfile;
    //  outfile.open(std::string(std::string("/tmp/mel.txt")).c_str());
    //  outfile << mel_filter_bank_;
    //  outfile.close();

    return true;
  }

template<class MessageType>
  void FFTSignalProcessor<MessageType>::setupOutput()
  {
    for (int i = 0; i < num_output_signals_; ++i)
    {
      output_signal_spectrum_(i) = static_cast<double> (amplitude_spectrum_.transpose() * mel_filter_bank_.col(i));
      if (output_signal_spectrum_(i) < 10e-6)
      {
        output_signal_spectrum_(i) = 10e-6;
      }
      output_signal_spectrum_(i) = log(output_signal_spectrum_(i));
    }
  }

}

#endif /* FFT_TASK_SIGNAL_PROCESSOR_H_ */
