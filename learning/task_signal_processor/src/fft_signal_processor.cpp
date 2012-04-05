/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		fft_signal_processor.cpp

 \author	Peter Pastor
 \date		Jul 22, 2011

 *********************************************************************/

// system includes

// local includes
#include <task_signal_processor/fft_signal_processor.h>

namespace task_signal_processor
{

FFTSignalProcessor::FFTSignalProcessor()
  : initialized_(false)
{
}

FFTSignalProcessor::~FFTSignalProcessor()
{
  if(initialized_)
  {
    delete[] fftw_input_;
    fftw_free(fftw_out_);
    fftw_destroy_plan(fftw_plan_);
  }
}

bool FFTSignalProcessor::initialize(const int num_frames_per_period,
                                    const int num_output_signals,
                                    const double output_sample_rate,
                                    bool apply_window,
                                    const double mel_filter_parameter_a,
                                    const double mel_filter_parameter_b)
{
  num_frames_per_period_ = num_frames_per_period;
  num_output_signals_ = num_output_signals;
  output_sample_rate_ = output_sample_rate;
  apply_window_ = apply_window;
  ROS_VERIFY(initMelFilterBank(mel_filter_parameter_a, mel_filter_parameter_b));

  fftw_input_ = new double[num_frames_per_period_];
  fftw_out_ = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * num_frames_per_period_);
  fftw_plan_ = fftw_plan_dft_r2c_1d(num_frames_per_period_, fftw_input_, fftw_out_, FFTW_MEASURE);

  current_data_.resize(num_frames_per_period_);
  output_signal_spectrum_ = Eigen::VectorXd((Eigen::DenseIndex)num_output_signals_);
  amplitude_spectrum_ = Eigen::VectorXd((Eigen::DenseIndex)num_frames_per_period_);
  if(apply_window_)
  {
    hamming_window_ = Eigen::VectorXd::Zero((Eigen::DenseIndex)num_frames_per_period_);
    for (int i = 0; i < (int)num_frames_per_period_; ++i)
    {
      hamming_window_(i) = 0.54 - 0.46 * cos(static_cast<double> (2.0 * M_PI) * static_cast<double> (i) / static_cast<double> (num_frames_per_period_));
    }
  }

  double default_value = 0.0;
  cb_.reset(new task_recorder2_utilities::CircularMessageBuffer<double>(num_frames_per_period_, default_value));

  return (initialized_ = true);
}

bool FFTSignalProcessor::filter(const double value, std::vector<double>& data)
{
  ROS_ASSERT((int)data.size() == num_output_signals_);

  cb_->push_front(value);
  ROS_VERIFY(cb_->get(current_data_));

  for (int i = 0; i < num_frames_per_period_; ++i)
  {
    fftw_input_[i] = current_data_[i];
  }

  // apply hamming window
  if (apply_window_)
  {
    for (int i = 0; i < (int)num_frames_per_period_; ++i)
    {
      fftw_input_[i] = fftw_input_[i] * hamming_window_(i);
    }
  }

  // compute DFT
  fftw_execute(fftw_plan_);

  // compute amplitude
  for (int i = 0; i < (int)num_frames_per_period_; ++i)
  {
    amplitude_spectrum_(i) = sqrt(fftw_out_[i][0] * fftw_out_[i][0] + fftw_out_[i][1] * fftw_out_[i][1]);
  }

  // fill in the output signal
  setupOutput();

  // write to data
  for (int i = 0; i < num_output_signals_; ++i)
  {
		if(isnan(output_signal_spectrum_(i)))
		{
			data[i] = 0.0;
		}
		else
		{
			data[i] = output_signal_spectrum_(i);
		}
  }
  return true;
}

void FFTSignalProcessor::setupOutput()
{
  for (int i = 0; i < num_output_signals_; ++i)
  {
    output_signal_spectrum_(i) = static_cast<double> (amplitude_spectrum_.transpose() * mel_filter_bank_.col(i));
  }
}

bool FFTSignalProcessor::initMelFilterBank(const double a,
                                           const double b)
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
  ROS_DEBUG("Max MEL frequency is >%f< Hz.", phi_c(phi_c.size()-1));

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
	// std::ofstream outfile;
	// outfile.open(std::string(std::string("/tmp/mel.txt")).c_str());
	// outfile << mel_filter_bank_;
	// outfile.close();

  return true;
}

}

