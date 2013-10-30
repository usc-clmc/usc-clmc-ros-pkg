/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      This code is a modified version of the ALSA example associated with the
 Linux Journal article by Jeff Tranter found here:
 http://www.linuxjournal.com/article/6735

 \file		audio_processor.cpp

 \author	Peter Pastor
 \date		Jun 10, 2011

 *********************************************************************/

// system includes
#include <math.h>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <ros/assert.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <boost/thread.hpp>

// local includes
#include <alsa_audio/audio_processor.h>

// Use the newer ALSA API
#define ALSA_PCM_NEW_HW_PARAMS_API

using namespace Eigen;

namespace alsa_audio
{

// static const int DUMP_RAW_AUDIO_BUFFER_SIZE = 1000000;

AudioProcessor::AudioProcessor(ros::NodeHandle node_handle) :
  initialized_(false), node_handle_(node_handle),
  pcm_handle_(NULL), hw_params_(NULL),
  num_frames_per_period_(0), num_new_frames_per_period_(0),
  num_new_bytes_per_period_(0), num_previous_bytes_read_(0),
  output_sample_rate_(0), num_channels_(0), timer_update_period_duration_(0.0), timer_update_rate_(0.0),
  desired_publishing_rate_(0.0), audio_buffer_size_(0), previous_audio_buffer_size_(0), current_audio_buffer_size_(0),
  max_device_audio_buffer_size_(0), num_received_frames_(0), apply_hamming_window_(false), apply_dct_(false),
  mel_filter_parameter_a_(0.0), mel_filter_parameter_b_(0.0), num_output_signals_(0), num_published_signals_(0),
  fftw_input_(NULL), fftw_out_(NULL), dctw_input_(NULL), dctw_output_(NULL), num_overlapping_frames_(0),
  min_freq_(1.0), max_freq_(100.0), received_first_frame_(false), frame_count_(0),
  publish_visualization_markers_(true), visualization_publishing_rate_(0.0),
  visualization_spectrum_max_amplitude_(0.0), visualization_spectrum_max_height_(0.0), spectrum_marker_width_(0.0),
  user_callback_enabled_(false)
{
}

AudioProcessor::~AudioProcessor()
{
  if(initialized_)
  {
    delete[] fftw_input_;
    fftw_free(fftw_out_);
    fftw_destroy_plan(fftw_plan_);
    int rc = snd_pcm_close(pcm_handle_);
    if (rc < 0)
    {
      ROS_ERROR("Unable to close handle : %s.", snd_strerror(rc) );
    }
  }
}

bool AudioProcessor::initialize()
{
  ROS_VERIFY(readParams());

  const int PUBLISHER_BUFFER_SIZE = 100;
  audio_sample_publisher_ = node_handle_.advertise<alsa_audio::AudioSample> ("audio_samples", PUBLISHER_BUFFER_SIZE);

  visualization_markers_.reset(new visualization_msgs::MarkerArray());
  // visualization_markers_->markers.resize(num_output_signals_);
  visualization_markers_->markers.resize(num_published_signals_);

  std::string visualization_marker_topic;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_marker_topic", visualization_marker_topic));
  visualization_marker_publisher_
      = node_handle_.advertise<visualization_msgs::MarkerArray> (visualization_marker_topic, PUBLISHER_BUFFER_SIZE);

  ROS_VERIFY(usc_utilities::read(node_handle_, "spectrum_base_frame_pose", spectrum_base_frame_pose_));

  double visualization_spectrum_length;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_spectrum_length", visualization_spectrum_length));
  ROS_ASSERT(visualization_spectrum_length > 0);
  // spectrum_marker_width_ = visualization_spectrum_length / (double)num_output_signals_;
  spectrum_marker_width_ = visualization_spectrum_length / (double)num_published_signals_;

  // for (int i = 0; i < num_output_signals_; ++i)
  for (unsigned int i = 0; i < num_published_signals_; ++i)
  {
    visualization_markers_->markers[i].header.frame_id = "/BASE";
    visualization_markers_->markers[i].ns.assign("spectrum_markers");
    visualization_markers_->markers[i].id = i;

    visualization_markers_->markers[i].type = visualization_msgs::Marker::CUBE;
    visualization_markers_->markers[i].action = visualization_msgs::Marker::ADD;

    spectrum_base_frame_pose_.position.x -= spectrum_marker_width_;
    visualization_markers_->markers[i].pose = spectrum_base_frame_pose_;

    visualization_markers_->markers[i].frame_locked = true;
  }

  audio_sample_.reset(new AudioSample());
  // audio_sample_->data.resize(num_output_signals_);
  audio_sample_->data.resize(num_published_signals_, 0.0);

  if(!initializeAudio())
  {
    ROS_ERROR("Failed to initialize audio processor. Publishing zeros at 100Hz instead.");
    update_timer_ = node_handle_.createTimer(ros::Duration(0.01), &AudioProcessor::updateCB, this);
    return (initialized_ = false);
  }

  output_signal_spectrum_ = Eigen::VectorXd::Zero(num_output_signals_);
  amplitude_spectrum_ = Eigen::VectorXd((int)num_frames_per_period_);

  fftw_input_ = new double[(int)num_frames_per_period_];
  for (unsigned int i = 0; i < num_frames_per_period_; ++i)
  {
    fftw_input_[i] = 0.0;
    fftw_input_[i] = 0.0;
  }
  fftw_out_ = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (int)num_frames_per_period_);
  for (unsigned int i = 0; i < num_frames_per_period_; ++i)
  {
    fftw_out_[i][0] = 0.0;
    fftw_out_[i][1] = 0.0;
  }

  // fftw_plan_ = fftw_plan_dft_r2c_1d((int)num_frames_per_period_, fftw_input_, fftw_out_, FFTW_ESTIMATE);
  fftw_plan_ = fftw_plan_dft_r2c_1d((int)num_frames_per_period_, fftw_input_, fftw_out_, FFTW_MEASURE);

  dctw_input_ = new double[(int)num_output_signals_];
  dctw_output_ = new double[(int)num_output_signals_];
  // dctw_plan_ = fftw_plan_r2r_1d((int)num_output_signals_, dctw_input_, dctw_output_, FFTW_REDFT10, FFTW_ESTIMATE);
  dctw_plan_ = fftw_plan_r2r_1d((int)num_output_signals_, dctw_input_, dctw_output_, FFTW_REDFT01, FFTW_MEASURE);

//  // Diagnostics
//  diagnostic_updater_.add("AudioProcessor Status", this, &AudioProcessor::diagnostics);
//  diagnostic_updater_.add(freq_status_);
//  diagnostic_updater_.setHardwareID("none");
//  diagnostic_updater_.force_update();

  // Timer

  bool own_callback = true;
  if(own_callback)
  {
    boost::thread(boost::bind(&AudioProcessor::callbackTimer, this));
  }
  else
  {
    ROS_DEBUG("Setting update timer period to >%.2f< ms, i.e. >%.2f< Hz.", timer_update_period_duration_ * 1000, timer_update_rate_);
    min_freq_ = 0.97 * timer_update_rate_;
    max_freq_ = 1.03 * timer_update_rate_;
    update_timer_ = node_handle_.createTimer(ros::Duration(timer_update_period_duration_), &AudioProcessor::updateCB,this);
  }

  // int16_t aux = 0;
  // cb_.reset(new CircularMessageBuffer<int16_t>(DUMP_RAW_AUDIO_BUFFER_SIZE, aux));
  // dump_raw_audio_service_server_  = node_handle_.advertiseService(std::string("dump_raw_audio"), &AudioProcessor::dumpRawAudio, this);
  // set_background_noise_service_server_ = node_handle_.advertiseService(std::string("set_background_noise"), &AudioProcessor::setBackgroundNoise, this);

  return (initialized_ = true);
}

void AudioProcessor::callbackTimer()
{
  ROS_DEBUG("Setting update timer period to >%.2f< ms, i.e. >%.2f< Hz.", timer_update_period_duration_ * 1000, timer_update_rate_);
  ros::Rate r(timer_update_rate_);
  const ros::TimerEvent timer_event;
  while (ros::ok())
  {
    updateCB(timer_event);
    r.sleep();
  }
}

bool AudioProcessor::initializeAudio()
{
  ROS_DEBUG("ALSA library version: %s", SND_LIB_VERSION_STR);

  // Open PCM device for recording (capture).
  // In order to find out if this is really true, you can run the command line:
  // $ arecord -l
  // This will give a listing of all the audio capture devices.
  int sound_card_id;
  ROS_VERIFY(usc_utilities::read(node_handle_, "sound_card_id", sound_card_id));
  int device_id;
  ROS_VERIFY(usc_utilities::read(node_handle_, "device_id", device_id));
  std::string hw_description;
  hw_description.assign("hw:" + usc_utilities::getString(sound_card_id) + "," + usc_utilities::getString(device_id));

  int rc = snd_pcm_open(&pcm_handle_, hw_description.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
  if (rc < 0)
  {
    ROS_ERROR("Unable to open pcm device : %s.", snd_strerror(rc));
    ROS_WARN("Maybe you need to change the card and device id.");
    ROS_WARN("Use \"sudo arecord -l\" to list sound cards and devices.");

    ROS_WARN("This would mean card id 2 and device id 0.");
    ROS_WARN("card 2: UA25EX [UA-25EX], device 0: USB Audio [USB Audio]");
    ROS_WARN("  Subdevices: 1/1");
    ROS_WARN("  Subdevice #0: subdevice #0");

    bool succeeded = false;
    for (unsigned int device_id = 0; !succeeded && device_id < 3; ++device_id)
    {
      ROS_WARN("Trying settings for mandy (card: >%i< device: >%i<).", sound_card_id, device_id);
      hw_description.assign("hw:" + usc_utilities::getString(sound_card_id) + "," + usc_utilities::getString(device_id));
      rc = snd_pcm_open(&pcm_handle_, hw_description.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
      if (rc < 0)
      {
        ROS_WARN("failed.");
      }
      else
      {
        ROS_INFO("succeeded with settings card: >%i< and device: >%i<.", sound_card_id, device_id);
        succeeded = true;
      }
    }
    if (!succeeded)
      return false;
  }
  //    sound_card_id = 1;
  //    device_id = 0;
  //    ROS_WARN("Trying default settings for mandy (card: >%i< device: >%i<).", sound_card_id, device_id);
  //    hw_description.assign("hw:" + usc_utilities::getString(sound_card_id) + "," + usc_utilities::getString(device_id));
  //    rc = snd_pcm_open(&pcm_handle_, hw_description.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
  //    if (rc < 0)
  //    {
  //      sound_card_id = 0;
  //      device_id = 1;
  //      ROS_WARN("Trying default settings for mandy (card: >%i< device: >%i<).", sound_card_id, device_id);
  //      hw_description.assign("hw:" + usc_utilities::getString(sound_card_id) + "," + usc_utilities::getString(device_id));
  //      rc = snd_pcm_open(&pcm_handle_, hw_description.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
  //      if (rc < 0)
  //      {
  //        ROS_ERROR("Unable to open pcm device %s.", snd_strerror(rc));
  //        return false;
  //      }
  //    }
  //  }
	
  // Allocate the hardware parameters object.
  snd_pcm_hw_params_alloca(&hw_params_);

  // Fill it in with default values.
  rc = snd_pcm_hw_params_any(pcm_handle_, hw_params_);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set any hw parameters : %s.", snd_strerror(rc) );
    return false;
  }

  // Set the desired hardware parameters.

  // Interleaved mode
  rc = snd_pcm_hw_params_set_access(pcm_handle_, hw_params_, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set access parameters : %s.", snd_strerror(rc) );
    return false;
  }

  // Signed 16-bit little-endian format
  rc = snd_pcm_hw_params_set_format(pcm_handle_, hw_params_, SND_PCM_FORMAT_S16_LE);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set format : %s.", snd_strerror(rc) );
    return false;
  }

  // Try to set sampling rate in bits/sec
  int dir = 0;
  // rc = snd_pcm_hw_params_set_rate_near(pcm_handle_, hw_params_, &output_sample_rate_, &dir);
  rc = snd_pcm_hw_params_set_rate(pcm_handle_, hw_params_, output_sample_rate_, dir);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set rate : %s.", snd_strerror(rc) );
    return false;
  }

  rc = snd_pcm_hw_params_set_channels(pcm_handle_, hw_params_, num_channels_);
  if (rc < 0)
  {
    if(num_channels_ == 2)
    {
      ROS_WARN("Stereo settings did not work. Falling back to mono.");
      num_channels_ = 1;
      rc = snd_pcm_hw_params_set_channels(pcm_handle_, hw_params_, num_channels_);
    }
    if (rc < 0)
    {
      ROS_ERROR("Unable to set channels : %s.", snd_strerror(rc) );
      return false;
    }
  }

  num_new_frames_per_period_ = num_frames_per_period_ - static_cast<snd_pcm_uframes_t> (num_overlapping_frames_);
  num_new_bytes_per_period_ = num_new_frames_per_period_ * 2 * num_channels_;
  num_previous_bytes_read_ = num_new_bytes_per_period_;

  // Try to set period size in number of frames.
  rc = snd_pcm_hw_params_set_period_size_near(pcm_handle_, hw_params_, &num_new_frames_per_period_, &dir);
  // rc = snd_pcm_hw_params_set_period_size(pcm_handle_, hw_params_, num_new_frames_per_period_, dir);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set period : %s.", snd_strerror(rc) );
    return false;
  }

  // Write the parameters to the driver
  rc = snd_pcm_hw_params(pcm_handle_, hw_params_);
  if (rc < 0)
  {
    ROS_ERROR("Unable to set hw parameters : %s.", snd_strerror(rc) );
    return false;
  }

  unsigned int num_channels;
  rc = snd_pcm_hw_params_get_channels(hw_params_, &num_channels);
  if (rc < 0)
  {
    ROS_ERROR("Unable to get channels : %s.", snd_strerror(rc) );
    return false;
  }
  ROS_DEBUG("Number of channels is >%i<.", (int)num_channels);
  if (num_channels != num_channels_)
  {
    ROS_ERROR("Number of channels is >%i<, but should be >%i<.", (int)num_channels, (int)num_channels_);
    return false;
  }

  unsigned int sampling_rate;
  rc = snd_pcm_hw_params_get_rate(hw_params_, &sampling_rate, &dir);
  if (rc < 0)
  {
    ROS_ERROR("Unable to get rate : %s.", snd_strerror(rc) );
    return false;
  }
  if (sampling_rate != output_sample_rate_)
  {
    ROS_ERROR("Desired output sampling rate is >%d<, but actual sampling rate is set to >%d<.",
        (int)sampling_rate, output_sample_rate_);
    return false;
  }
  ROS_DEBUG("Actual sampling rate is >%d< Hz.", sampling_rate);

  // Get what period size was actually set (may be different than desired)
  snd_pcm_uframes_t num_frames;
  rc = snd_pcm_hw_params_get_period_size(hw_params_, &num_frames, &dir);
  if (rc < 0)
  {
    ROS_ERROR("Unable to get period size : %s.", snd_strerror(rc) );
    return false;
  }
  ROS_DEBUG("Actual period size, in frames is >%d<.", (int)num_frames);
  if (num_new_frames_per_period_ != num_frames)
  {
    ROS_ERROR("Period size has been set to >%d< samples, but the actual period size is >%d<",
        (int)num_new_frames_per_period_, (int)num_frames);
    return false;
  }

  snd_pcm_format_t format;
  rc = snd_pcm_hw_params_get_format(hw_params_, &format);
  if (rc < 0)
  {
    ROS_ERROR("Could not retrieve format : %s.", snd_strerror(rc) );
    return false;
  }
  if (format != SND_PCM_FORMAT_S16_LE)
  {
    ROS_ERROR("Sound format should be SND_PCM_FORMAT_S16_LE >%i<, but it is >%i<.",
        (int)SND_PCM_FORMAT_S16_LE, (int)format);
    return false;
  }

  unsigned int period_time;
  snd_pcm_hw_params_get_period_time(hw_params_, &period_time, &dir);
  ROS_DEBUG("Period time is >%d< us.", (int)period_time);

  unsigned int buffer_time;
  snd_pcm_hw_params_get_buffer_time(hw_params_, &buffer_time, &dir);
  ROS_DEBUG("Buffer time is >%d< us.", buffer_time);

  unsigned int periods_per_buffer;
  snd_pcm_hw_params_get_periods(hw_params_, &periods_per_buffer, &dir);
  ROS_DEBUG("Periods per buffer is >%d< frames.", periods_per_buffer);

  unsigned int val, val2;
  snd_pcm_hw_params_get_rate_numden(hw_params_, &val, &val2);
  ROS_DEBUG("Exact rate is >%d/%d< bps.", val, val2);

  // convert period_time from us to s
  timer_update_period_duration_ = static_cast<double> (period_time) / static_cast<double> (1000.0 * 1000.0);
  ROS_ASSERT(timer_update_period_duration_ > 0);
  ROS_DEBUG("Timer update duration is >%f< seconds.", timer_update_period_duration_);
  //  double ratio = 1.0 - ((double)num_overlapping_frames_ / (double)num_frames_per_period_);
  //  ratio = 1.0;
  //  timer_update_period_duration_ *= ratio;
  //  ROS_INFO_COND(ratio < 1.0, "Reduced timer update duration is >%f< seconds.", timer_update_period_duration_);

  timer_update_rate_ = static_cast<double> (1.0) / timer_update_period_duration_;
  ROS_DEBUG("Timer update rate is >%.2f< Hz.", timer_update_rate_);

  if (apply_hamming_window_)
  {
    hamming_window_ = VectorXd::Zero((DenseIndex)num_frames_per_period_);
    for (unsigned int i = 0; i < num_frames_per_period_; ++i)
    {
      hamming_window_(i) = 0.54 - 0.46 * cos(static_cast<double> (2.0 * M_PI) * static_cast<double> (i)
          / static_cast<double> (num_frames_per_period_));
    }
    //  std::ofstream outfile;
    //  outfile.open(std::string(std::string("/tmp/ham.txt")).c_str());
    //  outfile << hamming_window_;
    //  outfile.close();
  }

  // TODO: check for 24bit and change the audio_buffer_size and everything else...

  audio_buffer_size_ = static_cast<int> (num_new_frames_per_period_) * 2 * static_cast<int> (num_channels_); // 2 bytes/sample, 2 channels
  audio_buffer_.resize(audio_buffer_size_);

  max_device_audio_buffer_size_ = 40 * audio_buffer_size_;
  max_device_audio_buffer_.resize(max_device_audio_buffer_size_);
  previous_max_device_audio_buffer_.resize(max_device_audio_buffer_size_, 0);
  ROS_DEBUG("Setting maximum audio buffer to >%i<.", max_device_audio_buffer_size_);

  // this buffer is needed for shuffling things around
  previous_audio_buffer_size_ = static_cast<int> (num_overlapping_frames_) * 2 * static_cast<int> (num_channels_); // 2 bytes/sample, 2 channels

  previous_audio_buffer_.resize(previous_audio_buffer_size_);
  ROS_DEBUG("Previous audio buffer is of size >%i<.", previous_audio_buffer_size_);

  // this buffer will always contain the last audio frame
  current_audio_buffer_size_ = static_cast<int> (num_frames_per_period_) * 2 * static_cast<int> (num_channels_); // 2 bytes/sample, 2 channels
  current_audio_buffer_.resize(current_audio_buffer_size_);

  // record_background_noise_ = false;
  // background_noise_audio_buffer_.resize(current_audio_buffer_size_, 0);

  return true;
}

int AudioProcessor::getNumOutputSignals() const
{
  // ROS_ASSERT(initialized_);
  ROS_WARN_COND(!initialized_, "Audio recorder is not initialized. Maybe initialization failed?");
  // return num_output_signals_;
  return (int)num_published_signals_;
}

//long readbuf(snd_pcm_t *handle, int8_t *buf, int len, snd_pcm_uframes_t *frames, int *max, int channels)
//{
//  long r;
//  int frame_bytes = (snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8) * channels;
//  do
//  {
//    r = snd_pcm_readi(handle, buf, len);
//    if (r > 0)
//    {
//      buf += r * frame_bytes;
//      len -= r;
//      *frames += r;
//      if ((long)*max < r)
//        *max = r;
//    }
//    // printf("r = %li, len = %li\n", r, len);
//  } while (r >= 1 && len > 0);
//  // showstat(handle, 0);
//  return r;
//}

void AudioProcessor::updateCB(const ros::TimerEvent& timer_event)
{
  if(!initialized_)
  {
    // publish zeros because audio processor was not or failed to initialize
    audio_sample_->header.stamp = ros::Time::now();
    audio_sample_->header.seq = frame_count_;
    frame_count_++;
    ROS_DEBUG_COND(frame_count_ % 100 == 0, "Publishing zeros because audio processor is either not initialized or failed to initialize.");
    // for (int i = 0; i < num_output_signals_; ++i)
    for (unsigned int i = 0; i < num_published_signals_; ++i)
    {
      audio_sample_->data[i] = 0.0;
    }
    audio_sample_publisher_.publish(audio_sample_);
    return;
  }

//  if (frame_count_ % 100 == 0)
//  {
//    ROS_INFO("ros::Time %f", ros::Time::now().toSec());
////    ROS_INFO("event::current_real::Time %f", timer_event.current_real.toSec());
////    ROS_INFO("event::current_expected::Time %f", timer_event.current_expected.toSec());
//  }


  // Make the call to capture the audio
  int pcm_rc = 0;
  int num_bytes_read = pcm_rc;
  while ((num_bytes_read < max_device_audio_buffer_size_)
      && (pcm_rc = snd_pcm_readi(pcm_handle_, &max_device_audio_buffer_[num_bytes_read], num_new_frames_per_period_)) > 0)
  {
    num_bytes_read += (pcm_rc * 2 * num_channels_);
  }
  ROS_WARN_COND(num_bytes_read >= max_device_audio_buffer_size_, "Stop reading from sound device. Buffer of >%i< bytes has been exceeded. This shouldn't matter though.", max_device_audio_buffer_size_);

//  int max = max_device_audio_buffer_size_;
//  snd_pcm_uframes_t frame = num_new_frames_per_period_;
//  unsigned int num_bytes_read = readbuf(pcm_handle_, &(max_device_audio_buffer_[0]), max_device_audio_buffer_size_, &frame, &max, num_channels_);

  if(num_bytes_read >= (int)num_new_bytes_per_period_) // we just read more than num_new_bytes_per_period_ frames
  {
    for (unsigned int i = 0; i < num_new_bytes_per_period_; ++i)
    {
      audio_buffer_[i] = max_device_audio_buffer_[i];
    }
  }
  else // we just read less than num_new_bytes_per_period_ frames and have to reuse previous ones
  {
    const unsigned int REUSE_BYTES = num_new_bytes_per_period_ - num_bytes_read;
    // ROS_INFO("we just read less than num_new_bytes_per_period_ frames and have to reuse previous ones %i", reuse_bytes);
    for (unsigned int i = 0; i < REUSE_BYTES; ++i)
    {
      audio_buffer_[i] = previous_max_device_audio_buffer_[num_bytes_read + i];
    }
    for (unsigned int i = REUSE_BYTES; i < num_new_bytes_per_period_; ++i)
    {
      audio_buffer_[i] = max_device_audio_buffer_[i - REUSE_BYTES];
    }

//    for (int i = 0; i < num_new_bytes_per_period_; ++i)
//    {
//      if(i <= reuse_bytes)
//      {
////        ROS_INFO("%i ) Reusing >%i< bytes starting at >%i<. Size is %i.",
////                 i, reuse_bytes, num_previous_bytes_read_ - reuse_bytes,
////                 (int)previous_max_device_audio_buffer_.size());
//        // audio_buffer_[i] = previous_max_device_audio_buffer_[num_previous_bytes_read_ - reuse_bytes + i];
//        audio_buffer_[i] = previous_max_device_audio_buffer_[num_bytes_read + i];
//      }
//      else // we just read less than num_new_frames_per_period_ frames and have to reuse previous ones
//      {
//        audio_buffer_[i] = max_device_audio_buffer_[i - reuse_bytes];
//      }
//    }
  }

  // TODO: For now, ignore the first cycle and hope that the sound buffer has enough data
  previous_max_device_audio_buffer_ = max_device_audio_buffer_;
  num_previous_bytes_read_ = num_bytes_read;

  now_time_ = ros::Time::now(); // try to grab as close to getting message as possible

//  current_real_ = timer_event.current_real;
//  current_expected_ = timer_event.current_expected;
//  last_real_ = timer_event.last_real;
//  last_expected_ = timer_event.last_expected;

  if (pcm_rc == -EPIPE)
  {
    // EPIPE means overrun
    ROS_WARN("Overrun occurred.");
    snd_pcm_prepare(pcm_handle_);
    // consequtively_dropped_frames_++;
  }
//  else if (pcm_rc < 0)
//  {
//    ROS_ERROR("Could not read sound device: %s.", snd_strerror(pcm_rc));
//    ROS_BREAK();
//  }
//  else if (pcm_rc != (int)num_new_frames_per_period_)
//  {
//    ROS_WARN("Number of frames read >%d< is wrong. Missed >%i< frames.", pcm_rc, (int)num_new_frames_per_period_ - pcm_rc);
//    consequtively_dropped_frames_++;
//  }
  else
  {
    // freq_status_.tick();
    if (received_first_frame_) // after the first frame has been received
    {
      // max_period_between_updates_ = std::max(max_period_between_updates_, (timer_event.current_real - timer_event.last_real).toSec());
      // last_callback_duration_ = timer_event.profile.last_duration.toSec();
      setupBuffer();
      ROS_VERIFY(processFrame());
      bufferPreviousFrames();
    }
    else // only called when the very first frame was received
    {
      start_time_ = now_time_;
      // visualization_marker_lifetime_ = visualization_publishing_dt_;
      visualization_marker_lifetime_ = visualization_publishing_dt_ + ros::Duration(0.05);
      previous_visualization_marker_publishing_time_ = now_time_;
      // copy entire audio buffer
      for (int i = 0; i < audio_buffer_size_; ++i)
      {
        if (num_received_frames_ + i < (int)current_audio_buffer_.size())
        {
          current_audio_buffer_[num_received_frames_ + i] = audio_buffer_[i];
        }
      }
      num_received_frames_ += audio_buffer_size_;

    }
    if (received_first_frame_ || (num_received_frames_ >= current_audio_buffer_size_))
    {
      received_first_frame_ = true;
    }
    // consequtively_dropped_frames_ = 0;
  }

  //  if (consequtively_dropped_frames_ > max_dropped_frames_)
  //  {
  //    ROS_ERROR("Missed >%i< consequtive frames, which is more than allowed >%i<.", consequtively_dropped_frames_, max_dropped_frames_);
  //    ROS_BREAK();
  //  }

  // store raw audio
//  cb_mutex_.lock();
//  if (num_channels_ == 1) // mono
//  {
//    const int num_16bit_samples = num_bytes_read / 2;
//    for (int i = 0; i < num_16bit_samples; ++i)
//    {
//      int16_t mono = *((int16_t*)(&max_device_audio_buffer_[i * 2]));
//      cb_->push_back(mono);
//    }
//  }
//  else // stereo
//  {
//    const int num_32bit_samples = num_bytes_read / 4;
//    for (int i = 0; i < num_32bit_samples; ++i)
//    {
//      int16_t left = *((int16_t*)(&max_device_audio_buffer_[i * 4]));
//      int16_t right = *((int16_t*)(&max_device_audio_buffer_[i * 4 + 2]));
//      cb_->push_back(left);
//      cb_->push_back(right);
//      // int32_t left_32 = left;
//      // int32_t right_32 = right;
//      // int16_t mono = (int16_t) ((left_32 + right_32) / 2);
//      // cb_->push_back(mono);
//    }
//  }
//  cb_mutex_.unlock();

  //  background_noise_mutex_.lock();
  //  if(record_background_noise_)
  //  {
  //    background_noise_audio_buffer_ = current_audio_buffer_;
  //    record_background_noise_ = false;
  //  }
  //  background_noise_mutex_.unlock();

  frame_count_++;
//  diagnostic_updater_.update();
}

void AudioProcessor::setupBuffer()
{
  int current_audio_buffer_index = current_audio_buffer_size_ - audio_buffer_size_;
  for (int i = 0; i < audio_buffer_size_; ++i)
  {
    current_audio_buffer_[current_audio_buffer_index + i] = audio_buffer_[i];
  }
}

void AudioProcessor::bufferPreviousFrames()
{
  int audio_buffer_index = current_audio_buffer_size_ - previous_audio_buffer_size_;
  for (int i = 0; i < previous_audio_buffer_size_; ++i)
  {
    previous_audio_buffer_[i] = current_audio_buffer_[audio_buffer_index + i];
  }
  for (int i = 0; i < previous_audio_buffer_size_; ++i)
  {
    current_audio_buffer_[i] = previous_audio_buffer_[i];
  }
}

bool AudioProcessor::processFrame()
{

  for (unsigned int i = 0; i < num_output_signals_; ++i)
  {
    output_signal_spectrum_(i) = 0.0;
  }

//  for(int i=0; i<(int)current_audio_buffer_.size(); ++i)
//  {
//    current_audio_buffer_[i] = current_audio_buffer_[i] + background_noise_audio_buffer_[i];
//  }

  // set input
  if (num_channels_ == 1) // mono
  {
    for (int i = 0; i < (int)num_frames_per_period_; ++i)
    {
      int16_t mono = *((int16_t*)(&current_audio_buffer_[i * 2]));
      fftw_input_[i] = ((double)mono) / 65536.0;
    }
  }
  else // stereo
  {
    for (int i = 0; i < (int)num_frames_per_period_; ++i)
    {
      int16_t left = *((int16_t*)(&current_audio_buffer_[i * 4]));
      int16_t right = *((int16_t*)(&current_audio_buffer_[i * 4 + 2]));
      fftw_input_[i] = ((double)left + (double)right) / 65536.0;
    }
  }

  // apply hamming window
  if (apply_hamming_window_)
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

  // compute cosine transform
  if (apply_dct_)
  {
    for (unsigned int i = 0; i < num_output_signals_; ++i)
    {
      dctw_input_[i] = output_signal_spectrum_(i);
    }
    fftw_execute(dctw_plan_);
    for (unsigned int i = 0; i < num_output_signals_; ++i)
    {
      output_signal_spectrum_(i) = dctw_output_[i];
    }
  }
  else
  {
    for (unsigned int i = 0; i < num_output_signals_; ++i)
    {
      dctw_input_[i] = output_signal_spectrum_(i);
    }
    fftw_execute(dctw_plan_);
    output_signal_spectrum_(num_output_signals_ - 1) = dctw_output_[0];
    output_signal_spectrum_(num_output_signals_ - 2) = dctw_output_[1];
    output_signal_spectrum_(num_output_signals_ - 3) = dctw_output_[2];
    output_signal_spectrum_(num_output_signals_ - 4) = dctw_output_[3];
  }

  // scale
  scaleOutput();

  if (publish_visualization_markers_)
  {
    if ((now_time_ - previous_visualization_marker_publishing_time_) > visualization_publishing_dt_)
    {
      publishMarkers();
      visualization_marker_lifetime_ = (now_time_ - previous_visualization_marker_publishing_time_) + ros::Duration(0.05);
      previous_visualization_marker_publishing_time_ = now_time_;
    }
  }

  // publish

  audio_sample_->header.stamp = now_time_;
  audio_sample_->header.seq = frame_count_;
  for (unsigned int i = published_output_index_range_[0]; i < published_output_index_range_[1]; ++i)
  {
    audio_sample_->data[i - published_output_index_range_[0]] = output_signal_spectrum_(i);
  }
  audio_sample_publisher_.publish(audio_sample_);

//  boost::mutex::scoped_lock lock(callback_mutex_);
//  if (recording_ && user_callback_enabled_)
//  {
//    user_callback_(audio_sample_);
//  }
  return true;
}

bool AudioProcessor::initMelFilterBank()
{
  // from "Mel Frequency Cepstral Coefficients: An Evaluation of Robustness of MP3 Encoded Music"
  // Authors: Sigurdur Sigurdsson and Kaare Brandt Petersen and Tue Lehn-Schiøler

  mel_filter_bank_ = MatrixXd::Zero((DenseIndex)num_frames_per_period_, (DenseIndex)num_output_signals_);

  double bandwidth = static_cast<double> (output_sample_rate_) / 2.0;
  double frequency_step = bandwidth / static_cast<double> (mel_filter_bank_.rows());
  VectorXd f = VectorXd::Zero(mel_filter_bank_.rows() + 1);
  for (int k = 0; k < (int)f.size(); ++k)
  {
    f(k) = static_cast<double> (k) * frequency_step;
  }

  //  std::ofstream outfile_f;
  //  outfile_f.open(std::string(std::string("/tmp/f.txt")).c_str());
  //  outfile_f << f;
  //  outfile_f.close();

  double f_max = static_cast<double> (output_sample_rate_) / 2.0;
  double phi_max = mel_filter_parameter_b_ * log10((f_max / mel_filter_parameter_a_) + 1.0);
  double phi_min = 0.0;

  double mel_frequency_step = (phi_max - phi_min) / static_cast<double> (mel_filter_bank_.cols() - 1);
  VectorXd phi_c = VectorXd::Zero(mel_filter_bank_.cols() + 1);
  for (int m = 0; m < (int)phi_c.size(); ++m)
  {
    phi_c(m) = static_cast<double> (m) * mel_frequency_step;
  }
  // ROS_INFO("Max MEL frequency is >%f< Hz.", phi_c(phi_c.size()-1));

  //  std::ofstream outfile_phi_c;
  //  outfile_phi_c.open(std::string(std::string("/tmp/phi_c.txt")).c_str());
  //  outfile_phi_c << phi_c;
  //  outfile_phi_c.close();

  VectorXd f_c = VectorXd::Zero(phi_c.size());
  for (int i = 0; i < (int)f_c.size(); ++i)
  {
    f_c(i) = mel_filter_parameter_a_ * (pow(10.0, (phi_c(i) / mel_filter_parameter_b_)) - 1.0);
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

void AudioProcessor::setupOutput()
{
  for (unsigned int i = 0; i < num_output_signals_; ++i)
  {
//    if (amplitude_spectrum_.size() != mel_filter_bank_.col(i).size())
//      ROS_ERROR("Invalid size amplitude_spectrum_.size() %i != %i  mel_filter_bank_.col(i).size()",
//                (int)amplitude_spectrum_.size(), (int)mel_filter_bank_.col(i).size());
    output_signal_spectrum_(i) = static_cast<double> (amplitude_spectrum_.transpose() * mel_filter_bank_.col(i));
    if (output_signal_spectrum_(i) < 10e-6)
    {
      output_signal_spectrum_(i) = 10e-6;
    }
    output_signal_spectrum_(i) = log(output_signal_spectrum_(i));
  }
}

void AudioProcessor::scaleOutput()
{

  // ROS_INFO_STREAM("BEFORE:" << output_signal_spectrum_.transpose());
  for (unsigned int i = 0; i < num_output_signals_; ++i)
  {
    // output_signal_spectrum_(i) = fabs(output_signal_spectrum_(i) / output_scaling_(i));
    output_signal_spectrum_(i) = fabs(output_signal_spectrum_(i) / output_scaling_(i)) + output_offset_(i);
  }
  // output_signal_spectrum_ = (output_signal_spectrum_.array() / output_scaling_.array()).matrix().
  // ROS_INFO_STREAM("AFTER: " << output_signal_spectrum_.transpose());

}

//bool AudioProcessor::registerCallback(boost::function<void(const AudioSample::ConstPtr& audio_sample_msg)> callback)
//{
//  boost::mutex::scoped_lock lock(callback_mutex_);
//  if (user_callback_enabled_)
//  {
//    ROS_ERROR("User callback already enabled. Only one callback can be registered.");
//    return false;
//  }
//  user_callback_ = callback;
//  user_callback_enabled_ = true;
//  return true;
//}

//bool AudioProcessor::startRecording()
//{
//  // boost::mutex::scoped_lock lock(callback_mutex_);
//  if (recording_)
//  {
//    ROS_ERROR("Already recording audio... not doing anything.");
//    return false;
//  }
//  recording_ = true;
//  return true;
//}
//
//bool AudioProcessor::stopRecording()
//{
//  // boost::mutex::scoped_lock lock(callback_mutex_);
//  if (!recording_)
//  {
//    ROS_ERROR("Not recording audio... not doing anything.");
//    return false;
//  }
//  recording_ = false;
//  return true;
//}

void AudioProcessor::publishMarkers()
{
  // for (int i = 0; i < num_output_signals_; ++i)
  for (unsigned int i = 0; i < num_published_signals_; ++i)
  {
    visualization_markers_->markers[i].header.stamp = now_time_;
    visualization_markers_->markers[i].header.seq = frame_count_;
    visualization_markers_->markers[i].lifetime = visualization_marker_lifetime_;

    visualization_markers_->markers[i].scale.x = spectrum_marker_width_;
    visualization_markers_->markers[i].scale.y = spectrum_marker_width_;

    // double normalized_height = output_signal_spectrum_(i) / visualization_spectrum_max_amplitude_;
    double normalized_height = output_signal_spectrum_(published_output_index_range_[0] + i);
    // ROS_WARN("normalized_height: %f", normalized_height);
    if (fabs(normalized_height) > 1.0) // overshoot
    {
      visualization_markers_->markers[i].color.r = 0.0;
      visualization_markers_->markers[i].color.g = 0.0;
      visualization_markers_->markers[i].color.b = 1.0;
      visualization_markers_->markers[i].color.a = 1.0;
    }
    else
    {
      if (fabs(normalized_height) < 10e-6)
      {
        normalized_height = 10e-6;
      }
      visualization_markers_->markers[i].color.r = normalized_height;
      visualization_markers_->markers[i].color.g = 1.0 - normalized_height;
      visualization_markers_->markers[i].color.b = 0.0;
      visualization_markers_->markers[i].color.a = 1.0;
    }
    double marker_height = normalized_height * visualization_spectrum_max_height_;
    visualization_markers_->markers[i].scale.z = marker_height;
    visualization_markers_->markers[i].pose.position.z = spectrum_base_frame_pose_.position.z + (marker_height / 2.0);
  }
  visualization_marker_publisher_.publish(*visualization_markers_);
}

bool AudioProcessor::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "apply_hamming_window", apply_hamming_window_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "output_sample_rate", output_sample_rate_));
  ROS_ASSERT(output_sample_rate_ > 0);
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_channels", num_channels_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "num_frames_per_period", num_frames_per_period_));
  ROS_ASSERT(num_frames_per_period_ > 0);
  ROS_ASSERT(isPowerOfTwo(num_frames_per_period_));

  // num_frames_per_period_ = static_cast<snd_pcm_uframes_t>(output_sample_rate_ / 2);

  ROS_VERIFY(usc_utilities::read(node_handle_, "num_output_signals", num_output_signals_));
  ROS_ASSERT(num_output_signals_ > 0);
  ROS_ASSERT(num_frames_per_period_ >= (2*num_output_signals_));
  //  if ((num_frames_per_period_ % num_output_signals_) != 0)
  //  {
  //    ROS_ERROR("Number of output signals >%i< must be a devisor of number of frames per period >%i<.",
  //        num_output_signals_, (int)num_frames_per_period_);
  //    return false;
  //  }
  //  num_signals_per_bin_ = num_frames_per_period_ / (2*num_output_signals_);
  //  ROS_INFO("There are >%i< signals per bin.", num_signals_per_bin_);

  ROS_VERIFY(usc_utilities::read(node_handle_, "published_output_index_range", published_output_index_range_));
  ROS_ASSERT(published_output_index_range_.size() == 2);
  ROS_ASSERT(published_output_index_range_[1] > published_output_index_range_[0]);
  ROS_ASSERT(published_output_index_range_[1] <= num_output_signals_);
  num_published_signals_ = published_output_index_range_[1] - published_output_index_range_[0];

  ROS_VERIFY(usc_utilities::read(node_handle_, "desired_publishing_rate", desired_publishing_rate_));
  ROS_ASSERT(desired_publishing_rate_ > 1.0);
  ROS_ASSERT(desired_publishing_rate_ < 1000.0);

  //  ROS_VERIFY(usc_utilities::read(node_handle_, "num_overlapping_frames", num_overlapping_frames_));
  //  ROS_ASSERT(num_overlapping_frames_ < (int)num_frames_per_period_);
  //  ROS_ASSERT(num_overlapping_frames_ >= 0);

  int num_overlapping_frames = static_cast<int> (num_frames_per_period_) - ceil(static_cast<double> (output_sample_rate_) / desired_publishing_rate_);
  num_overlapping_frames_ = 0;
  if (num_overlapping_frames > 0)
    num_overlapping_frames_ = num_overlapping_frames;

  ROS_DEBUG("Number of overlapping frames is >%i<.", num_overlapping_frames_);

  ROS_VERIFY(usc_utilities::read(node_handle_, "publish_visualization_markers", publish_visualization_markers_));

  double visualization_publishing_rate;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_publishing_rate", visualization_publishing_rate));
  ROS_ASSERT(visualization_publishing_rate > 0.0);
  ROS_ASSERT(visualization_publishing_rate < 100.0);
  ROS_DEBUG("Visualization happens at >%2.1f< Hz.", visualization_publishing_rate);
  visualization_publishing_dt_ = ros::Duration(static_cast<double> (1.0) / visualization_publishing_rate);
  ROS_DEBUG("Visualization marker lifetime is >%f< sec.", visualization_publishing_dt_.toSec());

  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_spectrum_max_amplitude", visualization_spectrum_max_amplitude_));
  ROS_ASSERT(visualization_spectrum_max_amplitude_ > 0);
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_spectrum_max_height", visualization_spectrum_max_height_));
  ROS_ASSERT(visualization_spectrum_max_height_ > 0);

  ROS_VERIFY(usc_utilities::read(node_handle_, "apply_dct", apply_dct_));

  ROS_VERIFY(usc_utilities::read(node_handle_, "mel_filter_parameter_a", mel_filter_parameter_a_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "mel_filter_parameter_b", mel_filter_parameter_b_));
  ROS_VERIFY(initMelFilterBank());

  std::vector<double> output_scaling;
  if(apply_dct_)
  {
    ROS_VERIFY(usc_utilities::read(node_handle_, "dct_output_scaling", output_scaling));
  }
  else
  {
    ROS_VERIFY(usc_utilities::read(node_handle_, "no_dct_output_scaling", output_scaling));
  }
  output_scaling_ = Eigen::VectorXd::Map(&output_scaling[0], output_scaling.size());
  ROS_ASSERT_MSG(output_scaling_.size() == num_output_signals_, "Invalid number of scaling parameters >%i<. There should be >%i<.",
                 (int)output_scaling_.size(), (int)num_output_signals_);

  std::vector<double> output_offset;
  if(apply_dct_)
  {
    ROS_VERIFY(usc_utilities::read(node_handle_, "dct_output_offset", output_offset));
  }
  else
  {
    ROS_VERIFY(usc_utilities::read(node_handle_, "no_dct_output_offset", output_offset));
  }
  output_offset_ = Eigen::VectorXd::Map(&output_offset[0], output_offset.size());
  ROS_ASSERT_MSG(output_offset_.size() == num_output_signals_, "Invalid number of offset parameters >%i<. There should be >%i<.",
                 (int)output_offset_.size(), (int)num_output_signals_);


  return true;
}

bool AudioProcessor::isPowerOfTwo(unsigned int x)
{
  return ((x != 0) && ((x & (~x + 1)) == x));
}

//void AudioProcessor::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
//{
//  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
//  stat.add("Maximum period between updates [sec]", max_period_between_updates_);
//  stat.add("Latest callback runtime [sec]       ", last_callback_duration_);
//
//  stat.add("Now time [sec]        ", (now_time_ - start_time_).toSec());
//  stat.add("Current expected [sec]", (current_expected_ - start_time_).toSec());
//  stat.add("Current real [sec]    ", (current_real_ - start_time_).toSec());
//  stat.add("Last expected [sec]   ", (last_expected_ - start_time_).toSec());
//  stat.add("Last real [sec]       ", (last_real_ - start_time_).toSec());
//
//  stat.add("Latest audio frame number", last_frame_number_);
//  stat.add("Dropped frames count     ", dropped_frame_count_);
//  stat.add("Frame count              ", frame_count_);
//}

//bool AudioProcessor::setBackgroundNoise(alsa_audio::SetBackgroundNoise::Request& request,
//                                        alsa_audio::SetBackgroundNoise::Response& response)
//{
//  background_noise_mutex_.lock();
//  record_background_noise_ = true;
//  background_noise_mutex_.unlock();
//  response.result = SetBackgroundNoise::Response::SUCCEEDED;
//  return true;
//}

//bool AudioProcessor::dumpRawAudio(alsa_audio::DumpRawAudio::Request& request,
//                                  alsa_audio::DumpRawAudio::Response& response)
//{
//  std::string filename = request.abs_file_name;
//  if(filename.empty())
//  {
//    filename.assign("/tmp/raw_audio.bin");
//  }
//
//  std::ofstream audio_file(filename.c_str(), std::ios::out | std::ios::binary);
//  if(!audio_file.is_open())
//  {
//    ROS_ERROR("Problems when opening file >%s<.", filename.c_str());
//    response.result = DumpRawAudio::Response::FAILED;
//    return true;
//  }
//
//  std::vector<int16_t> copy(DUMP_RAW_AUDIO_BUFFER_SIZE, 0);
//
//  cb_mutex_.lock();
//  ROS_WARN_COND((int)cb_->size() != DUMP_RAW_AUDIO_BUFFER_SIZE, "Audio buffer not filed yet.");
//  ROS_VERIFY(cb_->get(copy));
//  cb_mutex_.unlock();
//
//  audio_file.seekp(std::ios::beg);
//  for (int i = 0; i < (int)copy.size(); ++i)
//  {
//    audio_file.write((char*)&(copy[i]), sizeof(copy[i]));
//  }
//  audio_file.close();
//
//  response.result = DumpRawAudio::Response::SUCCEEDED;
//  return true;
//}

}
