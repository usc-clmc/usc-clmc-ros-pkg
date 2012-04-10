/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		audio_processor.cpp

  \author	Peter Pastor
  \date		Jun 10, 2011

 *********************************************************************/

// system includes
#include <math.h>
#include <string>
#include <ros/assert.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <fmod_audio/audio_processor.h>

namespace fmod_audio
{

AudioProcessor::AudioProcessor(ros::NodeHandle node_handle)
  : node_handle_(node_handle), diagnostic_updater_(), min_freq_(1.0), max_freq_(100.0),
    freq_status_(diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_)),
    received_first_frame_(false), max_dropped_frames_(10), consequtively_dropped_frames_(0)
{

}

bool AudioProcessor::initialize()
{
  ROS_VERIFY(readParams());

  visualization_markers_.reset(new visualization_msgs::MarkerArray());
  visualization_markers_->markers.resize(number_of_frequency_bins_);

  std::string visualization_marker_topic;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_marker_topic", visualization_marker_topic));
  visualization_marker_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic, 100);

  ROS_VERIFY(usc_utilities::read(node_handle_, "spectrum_base_frame_pose", spectrum_base_frame_pose_));

  double visualization_spectrum_length;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_spectrum_length", visualization_spectrum_length));
  ROS_ASSERT(visualization_spectrum_length > 0);
  spectrum_marker_width_ = visualization_spectrum_length / number_of_frequency_bins_;

  for (int i = 0; i < number_of_frequency_bins_; ++i)
  {
    visualization_markers_->markers[i].header.frame_id = "/BASE";
    visualization_markers_->markers[i].ns.assign("spectrum_markers");
    visualization_markers_->markers[i].id = i;

    visualization_markers_->markers[i].type = visualization_msgs::Marker::CUBE;
    visualization_markers_->markers[i].action = visualization_msgs::Marker::ADD;

    spectrum_base_frame_pose_.position.x -= spectrum_marker_width_;
    visualization_markers_->markers[i].pose = spectrum_base_frame_pose_;

    visualization_markers_->markers[i].frame_locked = true;
    visualization_markers_->markers[i].lifetime = ros::Duration(1.0/visualization_publishing_rate_);
  }


  frequency_histogram_ = Eigen::VectorXd(number_of_frequency_bins_);
  frequency_histogram_.setZero(static_cast<Eigen::DenseIndex>(number_of_frequency_bins_));

  // Some enums defined in fmod.hpp
  sound_format_ = FMOD_SOUND_FORMAT_PCM16;
  resampler_ = FMOD_DSP_RESAMPLER_LINEAR;
  fft_method_ = FMOD_DSP_FFT_WINDOW_TRIANGLE;

  // Create a System object and initialize.
  result_ = FMOD::System_Create(&system_);
  checkForErrors(result_);

  unsigned int version;
  result_ = system_->getVersion(&version);
  checkForErrors(result_);
  if (version < FMOD_VERSION)
  {
    ROS_ERROR("You are using an old version of FMOD %08x. This program requires %08x.", version, FMOD_VERSION);
    return false;
  }

  result_ = system_->setOutput(FMOD_OUTPUTTYPE_ALSA);
  checkForErrors(result_);

  int numdrivers;
  result_ = system_->getNumDrivers(&numdrivers);
  checkForErrors(result_);

  ROS_DEBUG("Available playback drivers:");
  for (int count = 0; count < numdrivers; count++)
  {
    char name[256];
    result_ = system_->getDriverInfo(count, name, 256, 0);
    checkForErrors(result_);
    ROS_DEBUG("%d : %s", count + 1, name);
  }

  int playback_driver;
  ROS_VERIFY(usc_utilities::read(node_handle_, "playback_driver", playback_driver));
  result_ = system_->setDriver(playback_driver);
  checkForErrors(result_);

  result_ = system_->getRecordNumDrivers(&numdrivers);
  checkForErrors(result_);

  ROS_DEBUG("Available record drivers:");
  for (int count = 0; count < numdrivers; count++)
  {
    char name[256];
    result_ = system_->getRecordDriverInfo(count, name, 256, 0);
    checkForErrors(result_);
    ROS_DEBUG("%d : %s", count + 1, name);
  }

  int record_driver;
  ROS_VERIFY(usc_utilities::read(node_handle_, "record_driver", record_driver));
  result_ = system_->setDriver(record_driver);
  checkForErrors(result_);

  result_ = system_->setSoftwareFormat(output_sample_rate_, sound_format_, 1, 0, resampler_);
  checkForErrors(result_);

  result_ = system_->init(32, FMOD_INIT_NORMAL, 0);
  checkForErrors(result_);

  int outputfreq;
  system_->getSoftwareFormat(&outputfreq, 0, 0, 0, 0, 0);
  checkForErrors(result_);

  // Create a sound to record to.
  memset(&exinfo_, 0, sizeof(FMOD_CREATESOUNDEXINFO));
  exinfo_.cbsize = sizeof(FMOD_CREATESOUNDEXINFO);
  exinfo_.numchannels = 1;
  exinfo_.format = sound_format_;
  exinfo_.defaultfrequency = output_sample_rate_;
  exinfo_.length = exinfo_.defaultfrequency * sizeof(short) * exinfo_.numchannels * 5;

  result_ = system_->createSound(0, FMOD_2D | FMOD_SOFTWARE | FMOD_LOOP_NORMAL | FMOD_OPENUSER, &exinfo_, &sound_);
  checkForErrors(result_);

  result_ = system_->recordStart(record_driver, sound_, true);
  checkForErrors(result_);

//  Give it some time to record something
  ros::Duration(1.0).sleep();

  result_ = system_->playSound(FMOD_CHANNEL_REUSE, sound_, false, &channel_);
  checkForErrors(result_);

  // Dont hear what is being recorded otherwise it will feedback.
  // Spectrum analysis is done before volume scaling in the DSP chain
  result_ = channel_->setVolume(0);
  checkForErrors(result_);

  // Diagnostics
  diagnostic_updater_.add("AudioProcessor Status", this, &AudioProcessor::diagnostics);
  diagnostic_updater_.add(freq_status_);
  diagnostic_updater_.setHardwareID("none");
  diagnostic_updater_.force_update();

  // Timer
  double update_timer_period = 1.0 / update_rate_;
  min_freq_ = 0.95 * update_rate_;
  max_freq_ = 1.05 * update_rate_;
  update_timer_ = node_handle_.createTimer(ros::Duration(update_timer_period), &AudioProcessor::updateCB, this);

  return true;
}

void AudioProcessor::updateCB(const ros::TimerEvent& timer_event)
{

  result_ = channel_->getSpectrum(spectrum_, spectrum_size_, 0, fft_method_);
  if (result_ == FMOD_OK)
  {
    now_time_ = ros::Time::now(); // try to grab as close to getting message as possible
    freq_status_.tick();
    if (received_first_frame_)
    {
      max_period_between_updates_ = std::max(max_period_between_updates_, (timer_event.current_real - timer_event.last_real).toSec());
      last_callback_duration_ = timer_event.profile.last_duration.toSec();
    }
    received_first_frame_ = true;
    consequtively_dropped_frames_ = 0;
    processFrame();
  }
  else
  {
    ROS_ERROR("Could not get spectrum. (%d) : %s.", result_, FMOD_ErrorString(result_));
    consequtively_dropped_frames_++;
    if(consequtively_dropped_frames_ > max_dropped_frames_)
    {
      shutdown();
    }
  }

  frame_count_++;
  system_->update();
  diagnostic_updater_.update();
}

bool AudioProcessor::processFrame()
{

  // TODO: check whether this reallocates memory
  frequency_histogram_.setZero(static_cast<Eigen::DenseIndex>(number_of_frequency_bins_));

  for (int i = 0; i < num_frequencies_bins_; i++)
  {
    for (int j = 0; j < num_frequencies_per_bin_; ++j)
    {
      frequency_histogram_(i) = frequency_histogram_(i) + spectrum_[i * num_frequencies_per_bin_ + j];
    }
  }

  if(publish_visualization_markers_ && (frame_count_ % visualization_publishing_ratio_) == 0)
  {
    publishMarkers();
  }
  return true;
}

void AudioProcessor::publishMarkers()
{
  for (int i = 0; i < number_of_frequency_bins_; ++i)
  {
    visualization_markers_->markers[i].header.stamp = now_time_;
    visualization_markers_->markers[i].header.seq = frame_count_;

    visualization_markers_->markers[i].scale.x = spectrum_marker_width_;
    visualization_markers_->markers[i].scale.y = spectrum_marker_width_;
    double height = static_cast<double>(frequency_histogram_[i] / visualization_spectrum_height_ratio_);
    if(height < 0.001)
    {
      height = 0.001;
    }
    visualization_markers_->markers[i].scale.z = height;
    visualization_markers_->markers[i].pose.position.z = spectrum_base_frame_pose_.position.z + (height/2.0);

    visualization_markers_->markers[i].color.r = height;
    visualization_markers_->markers[i].color.g = visualization_spectrum_height_ratio_ - height;
    visualization_markers_->markers[i].color.b = 0.0;
    visualization_markers_->markers[i].color.a = 1.0;
  }
  visualization_marker_publisher_.publish(*visualization_markers_);
}

bool AudioProcessor::shutdown()
{
  ROS_INFO("Shutting down audio processing...");
  result_ = sound_->release();
  checkForErrors(result_);
  result_ = system_->release();
  checkForErrors(result_);
  return true;
}

bool AudioProcessor::readParams()
{
  ROS_VERIFY(usc_utilities::read(node_handle_, "output_sample_rate", output_sample_rate_));
  ROS_VERIFY(usc_utilities::read(node_handle_, "spectrum_size", spectrum_size_));
  spectrum_range_ = ((float)output_sample_rate_ / 2.0f);
  bin_size_ = spectrum_range_ / ((float)spectrum_size_);

  ROS_INFO("spectrum_size_ is %i.", spectrum_size_);
  ROS_INFO("spectrum_range_ is %f.", spectrum_range_);
  ROS_INFO("bin_size_ is %f.", bin_size_);

  spectrum_ = new float[spectrum_size_];

  ROS_VERIFY(usc_utilities::read(node_handle_, "update_rate", update_rate_));
  ROS_ASSERT(update_rate_ > 0);
  ROS_VERIFY(usc_utilities::read(node_handle_, "number_of_frequency_bins", number_of_frequency_bins_));
  ROS_ASSERT(number_of_frequency_bins_ > 0);
  ROS_ASSERT(isPowerOfTwo(number_of_frequency_bins_));
  ROS_ASSERT((spectrum_size_ % number_of_frequency_bins_) == 0);
  num_frequencies_bins_ = spectrum_size_ / number_of_frequency_bins_;

  num_frequencies_per_bin_ = static_cast<int>(spectrum_size_ / number_of_frequency_bins_);
  ROS_INFO("num_frequencies_per_bin_ is %i.", num_frequencies_per_bin_);

  ROS_VERIFY(usc_utilities::read(node_handle_, "publish_visualization_markers", publish_visualization_markers_));

  double visualization_marker_publishing_factor;
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_marker_publishing_factor", visualization_marker_publishing_factor));
  ROS_ASSERT(visualization_marker_publishing_factor > 0);
  ROS_ASSERT(visualization_marker_publishing_factor <= 1.0);
  visualization_publishing_rate_ = update_rate_ * visualization_marker_publishing_factor;
  ROS_INFO("Visualization happens at >%2.1f< Hz.", visualization_publishing_rate_);
  visualization_publishing_ratio_ = static_cast<int>(update_rate_ / visualization_publishing_rate_);
  ROS_VERIFY(usc_utilities::read(node_handle_, "visualization_spectrum_height_ratio", visualization_spectrum_height_ratio_));
  ROS_ASSERT(visualization_spectrum_height_ratio_ > 0);

  return true;
}

bool AudioProcessor::isPowerOfTwo(unsigned int x)
{
  return ((x != 0) && ((x & (~x + 1)) == x));
}

void AudioProcessor::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  stat.add("max period between updates", max_period_between_updates_);
  stat.add("latest callback runtime", last_callback_duration_);
  stat.add("latest VICON frame number", last_frame_number_);
  stat.add("dropped frames", dropped_frame_count_);
  stat.add("framecount", frame_count_);
}

void AudioProcessor::checkForErrors(FMOD_RESULT result)
{
  if (result != FMOD_OK)
  {
    ROS_ERROR("FMOD error! (%d) %s", result, FMOD_ErrorString(result));
    ROS_BREAK();
  }
}

}
