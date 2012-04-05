/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		audio_processor.h

  \author	Peter Pastor
  \date		Jun 10, 2011

 *********************************************************************/

#ifndef AUDIO_PROCESSOR_H_
#define AUDIO_PROCESSOR_H_

// system includes
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <string>
#include <boost/scoped_ptr.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

#include <fmodex/fmod.hpp>
#include <fmodex/fmod_errors.h>

// local includes

namespace fmod_audio
{

class AudioProcessor
{

public:

  /*!  Constructor
   * @param node_handle
   * @return
   */
  AudioProcessor(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~AudioProcessor()
  {
    delete[] spectrum_;
  };

  /*!
   * @return
   */
  bool initialize();

private:

  ros::NodeHandle node_handle_;
  ros::Publisher visualization_marker_publisher_;

  FMOD::System* system_;
  FMOD::Sound* sound_;
  FMOD::Channel* channel_;

  // Some defines
  FMOD_SOUND_FORMAT sound_format_;
  FMOD_DSP_RESAMPLER resampler_;
  FMOD_DSP_FFT_WINDOW fft_method_;

  FMOD_RESULT result_;
  FMOD_CREATESOUNDEXINFO exinfo_;

  // make is shared
  float* spectrum_;
  Eigen::VectorXd frequency_histogram_;

  // Timer
  ros::Timer update_timer_;
  double update_rate_;
  ros::Time now_time_;
  ros::Time last_time_;

  // Diagnostic Updater
  diagnostic_updater::Updater diagnostic_updater_;
  double min_freq_;
  double max_freq_;
  diagnostic_updater::FrequencyStatus freq_status_;
  bool received_first_frame_;
  double max_period_between_updates_;
  double last_callback_duration_;
  unsigned int last_frame_number_;
  unsigned int frame_count_;
  unsigned int dropped_frame_count_;
  int max_dropped_frames_;
  int consequtively_dropped_frames_;

  /*! Helper functions
   */
  bool readParams();
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateCB(const ros::TimerEvent& timer_event);
  void checkForErrors(FMOD_RESULT result);
  bool processFrame();
  bool shutdown();

  bool publish_visualization_markers_;
  void publishMarkers();
  int visualization_publishing_ratio_;
  double visualization_publishing_rate_;
  double visualization_spectrum_height_ratio_;
  double spectrum_marker_width_;
  int num_frequencies_bins_;
  int number_of_frequency_bins_;
  int num_frequencies_per_bin_;
  geometry_msgs::Pose spectrum_base_frame_pose_;

  int output_sample_rate_;
  int spectrum_size_;
  float bin_size_;
  float spectrum_range_;

  boost::scoped_ptr<visualization_msgs::MarkerArray> visualization_markers_;

  static bool isPowerOfTwo (unsigned int x);
};

}

#endif /* AUDIO_PROCESSOR_H_ */
