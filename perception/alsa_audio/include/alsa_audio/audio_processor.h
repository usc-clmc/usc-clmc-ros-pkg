/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks      This code is a modified version of the ALSA example associated with the
                Linux Journal article by Jeff Tranter found here:
                http://www.linuxjournal.com/article/6735
 
  \file		audio_processor.h

  \author	Peter Pastor
  \date		Jun 10, 2011

 *********************************************************************/

// This code is a modified version of the ALSA example associated with the
// Linux Journal article by Jeff Tranter found here:
// http://www.linuxjournal.com/article/6735

#ifndef AUDIO_PROCESSOR_H_
#define AUDIO_PROCESSOR_H_

// system includes
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
// #include <boost/function.hpp>

#include <alsa/asoundlib.h>
#include <fftw3.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

// local includes
#include <alsa_audio/AudioSample.h>
#include <alsa_audio/DumpRawAudio.h>
#include <alsa_audio/SetBackgroundNoise.h>
#include <alsa_audio/circular_message_buffer.h>

namespace alsa_audio
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
  virtual ~AudioProcessor();

  /*!
   * @return True on success, otherwise False
   */
  bool initialize();

//  /*!
//   * Register a callback function which will be called every time a new frame is processed are updated
//   * @param callback
//   * @return True on success, otherwise False
//   */
//  bool registerCallback(boost::function<void (const alsa_audio::AudioSample::ConstPtr& audio_sample_msg)> callback);

//  /*!
//   * @return True on success, otherwise False
//   */
//  bool startRecording();
//  /*!
//   * @return True on success, otherwise False
//   */
//  bool stopRecording();

  /*!
   * @return
   */
  int getNumOutputSignals() const;

//  /*!
//   * @param request
//   * @param response
//   * @return
//   */
//  bool dumpRawAudio(alsa_audio::DumpRawAudio::Request& request,
//                    alsa_audio::DumpRawAudio::Response& response);

  /*!
   * @param request
   * @param response
   * @return
   */
  //  bool setBackgroundNoise(alsa_audio::SetBackgroundNoise::Request& request,
  //                          alsa_audio::SetBackgroundNoise::Response& response);

private:

  void callbackTimer();

  bool initialized_;
  ros::NodeHandle node_handle_;
  ros::Publisher audio_sample_publisher_;
  ros::Publisher visualization_marker_publisher_;

  // Information that ALSA uses to do the capture
  snd_pcm_t* pcm_handle_;
  snd_pcm_hw_params_t* hw_params_;
  snd_pcm_uframes_t num_frames_per_period_; // In number of frames
  snd_pcm_uframes_t num_new_frames_per_period_; // In number of frames

  int num_new_bytes_per_period_;
  int num_previous_bytes_read_;

  unsigned int output_sample_rate_;
  unsigned int num_channels_;
  double timer_update_period_duration_;
  double timer_update_rate_;

  double desired_publishing_rate_;

  std::vector<int8_t> audio_buffer_;
  int audio_buffer_size_;
  std::vector<int8_t> previous_audio_buffer_;
  int previous_audio_buffer_size_;
  std::vector<int8_t> current_audio_buffer_;
  int current_audio_buffer_size_;

  std::vector<int8_t> max_device_audio_buffer_;
  std::vector<int8_t> previous_max_device_audio_buffer_;
  int max_device_audio_buffer_size_;

  int num_received_frames_;

  Eigen::VectorXd output_signal_spectrum_;
  Eigen::VectorXd amplitude_spectrum_;
  Eigen::VectorXd hamming_window_;
  bool apply_hamming_window_;
  bool apply_dct_;

  Eigen::MatrixXd mel_filter_bank_;
  double mel_filter_parameter_a_;
  double mel_filter_parameter_b_;

  int num_output_signals_;
  std::vector<unsigned int> published_output_index_range_;
  unsigned int num_published_signals_;
  // int num_signals_per_bin_;

  double* fftw_input_;
  fftw_complex *fftw_out_;
  fftw_plan fftw_plan_;

  double* dctw_input_;
  double* dctw_output_;
  fftw_plan dctw_plan_;

  int num_overlapping_frames_;

  Eigen::VectorXd output_scaling_;
  Eigen::VectorXd output_offset_;

  // Processing the audio signal
  void updateCB(const ros::TimerEvent& timer_event);
  bool processFrame();

  // Timer
  ros::Timer update_timer_;
  ros::Time now_time_;

  ros::Time start_time_;

//  ros::Time current_real_;
//  ros::Time current_expected_;
//  ros::Time last_real_;
//  ros::Time last_expected_;

  // Diagnostic Updater
//  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
//  diagnostic_updater::Updater diagnostic_updater_;
  double min_freq_;
  double max_freq_;
//  diagnostic_updater::FrequencyStatus freq_status_;
  bool received_first_frame_;
//  double max_period_between_updates_;
//  double last_callback_duration_;
//  unsigned int last_frame_number_;
  unsigned int frame_count_;
//  unsigned int dropped_frame_count_;
//  int max_dropped_frames_;
//  int consequtively_dropped_frames_;

  // Helper functions
  bool readParams();
  static bool isPowerOfTwo (unsigned int x);
  bool initializeAudio();
  void bufferPreviousFrames();
  void setupBuffer();

  void setupOutput();
  void scaleOutput();
  bool initMelFilterBank();

  // Publising visualization markers
  void publishMarkers();
  bool publish_visualization_markers_;
  ros::Duration visualization_publishing_dt_;
  ros::Duration visualization_marker_lifetime_;
  ros::Time previous_visualization_marker_publishing_time_;
  double visualization_publishing_rate_;
  double visualization_spectrum_max_amplitude_;
  double visualization_spectrum_max_height_;
  double spectrum_marker_width_;
  geometry_msgs::Pose spectrum_base_frame_pose_;
  boost::scoped_ptr<visualization_msgs::MarkerArray> visualization_markers_;

  bool user_callback_enabled_;
  // bool recording_;
  boost::shared_ptr<AudioSample> audio_sample_;
  // boost::function<void (const AudioSample::ConstPtr& msg)> user_callback_;
  // boost::mutex callback_mutex_;

//  ros::ServiceServer dump_raw_audio_service_server_;
//  boost::shared_ptr<CircularMessageBuffer<int16_t> > cb_;
//  boost::mutex cb_mutex_;

  //  ros::ServiceServer set_background_noise_service_server_;
  //  bool record_background_noise_;
  //  std::vector<int8_t> background_noise_audio_buffer_;
  //  boost::mutex background_noise_mutex_;

};

}

#endif /* AUDIO_PROCESSOR_H_ */
