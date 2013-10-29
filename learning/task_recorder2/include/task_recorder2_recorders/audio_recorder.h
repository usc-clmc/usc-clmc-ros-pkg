/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		audio_recorder.h

  \author	Peter Pastor
  \date		Jun 13, 2011

 *********************************************************************/

#ifndef AUDIO_RECORDER_H_
#define AUDIO_RECORDER_H_

// system includes
#include <vector>
#include <string>

#include <task_recorder2_msgs/DataSample.h>

#include <alsa_audio/audio_processor.h>
#include <alsa_audio/AudioSample.h>

// local includes
#include <task_recorder2/task_recorder.h>

namespace task_recorder2_recorders
{

class AudioRecorder : public task_recorder2::TaskRecorder<alsa_audio::AudioSample>
{

public:

  /*! Constructor
   * @param node_handle
   */
  AudioRecorder(ros::NodeHandle node_handle);
  /*! Destructor
   */
  virtual ~AudioRecorder() {};

  /*!
   * @return True on success, otherwise False
   */
  bool initialize(const std::string topic_name = std::string("/AudioProcessor/audio_samples"))
  {
    return task_recorder2::TaskRecorder<alsa_audio::AudioSample>::initialize(topic_name);
  }

  /*!
   * @param audio_sample
   * @param data_sample
   * @return True on success, otherwise False
   */
  bool transformMsg(const alsa_audio::AudioSample& audio_sample,
                    task_recorder2_msgs::DataSample& data_sample);

  /*!
   * @return
   */
  int getNumSignals() const
  {
    return num_signals_;
  }

  /*!
   * @return
   */
  std::vector<std::string> getNames() const;

  /*!
   * @return
   */
  static std::string getClassName()
  {
    return std::string("AudioRecorder");
  }

private:

  int num_signals_;
  alsa_audio::AudioProcessor audio_processor_;

};


}


#endif /* AUDIO_RECORDER_H_ */
