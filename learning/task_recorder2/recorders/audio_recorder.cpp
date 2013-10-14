/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		audio_recorder.cpp

  \author	Peter Pastor
  \date		Jun 13, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>


// local includes
#include <task_recorder2_recorders/audio_recorder.h>

namespace task_recorder2_recorders
{

AudioRecorder::AudioRecorder(ros::NodeHandle node_handle) :
  TaskRecorder<alsa_audio::AudioSample> (node_handle), audio_processor_(ros::NodeHandle("/AudioProcessor"))
{
  if(!audio_processor_.initialize())
  {
    ROS_ERROR("Could not initialize audio recorder.");
    ROS_ERROR("Trying to continue anyway.");
  }
  num_signals_ = audio_processor_.getNumOutputSignals();
}

bool AudioRecorder::transformMsg(const alsa_audio::AudioSample& audio_sample,
                                 task_recorder2_msgs::DataSample& data_sample)
{
  data_sample.header = audio_sample.header;
  data_sample.data = audio_sample.data;
  return true;
}

std::vector<std::string> AudioRecorder::getNames() const
{
  // ROS_ASSERT_MSG(initialized_, "AudioRecorder is not initialize.");
  std::vector<std::string> names;
  for (int i = 0; i < num_signals_; ++i)
  {
    names.push_back(std::string("audio_") + usc_utilities::getString(i));
  }
  return names;
}

}
