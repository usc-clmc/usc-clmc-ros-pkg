/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_fft_signal_processor_node.cpp

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

// system includes
#include <ros/ros.h>
#include <vector>
#include <cmath>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>

#include <std_msgs/Float64.h>

// local includes
#include <task_signal_processor/fft_signal_processor.h>

using namespace std;
using namespace task_signal_processor;

const int NUM_FRAMES_PER_PERIOD = 200;
const int NUM_OUTPUT_SIGNALS = 100;
const double LOOP_FREQUENCY = 100.0;
const double SIN_FREQUENCY = 25.0 * 2.0 * M_PI;
const double SIN_FREQUENCY2 = 10.0 * 2.0 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestFFTSignalProcessor");
  ros::NodeHandle node_handle("~");

  ros::Publisher signal_publisher = node_handle.advertise<std_msgs::Float64>("signal", 1000);

  FFTSignalProcessor fft_signal_processor;
  ROS_VERIFY(fft_signal_processor.initialize(NUM_FRAMES_PER_PERIOD, NUM_OUTPUT_SIGNALS, LOOP_FREQUENCY));

  std::vector<double> data;
  data.resize(NUM_OUTPUT_SIGNALS);

  ros::Rate rate(LOOP_FREQUENCY);
  while (ros::ok())
  {
    double value = std::sin(SIN_FREQUENCY * ros::Time::now().toSec()) + std::sin(SIN_FREQUENCY2 * ros::Time::now().toSec());
    std_msgs::Float64 msg;
    msg.data = value;
    signal_publisher.publish(msg);
    ROS_VERIFY(fft_signal_processor.filter(value, data));
    for (int i = 0; i < NUM_OUTPUT_SIGNALS/2; ++i)
    {
      std::cout << "(" << i << ")" << data[i] << " ";
    }
    std::cout << std::endl;
    rate.sleep();
  }

  return 0;
}
