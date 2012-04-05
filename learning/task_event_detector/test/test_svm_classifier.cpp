/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_svm_classifier.cpp

  \author	Peter Pastor
  \date		Jun 21, 2011

 *********************************************************************/

// system includes
// #include <shogun/lib/Mathematics.h>
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/logging.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

// local includes
#include <task_event_detector/svm_classifier.h>
#include <task_event_detector/shogun_init.h>

using namespace task_event_detector;
using namespace shogun;

void generateData(const int num_samples,
                  const int num_variables,
                  const std::vector<double> distances,
                  const int label,
                  std::vector<task_recorder2_msgs::DataSample>& data_samples,
                  std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels)
{
  ROS_VERIFY((int)distances.size() == num_variables);

  for (int i = 0; i < num_samples; ++i)
  {
    task_recorder2_msgs::DataSample data_sample;
    task_recorder2_msgs::DataSampleLabel data_sample_label;
    data_sample_label.type = task_recorder2_msgs::DataSampleLabel::BINARY_LABEL;
//    if (i % 2)
//    {
//      data_sample_label.binary_label.label = task_recorder2_msgs::BinaryLabel::SUCCEEDED;
//    }
//    else
//    {
//      data_sample_label.binary_label.label = task_recorder2_msgs::BinaryLabel::FAILED;
//    }
    data_sample_label.binary_label.label = label;
    for (int j = 0; j < num_variables; ++j)
    {
      data_sample.names.push_back(std::string("test_variable_") + usc_utilities::getString(j));
      data_sample.data.push_back(CMath::random(0.0, 1.0) + ((double)data_sample_label.binary_label.label * distances[j]));
    }
    data_samples.push_back(data_sample);
    data_sample_labels.push_back(data_sample_label);
  }
}

void generateData(const int num_samples,
                  const int num_variables,
                  const std::vector<std::vector<double> > distances,
                  const std::vector<int> labels,
                  std::vector<task_recorder2_msgs::DataSample>& data_samples,
                  std::vector<task_recorder2_msgs::DataSampleLabel>& data_sample_labels)
{
  ROS_VERIFY(distances.size() == labels.size());
  for (int i = 0; i < (int)distances.size(); ++i)
  {
    generateData(num_samples, num_variables, distances[i], labels[i], data_samples, data_sample_labels);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TestSVMClassifier");
  ros::NodeHandle node_handle("~");

  task_event_detector::init();

  SVMClassifier svm_classifier;
  ROS_VERIFY(svm_classifier.read(node_handle));

  bool generate_data = true;
  int num_variables = 2;
  int num_training_samples = 100;
  std::vector<std::vector<double> > all_training_distances;
  std::vector<double> training_distances(2, 0.0);
  const double DISTANCE = 0.5;
  training_distances[0] = DISTANCE;
  training_distances[1] = DISTANCE;
  all_training_distances.push_back(training_distances);
  training_distances[0] = -DISTANCE;
  training_distances[1] = DISTANCE;
  all_training_distances.push_back(training_distances);
  training_distances[0] = DISTANCE;
  training_distances[1] = -DISTANCE;
  all_training_distances.push_back(training_distances);
  training_distances[0] = -DISTANCE;
  training_distances[1] = -DISTANCE;
  all_training_distances.push_back(training_distances);

  std::vector<int> training_labels(all_training_distances.size(), 0);
  training_labels[0] = task_recorder2_msgs::BinaryLabel::SUCCEEDED;
  training_labels[1] = task_recorder2_msgs::BinaryLabel::FAILED;
  training_labels[2] = task_recorder2_msgs::BinaryLabel::FAILED;
  training_labels[3] = task_recorder2_msgs::BinaryLabel::SUCCEEDED;

  int num_test_samples = 200;
  std::vector<std::vector<double> > all_test_distances;
  std::vector<double> test_distances(2, 0.0);
  test_distances[0] = DISTANCE;
  test_distances[1] = DISTANCE;
  all_test_distances.push_back(test_distances);
  test_distances[0] = -DISTANCE;
  test_distances[1] = DISTANCE;
  all_test_distances.push_back(test_distances);
  test_distances[0] = DISTANCE;
  test_distances[1] = -DISTANCE;
  all_test_distances.push_back(test_distances);
  test_distances[0] = -DISTANCE;
  test_distances[1] = -DISTANCE;
  all_test_distances.push_back(test_distances);

  std::vector<int> test_labels(all_test_distances.size(), 0);
  test_labels[0] = task_recorder2_msgs::BinaryLabel::SUCCEEDED;
  test_labels[1] = task_recorder2_msgs::BinaryLabel::FAILED;
  test_labels[2] = task_recorder2_msgs::BinaryLabel::FAILED;
  test_labels[3] = task_recorder2_msgs::BinaryLabel::SUCCEEDED;

  // generate training data
  std::vector<task_recorder2_msgs::DataSample> training_data_samples;
  std::vector<task_recorder2_msgs::DataSampleLabel> training_data_sample_labels;
  if(generate_data)
  {
    ROS_INFO("Generating new training data.");
    generateData(num_training_samples, num_variables, all_training_distances, training_labels, training_data_samples, training_data_sample_labels);
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::writeToBagFile(training_data_samples, "/data_samples", "/tmp/training_samples.bag"));
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSampleLabel>::writeToBagFile(training_data_sample_labels, "/data_sample_labels", "/tmp/training_sample_labels.bag"));
  }
  else
  {
    ROS_INFO("Loading training data.");
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::readFromBagFile(training_data_samples, "/data_samples", "/tmp/training_samples.bag"));
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSampleLabel>::readFromBagFile(training_data_sample_labels, "/data_sample_labels", "/tmp/training_sample_labels.bag"));
  }

  // adding data
  ROS_INFO("Adding data to SVM.");
  ROS_VERIFY(svm_classifier.addTrainingData(training_data_samples, training_data_sample_labels));

  // training
  ROS_INFO("Training SVM.");
  ROS_VERIFY(svm_classifier.train());

  ROS_INFO("Save SVM.");
  ROS_VERIFY(svm_classifier.save("/tmp"));

  ROS_INFO("Load SVM.");
  SVMClassifier svm_classifier_copy;
  ROS_VERIFY(svm_classifier_copy.load("/tmp"));

  ROS_INFO("Generating test data.");

  // generate test data
  std::vector<task_recorder2_msgs::DataSample> test_data_samples;
  std::vector<task_recorder2_msgs::DataSampleLabel> true_test_data_sample_labels;
  if(generate_data)
  {
    generateData(num_test_samples, num_variables, all_test_distances, test_labels,test_data_samples, true_test_data_sample_labels);
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::writeToBagFile(test_data_samples, "/data_samples", "/tmp/test_samples.bag"));
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSampleLabel>::writeToBagFile(true_test_data_sample_labels, "/data_sample_labels", "/tmp/test_sample_labels.bag"));
  }
  else
  {
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSample>::readFromBagFile(test_data_samples, "/data_samples", "/tmp/test_samples.bag"));
    ROS_VERIFY(usc_utilities::FileIO<task_recorder2_msgs::DataSampleLabel>::readFromBagFile(true_test_data_sample_labels, "/data_sample_labels", "/tmp/test_sample_labels.bag"));
  }

  ROS_ERROR_COND(!task_event_detector::SVM_LOGGING_ENABLED, "SVM_LOGGING_ENABLED define in svm_classifier.cpp not set.");
  ROS_INFO("Getting predictions.");

  // get predictions
  std::vector<task_recorder2_msgs::DataSampleLabel> predicted_data_sample_labels;
  ROS_VERIFY(svm_classifier_copy.predict(test_data_samples, predicted_data_sample_labels));//, test_data_samples[0].names));

  ROS_INFO("Compute number of false positives.");

  // compute predicition error
  ROS_ASSERT(predicted_data_sample_labels.size() == test_data_samples.size());
  int num_false_possitive = 0;
  int num_false_negative = 0;
  int num_true_possitive = 0;
  int num_true_negative = 0;
  for (int i = 0; i < (int)predicted_data_sample_labels.size(); ++i)
  {
    ROS_ASSERT(predicted_data_sample_labels[i].type == task_recorder2_msgs::DataSampleLabel::BINARY_LABEL);

    if(predicted_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED
        && true_test_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED)
    {
      num_true_possitive++;
    }
    else if(predicted_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED
        && true_test_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED)
    {
      num_true_negative++;
    }
    else if(predicted_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED
        && true_test_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED)
    {
      num_false_possitive++;
    }
    else if(predicted_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::FAILED
          && true_test_data_sample_labels[i].binary_label.label == task_recorder2_msgs::BinaryLabel::SUCCEEDED)
    {
      num_false_negative++;
    }
    else
    {
      ROS_ERROR("This case should never happen !!!");
      return -1;
    }

  }

  ROS_INFO("There have been >%i< true possitives.", num_true_possitive);
  ROS_INFO("There have been >%i< true negatives.", num_true_negative);
  ROS_INFO("There have been >%i< false possitives.", num_false_possitive);
  ROS_INFO("There have been >%i< false negatives.", num_false_negative);
  ROS_INFO("There have been >%i< correct predictions and >%i< incorrect predictions.",
      num_true_possitive+num_true_negative, num_false_possitive+num_false_negative);

  task_event_detector::exit();
  return 0;
}
