/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		data_sample_utilities.h

  \author	Peter Pastor
  \date		Jun 8, 2011

 *********************************************************************/

#ifndef DATA_SAMPLE_UTILITIES_H_
#define DATA_SAMPLE_UTILITIES_H_

// system includes
#include <ros/ros.h>
#include <string>
#include <vector>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <task_recorder2_msgs/DataSample.h>

// local includes

namespace task_recorder2_utilities
{

/*!
 * @param complete_data_sample
 * @param names
 * @param extracted_data_sample
 * @return True on success, otherwise False
 */
inline bool extractDataSample(const task_recorder2_msgs::DataSample& complete_data_sample,
                              const std::vector<std::string>& names,
                              task_recorder2_msgs::DataSample& extracted_data_sample);

/*!
 * @param complete_data_sample
 * @param name
 * @param extracted_data_sample
 * @return True on success, otherwise False
 */
inline bool extractDataSample(const task_recorder2_msgs::DataSample& complete_data_sample,
                              const std::string& name,
                              task_recorder2_msgs::DataSample& extracted_data_sample);

/*!
 * @param complete_data_samples
 * @param names
 * @param extracted_data_samples
 * @return True on success, otherwise False
 */
inline bool extractDataSamples(const std::vector<task_recorder2_msgs::DataSample>& complete_data_samples,
                               const std::vector<std::string>& names,
                               std::vector<task_recorder2_msgs::DataSample>& extracted_data_samples);

/*!
 * @param complete_data_samples
 * @param name
 * @param extracted_data_samples
 * @return True on success, otherwise False
 */
inline bool extractDataSamples(const std::vector<task_recorder2_msgs::DataSample>& complete_data_samples,
                               const std::string& name,
                               std::vector<task_recorder2_msgs::DataSample>& extracted_data_samples);

/*!
 * @param all_names
 * @param subset_names
 * @param indices
 * @return True on success, otherwise False
 */
inline bool getIndices(const std::vector<std::string>& all_names,
                       const std::vector<std::string>& subset_names,
                       std::vector<int>& indices);

/*!
 * @param all_names
 * @param subset_names
 * @param verbose
 * @return True on success, otherwise False
 */
inline bool isContained(const std::vector<std::string>& all_names,
                        const std::vector<std::string>& subset_names,
                        bool verbose = true);
inline bool isContained(const std::vector<std::string>& all_names, const std::string& subset_name)
{
  std::vector<std::string> subset_names;
  subset_names.push_back(subset_name);
  return isContained(all_names, subset_names);
}

/*!
 * @param all_names
 * @param subset_names
 * @param verbose
 * @return True on success, otherwise False
 */
inline bool haveSameContent(const std::vector<std::string>& all_names,
                            const std::vector<std::string>& subset_names,
                            bool verbose = true);

// inline functions follow

bool getIndices(const std::vector<std::string>& all_names,
                const std::vector<std::string>& subset_names,
                std::vector<int>& indices)
{
  if(all_names.empty())
  {
    ROS_ERROR("There are no names provided.");
    return false;
  }
  if(all_names.size() < subset_names.size())
  {
    ROS_ERROR("There are fewer names in all_names >%i< than in subset_names >%i<.", (int)all_names.size(), (int)subset_names.size());
    return false;
  }

  indices.clear();
  for (std::vector<std::string>::const_iterator it = subset_names.begin(); it != subset_names.end(); ++it)
  {
    bool found = false;
    for (int i = 0; i < (int)all_names.size() && !found; ++i)
    {
      if(it->compare(all_names[i]) == 0)
      {
        indices.push_back(i);
        found = true;
      }
    }
    if(!found)
    {
      ROS_ERROR("Could not find variable named >%s<.", it->c_str());
      ROS_ERROR("Contained variables are:");
      for (int i = 0; i < (int)all_names.size() && !found; ++i)
      {
        ROS_ERROR(">%s<.", all_names[i].c_str());
      }
      return false;
    }
  }
  return true;
}

bool isContained(const std::vector<std::string>& all_names,
                 const std::vector<std::string>& subset_names,
                 bool verbose)
{
  if(subset_names.size() > all_names.size())
  {
    ROS_ERROR_COND(verbose, "Size of subset >%i< is greater than size of set >%i<.", (int)subset_names.size(), (int)all_names.size());
    return false;
  }
  std::vector<std::string>::const_iterator ci_sub;
  std::vector<std::string>::const_iterator ci_all;
  for (ci_sub = subset_names.begin(); ci_sub != subset_names.end(); ++ci_sub)
  {
    bool found = false;
    for (ci_all = subset_names.begin(); !found && ci_all != subset_names.end(); ++ci_all)
    {
      if(ci_sub->compare(*ci_all) == 0)
      {
        found = true;
      }
    }
    if(!found)
    {
      ROS_ERROR_COND(verbose, "Variable named >%s< is not contained.", ci_sub->c_str());
      return false;
    }
  }
  return true;
}

bool haveSameContent(const std::vector<std::string>& names1,
                     const std::vector<std::string>& names2,
                     bool verbose)
{
  return (isContained(names1, names2, verbose) && isContained(names2, names1, verbose));
}

bool extractDataSample(const task_recorder2_msgs::DataSample& complete_data_sample,
                       const std::string& name,
                       task_recorder2_msgs::DataSample& extracted_data_sample)
{
  std::vector<std::string> names;
  names.push_back(name);
  return extractDataSample(complete_data_sample, names, extracted_data_sample);
}

bool extractDataSample(const task_recorder2_msgs::DataSample& complete_data_sample,
                       const std::vector<std::string>& names,
                       task_recorder2_msgs::DataSample& extracted_data_sample)
{
  std::vector<task_recorder2_msgs::DataSample> complete_data_samples;
  complete_data_samples.push_back(complete_data_sample);
  std::vector<task_recorder2_msgs::DataSample> extracted_data_samples;
  if(!extractDataSamples(complete_data_samples, names, extracted_data_samples))
  {
    return false;
  }
  ROS_ASSERT(complete_data_samples.size() == extracted_data_samples.size());
  extracted_data_sample = extracted_data_samples[0];
  return true;
}

bool extractDataSamples(const std::vector<task_recorder2_msgs::DataSample>& complete_data_samples,
                        const std::string& name,
                        std::vector<task_recorder2_msgs::DataSample>& extracted_data_samples)
{
  std::vector<std::string> names;
  names.push_back(name);
  return extractDataSamples(complete_data_samples, names, extracted_data_samples);
}

bool extractDataSamples(const std::vector<task_recorder2_msgs::DataSample>& complete_data_samples,
                        const std::vector<std::string>& names,
                        std::vector<task_recorder2_msgs::DataSample>& extracted_data_samples)
{
  ROS_ASSERT(!complete_data_samples.empty());
  std::vector<int> indices;
  if(!getIndices(complete_data_samples[0].names, names, indices))
  {
    return false;
  }

  task_recorder2_msgs::DataSample data_sample;
  data_sample.names = names;
  data_sample.data.resize(data_sample.names.size(), 0.0);
  extracted_data_samples.resize(complete_data_samples.size(), data_sample);
  for (unsigned int i = 0; i < complete_data_samples.size(); ++i)
  {
    extracted_data_samples[i].header = complete_data_samples[i].header;
    for (unsigned int j = 0; j < names.size(); ++j)
    {
      extracted_data_samples[i].data[j] = complete_data_samples[i].data[indices[j]];
    }
  }
  return true;
}

}

#endif /* DATA_SAMPLE_UTILITIES_H_ */
