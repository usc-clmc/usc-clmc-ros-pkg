/*
 * feature_set.cpp
 *
 *  Created on: May 25, 2012
 *      Author: kalakris
 */

#include <learnable_cost_function/feature_set.h>

namespace learnable_cost_function
{

FeatureSet::FeatureSet()
{
  clear();
}

FeatureSet::~FeatureSet()
{
}

bool FeatureSet::initialize(XmlRpc::XmlRpcValue& config)
{
  return true;
}

int FeatureSet::getNumValues() const
{
  return num_values_;
}

void FeatureSet::computeValuesAndGradients(boost::shared_ptr<Input const> input, std::vector<double>& feature_values,
                               bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity)
{
  if (compute_gradients)
  {
    int num_d = input->getNumDimensions();
    gradients.resize(num_values_, Eigen::VectorXd::Zero(num_d));
  }
  feature_values.resize(num_values_);

  state_validity = true;
  int index = 0;
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    bool validity;
    features_[i].feature->computeValuesAndGradients(input, features_[i].values, compute_gradients,
                                                    features_[i].gradients, validity);
    if (!validity)
      state_validity = false;
    for (int j=0; j<features_[i].num_values; ++j)
    {
      feature_values[index] = features_[i].values[j];
      if (compute_gradients)
      {
        gradients[index] = features_[i].gradients[j];
      }
      ++index;
    }
  }

//  ROS_DEBUG("Features:");
//  for (int d=0; d<num_values_; ++d)
//  {
//    ROS_DEBUG("%d: %lf", d, feature_values[d]);
//  }

}

std::string FeatureSet::getName() const
{
  return "FeatureSet";
}

boost::shared_ptr<Feature> FeatureSet::clone() const
{
  boost::shared_ptr<FeatureSet> ret(new FeatureSet());
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    ret->addFeature(features_[i].feature->clone());
  }
  return ret;
}

void FeatureSet::addFeature(boost::shared_ptr<Feature> feature)
{
  FeatureInfo fi;
  fi.feature = feature;
  fi.num_values = feature->getNumValues();
  num_values_ += fi.num_values;
  features_.push_back(fi);
}

void FeatureSet::clear()
{
  features_.clear();
  num_values_ = 0;
}

} /* namespace stomp_ros_interface */
