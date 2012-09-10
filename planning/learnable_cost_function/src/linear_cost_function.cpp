/*
 * linear_cost_function.cpp
 *
 *  Created on: Jul 27, 2011
 *      Author: kalakris
 */

#include <ros/assert.h>
#include <usc_utilities/file_io.h>
#include <usc_utilities/assert.h>
#include <learnable_cost_function/linear_cost_function.h>
#include <fstream>

namespace learnable_cost_function
{

LinearCostFunction::LinearCostFunction()
{
}

LinearCostFunction::~LinearCostFunction()
{
}

void LinearCostFunction::getValueAndGradient(boost::shared_ptr<Input const> input, double& value,
                         bool compute_gradient, Eigen::VectorXd& gradient, bool& state_validity,
                         Eigen::VectorXd& feature_values)
{
  value = 0.0;
  int num_input_dimensions = input->getNumDimensions();
  gradient = Eigen::VectorXd::Zero(num_input_dimensions);
  state_validity = true;
  feature_values = Eigen::VectorXd::Zero(num_feature_values_);
  int counter = 0;
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    bool validity;
    features_[i].feature->computeValuesAndGradients(input,
                                                    features_[i].values,
                                                    compute_gradient,
                                                    features_[i].gradients,
                                                    validity
                                                    );
    state_validity = state_validity && validity;
    for (int j=0; j<features_[i].num_values; ++j)
    {
      feature_values[counter] = features_[i].values[j];
      value += features_[i].weights[j] * features_[i].values[j];
      ++counter;
      if (compute_gradient)
      {
        gradient += features_[i].weights[j] * features_[i].gradients[j];
      }
    }
  }
}

void LinearCostFunction::clear()
{
  features_.clear();
  num_feature_values_ = 0;
}

void LinearCostFunction::setWeights(const Eigen::VectorXd& weights)
{
  int counter = 0;
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    for (int j=0; j<features_[i].num_values; ++j)
    {
      features_[i].weights[j] = weights[counter];
      ++counter;
    }
  }
}

void LinearCostFunction::addFeatures(std::vector<boost::shared_ptr<Feature> > features)
{
  int num_features = features.size();

  for (int i=0; i<num_features; ++i)
  {
    FeatureInfo fi;
    fi.feature = features[i];
    fi.num_values = features[i]->getNumValues();
    fi.weights.resize(fi.num_values, 0.0);
    fi.values.resize(fi.num_values, 0.0);
    features_.push_back(fi);
    num_feature_values_ += fi.num_values;
  }

}

void LinearCostFunction::addFeaturesAndWeights(std::vector<boost::shared_ptr<Feature> > features,
                           std::vector<double> weights)
{
  int num_features = features.size();

  int weight_index = 0;
  for (int i=0; i<num_features; ++i)
  {
    FeatureInfo fi;
    fi.feature = features[i];
    fi.num_values = features[i]->getNumValues();
    fi.weights.resize(fi.num_values);
    fi.values.resize(fi.num_values);
    for (int j=0; j<fi.num_values; ++j)
    {
      ROS_ASSERT(weight_index < (int)weights.size());
      fi.weights[j] = weights[weight_index++];
    }
    features_.push_back(fi);
  }

  num_feature_values_ += weight_index;

}

boost::shared_ptr<CostFunction> LinearCostFunction::clone()
{
  LinearCostFunction* lcf = new LinearCostFunction();
  lcf->features_ = features_;

  // clone each feature:
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    lcf->features_[i].feature = features_[i].feature->clone();
  }

  lcf->num_feature_values_ = num_feature_values_;
  //ROS_DEBUG("Cloned linear cost function with %d features, %d values", features_.size(), num_feature_values_);
  boost::shared_ptr<CostFunction> ret(lcf);
  return ret;
}

void LinearCostFunction::debugCost(double cost, const Eigen::VectorXd& feature_values)
{
  ROS_DEBUG("Cost = %f", cost);
  int counter = 0;
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    ROS_DEBUG("Feature %s:", features_[i].feature->getName().c_str());
    for (int j=0; j<features_[i].num_values; ++j)
    {
      ROS_DEBUG("%d: %f", j, feature_values[counter]);
      ++counter;
    }
  }
}

void LinearCostFunction::loadWeightsFromFile(const std::string& weights_file)
{
  std::vector<double> weights;
  ROS_VERIFY(usc_utilities::readDoubleArrayFromFile(weights_file, weights));
  ROS_ASSERT(num_feature_values_ == weights.size());
  int counter = 0;
  for (unsigned int i=0; i<features_.size(); ++i)
  {
    features_[i].weights.resize(features_[i].num_values);
    for (int j=0; j<features_[i].num_values; ++j)
    {
      features_[i].weights[j] = weights[counter];
      ++counter;
    }
  }
}

}
