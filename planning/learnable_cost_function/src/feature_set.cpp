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
}

FeatureSet::~FeatureSet()
{
}

bool FeatureSet::initialize(XmlRpc::XmlRpcValue& config)
{

}

int FeatureSet::getNumValues() const
{

}

void FeatureSet::computeValuesAndGradients(boost::shared_ptr<Input const> input, std::vector<double>& feature_values,
                               bool compute_gradients, std::vector<Eigen::VectorXd>& gradients, bool& state_validity)
{

}

std::string FeatureSet::getName() const
{
  return "FeatureSet";
}

boost::shared_ptr<Feature> FeatureSet::clone() const
{

}

} /* namespace stomp_ros_interface */
