/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    lwpr_model.cpp

 \author  Peter Pastor
 \date    June 4, 2011

 *********************************************************************/

// system includes
#include <sstream>

#include <ros/ros.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>

// local includes
#include <lwpr/lwpr_model.h>

using namespace lwpr_lib;

namespace lwpr
{

LWPRModel::LWPRModel() : initialized_(false)
{
}

bool LWPRModel::initialize(ros::NodeHandle node_handle, const int index)
{
  node_handle_ = node_handle;
  index_ = index;

  ROS_VERIFY(usc_utilities::read(node_handle_, "cutoff", parameters_.cutoff_));

  int num_input_dimension, num_output_dimension;
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_input_dimension", num_input_dimension));
  ROS_VERIFY(usc_utilities::read(node_handle_, "num_output_dimension", num_output_dimension));
	if ((num_input_dimension <= 0) | (num_output_dimension <= 0))
	{
		ROS_ERROR_COND(num_input_dimension <= 0, "Number of input dimension >%i< is invalid.", num_input_dimension);
		ROS_ERROR_COND(num_output_dimension <= 0, "Number of output dimension >%i< is invalid.", num_output_dimension);
    return (initialized_ = false);
	}
  try
  {
    lwpr_object_.reset(new LWPR_Object(num_input_dimension, num_output_dimension));
  }
  catch (lwpr_lib::LWPR_Exception exception)
  {
    lwpr_lib::LWPR_Exception lwpr_exception(exception);
    ROS_ERROR("Could not create LWPR object : %s.", exception.getString());
    return (initialized_ = false);
  }
  parameters_.input_.resize(num_input_dimension);
  parameters_.output_.resize(num_output_dimension);
  parameters_.prediction_.resize(num_output_dimension);
  parameters_.confidence_.resize(num_output_dimension);

  std::vector<double> input_normalization_factors;
	ROS_VERIFY(usc_utilities::read(node_handle_, "input_normalization_factors", input_normalization_factors));
  if(input_normalization_factors.size() >= static_cast<unsigned int>(index_))
  {
    std::vector<double> norm;
    norm.push_back(input_normalization_factors[index_]);
    lwpr_object_->normIn(norm);
  }
  else
  {
    ROS_ERROR("Not enough (%i) input normalization factors provided (index = %i).",static_cast<int>(input_normalization_factors.size()), index_);
    return (initialized_ = false);
  }

  std::vector<double> output_normalization_factors;
  ROS_VERIFY(usc_utilities::read(node_handle_, "output_normalization_factors", output_normalization_factors));
  if(output_normalization_factors.size() >= static_cast<unsigned int>(index_))
  {
    std::vector<double> norm;
    norm.push_back(output_normalization_factors[index_]);
    lwpr_object_->normOut(norm);
  }
  else
  {
    ROS_ERROR("Not enough (%i) output normalization factors provided (index = %i).",static_cast<int>(output_normalization_factors.size()), index_);
    return (initialized_ = false);
  }

  std::string kernel_type;
	if (node_handle_.hasParam("kernel_type"))
	{
		node_handle_.getParam("kernel_type", kernel_type);
	}
	else
	{
	  kernel_type = std::string("Gaussian");
	}
	lwpr_object_->kernel(kernel_type.c_str());

	bool use_only_diagonal_elements;
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_only_diagonal_elements", use_only_diagonal_elements));
  lwpr_object_->diagOnly(use_only_diagonal_elements);

  bool allow_d_update;
  ROS_VERIFY(usc_utilities::read(node_handle_, "allow_d_update", allow_d_update));
  lwpr_object_->updateD(allow_d_update);

  bool init_all_alpha;
  ROS_VERIFY(usc_utilities::read(node_handle_, "init_all_alpha", init_all_alpha));
  lwpr_object_->setInitAlpha(init_all_alpha);

  bool allow_meta_learning;
  ROS_VERIFY(usc_utilities::read(node_handle_, "allow_meta_learning", allow_meta_learning));
  lwpr_object_->useMeta(allow_meta_learning);

  double meta_learning_rate;
  ROS_VERIFY(usc_utilities::read(node_handle_, "meta_learning_rate", meta_learning_rate));
  lwpr_object_->metaRate(meta_learning_rate);

  double penalty;
  ROS_VERIFY(usc_utilities::read(node_handle_, "penalty", penalty));
  lwpr_object_->penalty(penalty);

  bool use_init_all_diag_D;
  ROS_VERIFY(usc_utilities::read(node_handle_, "use_init_all_diag_D", use_init_all_diag_D));
	if (use_init_all_diag_D)
  {
	  if(num_input_dimension == 1)
	  {
      double init_all_diag_D;
      ROS_VERIFY(usc_utilities::read(node_handle_, "init_all_diag_D", init_all_diag_D));
      lwpr_object_->setInitD(init_all_diag_D);
	  }
	  else
	  {
      std::vector<double> init_all_diag_D;
      ROS_VERIFY(usc_utilities::read(node_handle_, "init_all_diag_D", init_all_diag_D));
      lwpr_object_->setInitD(init_all_diag_D);
	  }
  }
  else
  {
    ROS_ERROR("Computed D not implemented yet.");
    return (initialized_ = false);
  }

	double weight_activation_threshold;
  ROS_VERIFY(usc_utilities::read(node_handle_, "weight_activation_threshold", weight_activation_threshold));
  lwpr_object_->wGen(weight_activation_threshold);

  double weight_prune_threshold;
  ROS_VERIFY(usc_utilities::read(node_handle_, "weight_prune_threshold", weight_prune_threshold));
  lwpr_object_->wPrune(weight_activation_threshold);

  double add_regression_direction_threshold;
  ROS_VERIFY(usc_utilities::read(node_handle_, "add_regression_direction_threshold", add_regression_direction_threshold));
  // WARNING: add_regression_direction_threshold not used

  double init_lambda;
  ROS_VERIFY(usc_utilities::read(node_handle_, "init_lambda", init_lambda));
  lwpr_object_->initLambda(init_lambda);
  double final_lambda;
  ROS_VERIFY(usc_utilities::read(node_handle_, "final_lambda", final_lambda));
  lwpr_object_->finalLambda(final_lambda);
  double tau_lambda;
  ROS_VERIFY(usc_utilities::read(node_handle_, "tau_lambda", tau_lambda));
  lwpr_object_->tauLambda(tau_lambda);

	return (initialized_ = true);
}

bool LWPRModel::writeToDisc(const std::string& file_name)
{
  return lwpr_object_->writeBinary(file_name.c_str());
}

bool LWPRModel::update(const double& x, const double& y, double& prediction)
{
  try
  {
    parameters_.input_[0] = x;
    parameters_.output_[0] = y;
    parameters_.prediction_ = lwpr_object_->update(parameters_.input_, parameters_.output_);
    prediction = parameters_.prediction_[0];
  }
  catch (lwpr_lib::LWPR_Exception exception)
  {
    ROS_ERROR("Problem when updating LWPR model : %s.", exception.getString());
    return false;
  }
  return true;
}

bool LWPRModel::predict(const double& x, double& y)
{
  try
  {
    parameters_.input_[0] = x;
    parameters_.output_ = lwpr_object_->predict(parameters_.input_, parameters_.cutoff_);
    y = parameters_.output_[0];
  }
  catch (lwpr_lib::LWPR_Exception exception)
  {
    ROS_ERROR("Problem when predicting output from LWPR model : %s.", exception.getString());
    return false;
  }

  return true;
}

bool LWPRModel::predict(const double& x, double& y, double& conf)
{
  try
  {
    parameters_.input_[0] = x;
    parameters_.output_ = lwpr_object_->predict(parameters_.input_, parameters_.confidence_, parameters_.cutoff_);
    y = parameters_.output_[0];
    conf = parameters_.confidence_[0];
  }
  catch (lwpr_lib::LWPR_Exception exception)
  {
    ROS_ERROR("Problem when predicting output from LWPR model : %s.", exception.getString());
    return false;
  }

  return true;
}

}

