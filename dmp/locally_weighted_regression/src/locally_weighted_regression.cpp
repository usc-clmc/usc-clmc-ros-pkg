/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		locally_weighted_regression.cpp

 \author	Peter Pastor
 \date		Dec 6, 2010

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// local includes
#include <locally_weighted_regression/locally_weighted_regression.h>

using namespace Eigen;
using namespace std;

namespace lwr
{

static const char* lwr_model_topic_file_name = "lwr_model";

bool LocallyWeightedRegression::initFromNodeHandle(ros::NodeHandle& node_handle, const double cutoff)
{
  ROS_DEBUG("Initializing LWR model from node handle.");
  ROS_WARN_COND(initialized_,"LWR model already initialized. Re-initializing...");

  // read LWR parameters from param server and initialize base class
  ros::NodeHandle lwr_node_handle(node_handle, "lwr");

  double activation = 0;
  ROS_VERIFY(usc_utilities::read(lwr_node_handle, "activation", activation));
  if ((activation <= 0 || activation >= 1.0))
  {
    ROS_ERROR("Wrong value for activation parameter (%f) ...should be between 0 and 1.",activation);
    return (initialized_ = false);
  }
  int num_rfs = 0;
  ROS_VERIFY(usc_utilities::read(lwr_node_handle, "num_rfs", num_rfs));

  bool exponentially_spaced = true;
  ROS_VERIFY(usc_utilities::read(lwr_node_handle, "exponentially_spaced", exponentially_spaced));

  // Not used yet
  bool use_offset = false;
  ROS_VERIFY(usc_utilities::read(lwr_node_handle, "use_offset", use_offset));

  if(exponentially_spaced)
  {
    if(cutoff < 0)
    {
      ROS_ERROR("If basis functions are spaced exponentially, a cutoff has to be provided.");
      return (initialized_ = false);
    }
  }

  lwr_lib::LWRParamPtr parameters(new lwr_lib::LWRParameters());
  ROS_VERIFY(parameters->initialize(num_rfs, activation, exponentially_spaced, cutoff));

  lwr_model_.reset(new lwr_lib::LWR());
  ROS_VERIFY(lwr_model_->initialize(parameters));

  return (initialized_ = true);
}

bool LocallyWeightedRegression::initFromMessage(const LWRModelMsg& model)
{
  ROS_DEBUG("Initializing LWR model from message.");
  ROS_WARN_COND(initialized_,"LWR model already initialized. Re-initializing...");

  // create parameters
  lwr_lib::LWRParamPtr parameters(new lwr_lib::LWRParameters());
  if(!parameters->initialize(model.centers, model.widths, model.slopes, model.offsets))
  {
    return (initialized_ = false);
  }

  // create lwr model using these parameters
  lwr_model_.reset(new lwr_lib::LWR());
  if(!lwr_model_->initialize(parameters))
  {
    return (initialized_ = false);
  }

  return (initialized_ = true);
}

bool LocallyWeightedRegression::isInitialized() const
{
  return initialized_;
}

bool LocallyWeightedRegression::learn(const VectorXd& x_input_vector,
                                      const VectorXd& y_target_vector)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->learn(x_input_vector, y_target_vector);
}

// REAL-TIME REQUIREMENTS
bool LocallyWeightedRegression::predict(const double x_query,
                                        double& y_prediction)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->predict(x_query, y_prediction);
}

bool LocallyWeightedRegression::getThetas(VectorXd& thetas) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getThetas(thetas);
}

bool LocallyWeightedRegression::getThetas(vector<double>& thetas) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getThetas(thetas);
}

bool LocallyWeightedRegression::setThetas(const VectorXd& thetas)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setThetas(thetas);
}

bool LocallyWeightedRegression::setThetas(const vector<double>& thetas)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setThetas(thetas);
}

bool LocallyWeightedRegression::updateThetas(const VectorXd& delta_thetas)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->updateThetas(delta_thetas);
}

bool LocallyWeightedRegression::setWidthsAndCenters(const VectorXd& widths,
                                                    const VectorXd& centers)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setWidthsAndCenters(widths, centers);
}

bool LocallyWeightedRegression::setWidthsAndCenters(const vector<double>& widths,
                                                    const vector<double>& centers)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setWidthsAndCenters(widths, centers);
}

bool LocallyWeightedRegression::getWidthsAndCenters(VectorXd& widths,
                                                    VectorXd& centers) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getWidthsAndCenters(widths, centers);
}

bool LocallyWeightedRegression::getWidthsAndCenters(vector<double>& widths,
                                                    vector<double>& centers) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getWidthsAndCenters(widths, centers);
}

bool LocallyWeightedRegression::setOffsets(const VectorXd& offsets)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setOffsets(offsets);
}

bool LocallyWeightedRegression::setOffsets(const vector<double>& offsets)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setOffsets(offsets);
}

bool LocallyWeightedRegression::getOffsets(VectorXd& offsets) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getOffsets(offsets);
}

bool LocallyWeightedRegression::getOffsets(vector<double>& offsets) const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getOffsets(offsets);
}

bool LocallyWeightedRegression::setNumRFS(const int num_rfs)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->setNumRFS(num_rfs);
}

int LocallyWeightedRegression::getNumRFS() const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getNumRFS();
}

bool LocallyWeightedRegression::generateBasisFunctionMatrix(const Eigen::VectorXd& x_input_vector,
                                                            Eigen::MatrixXd& basis_function_matrix)
{
  ROS_ASSERT(initialized_);
  return lwr_model_->generateBasisFunctionMatrix(x_input_vector, basis_function_matrix);
}

string LocallyWeightedRegression::getInfoString() const
{
  ROS_ASSERT(initialized_);
  return lwr_model_->getInfoString();
}

bool LocallyWeightedRegression::writeToMessage(LWRModelMsg& model)
{
  ROS_ASSERT(initialized_);
  ROS_ASSERT(lwr_model_->isInitialized());
  model.num_rfs = lwr_model_->getNumRFS();
  ROS_VERIFY(lwr_model_->getWidthsAndCenters(model.widths, model.centers));
  ROS_VERIFY(lwr_model_->getThetas(model.slopes));
  ROS_VERIFY(lwr_model_->getOffsets(model.offsets));
  return true;
}

bool LocallyWeightedRegression::readFromDisc(const string& bag_file_name)
{
  ROS_WARN_COND(initialized_,"LWR model already initialized. Re-initializing...");
  try
  {
    rosbag::Bag bag(bag_file_name, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(lwr_model_topic_file_name));
    int num_msgs = 0;
    BOOST_FOREACH(rosbag::MessageInstance const msg, view)
    {
      if(num_msgs > 0)
      {
        ROS_ERROR("Bag file contains more than 1 message on topic >%s<.", lwr_model_topic_file_name);
        bag.close();
        return false;
      }
      LWRModelMsg::ConstPtr model = msg.instantiate<LWRModelMsg> ();
      if(model != NULL)
      {
        num_msgs++;
        if(!initFromMessage(*model))
        {
          ROS_ERROR("Could not initialize LWR model from message.");
          bag.close();
          return (initialized_ = false);
        }
      }
    }
    if(num_msgs != 1)
    {
      ROS_ERROR("Could not read msg of topic >%s< bag file >%s<.", lwr_model_topic_file_name, bag_file_name.c_str());
      bag.close();
      return (initialized_ = false);
    }
    bag.close();
  }
  catch (rosbag::BagIOException& ex)
  {
    ROS_ERROR("Could not open bag file %s: %s", bag_file_name.c_str(), ex.what());
    return (initialized_ = false);
  }
  return (initialized_ = true);
}

bool LocallyWeightedRegression::writeToDisc(const string& bag_file_name)
{
  ROS_ASSERT(initialized_);
  try
  {
    rosbag::Bag bag(bag_file_name, rosbag::bagmode::Write);
    LWRModelMsg model;
    if (!writeToMessage(model))
    {
      ROS_ERROR("Could not write LWR model to ROS message.");
      return false;
    }
    bag.write(string(lwr_model_topic_file_name), ros::Time::now(), model);
    bag.close();
  }
  catch (rosbag::BagIOException& ex)
  {
    ROS_ERROR("Could not open bag file %s: %s", bag_file_name.c_str(), ex.what());
    return false;
  }
  return true;
}

lwr_lib::LWRPtr LocallyWeightedRegression::getModel() const
{
  ROS_ASSERT(initialized_);
  return lwr_model_;
}

bool writeToMessage(const lwr_lib::LWRConstPtr lwr_model, LWRModelMsg& model)
{
  ROS_ASSERT(lwr_model->isInitialized());
  model.num_rfs = lwr_model->getNumRFS();
  ROS_VERIFY(lwr_model->getWidthsAndCenters(model.widths, model.centers));
  ROS_VERIFY(lwr_model->getThetas(model.slopes));
  ROS_VERIFY(lwr_model->getOffsets(model.offsets));
  return true;
}

}
