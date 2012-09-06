/*
 * irl_data.cpp
 *
 *  Created on: Aug 28, 2012
 *      Author: kalakris
 */

#include <inverse_reinforcement_learning/irl_data.h>
#include <ros/assert.h>
#include <fstream>

namespace inverse_reinforcement_learning
{

IRLData::IRLData(int num_features):
    num_samples_(0),
    num_features_(num_features)
{
}

IRLData::~IRLData()
{

}

void IRLData::addSamples(const Eigen::MatrixXd& features, const Eigen::VectorXd& target)
{
  ROS_ASSERT(features.rows() == target.rows());
  ROS_ASSERT(features.cols() == num_features_);

  int new_num_samples = features.rows();
  int total_num_samples = new_num_samples + num_samples_;
  Eigen::MatrixXd new_features = Eigen::MatrixXd(total_num_samples, num_features_);
  Eigen::VectorXd new_target = Eigen::VectorXd(total_num_samples);

  if (num_samples_ > 0)
    new_features.topRows(num_samples_) = features_;
  new_features.bottomRows(new_num_samples) = features;
  features_ = new_features;

  if (num_samples_ > 0)
    new_target.head(num_samples_) = target_;
  new_target.tail(new_num_samples) = target;
  target_ = new_target;

  num_samples_ = total_num_samples;
}

double IRLData::getRank(const Eigen::VectorXd& weights)
{
  exp_w_phi_ = features_ * weights;
  std::vector<std::pair<double, int> > sorter;
  for (int i=0; i<num_samples_; ++i)
  {
    sorter.push_back(std::make_pair(exp_w_phi_[i], i));
  }
  std::sort(sorter.begin(), sorter.end());
  double total_rank = 0.0;
  int total_rank_div = 0;
  for (int i=0; i<num_samples_; ++i)
  {
    if (target_[i] > 0.5)
    {
      for (int j=0; j<num_samples_; ++j)
      {
        if (sorter[j].second == i)
        {
          total_rank += double(j)/double(num_samples_-1);
          ++total_rank_div;
          break;
        }
      }
    }
  }
  return total_rank / total_rank_div;
}

void IRLData::computeGradient(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient, double& log_likelihood)
{
  ROS_ASSERT(weights.rows() == num_features_);
  exp_w_phi_ = (-features_ * weights).array().exp().matrix();
  double sum_exp_w_phi = exp_w_phi_.sum();
//  if (sum_exp_w_phi < 1e-30)
//  {
//    std::cout << exp_w_phi_ << std::endl;
//    ROS_BREAK();
//    sum_exp_w_phi = 1e-30;
//  }
  if (sum_exp_w_phi > 0.0)
    y_ = exp_w_phi_ / sum_exp_w_phi;
  else
  {
    //printf("triggered!\n");
    y_ = (1.0/num_samples_) * Eigen::VectorXd::Ones(num_samples_);
  }
  log_likelihood = log(y_.dot(target_)); // assumes that only a single target is 1.0 and everything else is 0.0
  gradient = ((target_ - y_).transpose() * features_).transpose();
}

void IRLData::computeGradient2(const Eigen::VectorXd& weights, Eigen::VectorXd& gradient, double& log_likelihood,
                      Eigen::VectorXd& gradient2)
{
  computeGradient(weights, gradient, log_likelihood);
  gradient2 =  (features_.array()*features_.array()).matrix().transpose()*
      ((y_.array()*(( Eigen::VectorXd::Ones(num_samples_) -y_).array())).matrix());
}

void IRLData::saveToFile(const std::string& abs_file_name)
{
  std::ofstream f(abs_file_name.c_str());
  //printf("Writing data to %s\n", abs_file_name.c_str());
  f << num_samples_ << "\t" << num_features_ << std::endl;
  for (int i=0; i<num_samples_; ++i)
  {
    f << target_[i];
    for (int j=0; j<num_features_; ++j)
    {
      f << "\t" << features_(i,j);
    }
    f << std::endl;
  }
  f.close();
}

void IRLData::loadFromFile(const std::string& abs_file_name)
{
  std::ifstream f(abs_file_name.c_str());
  f >> num_samples_ >> num_features_;
  features_ = Eigen::MatrixXd(num_samples_, num_features_);
  target_ = Eigen::VectorXd(num_samples_);
  for (int i=0; i<num_samples_; ++i)
  {
    f >> target_[i];
    for (int j=0; j<num_features_; ++j)
    {
      f >> features_(i,j);
    }
  }
  f.close();
//  printf("Loaded %d samples with %d features from %s\n",
//         num_samples_, num_features_, abs_file_name.c_str());
}

int IRLData::getNumFeatures()
{
  return num_features_;
}

int IRLData::getNumSamples()
{
  return num_samples_;
}

} /* namespace inverse_reinforcement_learning */
