/*
 * irl_test.cpp
 *
 *  Created on: Aug 29, 2012
 *      Author: kalakris
 */

#include <boost/scoped_ptr.hpp>
#include <inverse_reinforcement_learning/irl_data.h>
#include <inverse_reinforcement_learning/irl_optimizer.h>
#include <cstdio>
#include <sstream>
#include <iomanip>

namespace inverse_reinforcement_learning
{

class IRLTest
{
public:
  IRLTest();
  virtual ~IRLTest();

  void generateData();
  void runOptimization();
  void loadDataFromDir(const char* dir, int num_files);

private:
  int num_features_;
  bool real_weights_exist_;
  Eigen::VectorXd real_weights_;
  Eigen::VectorXd est_weights_;

  std::vector<boost::shared_ptr<IRLData> > data_;
  boost::scoped_ptr<IRLOptimizer> optimizer_;
};

IRLTest::IRLTest()
{
  real_weights_exist_ = false;
}

IRLTest::~IRLTest()
{
}

void IRLTest::generateData()
{
  int num_active_features;
  int num_data;
  int num_samples_per_data;
  double cost_noise_stddev;

  num_features_ = 10;
  num_active_features = 2;
  num_data = 10;
  num_samples_per_data = 10;
  cost_noise_stddev = 0.1;

  // generate random weights
  real_weights_ = Eigen::VectorXd::Zero(num_features_);
  real_weights_.head(num_active_features).setRandom();

  data_.resize(num_data);
  for (int i=0; i<num_data; ++i)
  {
    IRLData* d = new IRLData(num_features_);
    Eigen::MatrixXd features = Eigen::MatrixXd::Zero(num_samples_per_data, num_features_);
    Eigen::VectorXd target = Eigen::VectorXd::Zero(num_samples_per_data);
    features.setRandom();

    // compute w^t * phi
    Eigen::VectorXd costs = features*real_weights_;

    // add random noise to costs
    costs += cost_noise_stddev * Eigen::VectorXd::Random(num_samples_per_data);

    // set min cost target to 1
    int min_index;
    double min_cost = costs.minCoeff(&min_index);
    target(min_index) = 1.0;
    d->addSamples(features, target);
    data_[i].reset(d);
  }
  real_weights_exist_ = true;
}

void IRLTest::loadDataFromDir(const char* dir, int num_files)
{
  real_weights_exist_ = false;
  data_.resize(num_files);

  for (int i=0; i<num_files; ++i)
  {
    std::stringstream file_name;
    file_name << dir << "/" << std::setfill('0') <<
        std::setw(3) << i << ".txt";
    IRLData* d = new IRLData(0);
    d->loadFromFile(file_name.str());
    data_[i].reset(d);
    num_features_ = d->getNumFeatures();
  }
}

void IRLTest::runOptimization()
{
  est_weights_ = Eigen::VectorXd::Zero(num_features_);
  optimizer_.reset(new IRLOptimizer(num_features_));
  //optimizer_->setBeta(100.0);
  optimizer_->setData(data_);
  optimizer_->setWeights(est_weights_);
  //optimizer_->runSBMLR();
  optimizer_->optimizeWithCV(6);
//  optimizer_->setAlpha(0.000000001);
//  optimizer_->useAllData();
//  optimizer_->runLBFGS();
  optimizer_->getWeights(est_weights_);

  if (real_weights_exist_)
  {
    printf("Weights: (real / est)\n");
    for (int i=0; i<num_features_; ++i)
    {
      printf("%f\t%f\n", real_weights_[i], est_weights_[i]);
    }
  }
  else
  {
    printf("Weights: (est)\n");
    for (int i=0; i<num_features_; ++i)
    {
      printf("%f\n", est_weights_[i]);
    }
  }
  optimizer_->computeAverageRank();
}

} /* namespace inverse_reinforcement_learning */

int main(int argc, char** argv)
{
  inverse_reinforcement_learning::IRLTest irl_test;
  if (argc == 3)
  {
    int num_files = atoi(argv[2]);
    irl_test.loadDataFromDir(argv[1], num_files);
  }
  else
  {
    irl_test.generateData();
  }
  irl_test.runOptimization();
}
