/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		parameters.cpp

 \author	Peter Pastor
 \date		Nov 4, 2010

 *********************************************************************/

// system include
#include <cassert>
#include <sstream>

// local include
#include <lwr_lib/lwr_parameters.h>
#include <lwr_lib/logger.h>

using namespace Eigen;
using namespace std;

namespace lwr_lib
{

//bool LWRParameters::initialize(const string& file_name)
//{
//    printf("INFO: Reading lwr parameters from file >%s< (NOT IMPLEMENTED YET)\n.", file_name.c_str());
//    return false;
//}

bool LWRParameters::initialize(const VectorXd& centers,
                               const VectorXd& widths,
                               const VectorXd& slopes,
                               const VectorXd& offsets)
{
  Logger::logPrintf(initialized_, "LWR parameters already initialized. Re-initializing...", Logger::WARN);
  assert(centers.rows() == widths.rows());
  assert(centers.rows() == slopes.rows());
  assert(centers.rows() == offsets.rows());
  assert(centers.cols() == widths.cols());
  assert(centers.cols() == slopes.cols());
  assert(centers.cols() == offsets.cols());
  if (centers.rows() < centers.cols())
  {
    Logger::logPrintf("Cannot initialize lwr parameters from row vectors.", Logger::ERROR);
    return (initialized_ = false);
  }
  num_rfs_ = centers.rows();
  if (num_rfs_ == 0)
  {
    Logger::logPrintf("Cannot initialize lwr parameters from empty vectors.", Logger::ERROR);
    return (initialized_ = false);
  }
  centers_ = centers;
  widths_ = widths;
  slopes_ = slopes;
  offsets_ = offsets;
  return (initialized_ = true);
}

bool LWRParameters::initialize(const vector<double>& centers,
                               const vector<double>& widths,
                               const vector<double>& slopes,
                               const vector<double>& offsets)
{
  Logger::logPrintf(initialized_, "LWR parameters already initialized. Re-initializing...", Logger::WARN);
  assert(centers.size() == widths.size());
  assert(centers.size() == slopes.size());
  assert(centers.size() == offsets.size());
  num_rfs_ = static_cast<int> (centers.size());
  if (num_rfs_ == 0)
  {
    Logger::logPrintf("Cannot initialize lwr parameters from empty vectors.", Logger::ERROR);
    return (initialized_ = false);
  }
  centers_ = VectorXd::Zero(num_rfs_, 1);
  slopes_ = VectorXd::Zero(num_rfs_, 1);
  widths_ = VectorXd::Zero(num_rfs_, 1);
  offsets_ = VectorXd::Zero(num_rfs_, 1);
  if (!setThetas(slopes))
  {
    return (initialized_ = false);
  }
  if (!setWidthsAndCenters(widths, centers))
  {
    return (initialized_ = false);
  }
  if (!setOffsets(offsets))
  {
    return (initialized_ = false);
  }
  return (initialized_ = true);
}

bool LWRParameters::initialize(const int num_rfs,
                               const double activation,
                               bool exponentially_spaced,
                               const double cutoff)
{
  Logger::logPrintf(initialized_, "LWR parameters already initialized. Re-initializing...", Logger::WARN);
  if (num_rfs <= 0)
  {
    Logger::logPrintf("Number of receptive fields >%i< is invalid.", Logger::ERROR, num_rfs_);
    return (initialized_ = false);
  }
  num_rfs_ = num_rfs;
  centers_ = VectorXd::Zero(num_rfs_, 1);
  slopes_ = VectorXd::Zero(num_rfs_, 1);
  widths_ = VectorXd::Zero(num_rfs_, 1);
  offsets_ = VectorXd::Zero(num_rfs_, 1);

  if (exponentially_spaced)
  {
    double last_input_x = 1.0;
    double alpha_x = -log(cutoff);
    for (int i = 0; i < num_rfs_; ++i)
    {
      double t = (i + 1) * (1. / static_cast<double> (num_rfs_ - 1)) * 1.0; // 1.0 is the default duration
      double input_x = exp(-alpha_x * t);
      widths_(i) = pow(input_x - last_input_x, 2) / -log(activation);
      centers_(i) = last_input_x;
      last_input_x = input_x;
    }
  }
  else
  {
    double diff;
    if (num_rfs_ == 1)
    {
      centers_(0) = 0.5;
      diff = 0.5;
    }
    else
    {
      for (int i = 0; i < num_rfs_; i++)
      {
        centers_(i) = static_cast<double> (i) / static_cast<double> (num_rfs - 1);
      }
      diff = static_cast<double> (1.0) / static_cast<double> (num_rfs - 1);
    }
    double width = -pow(diff / static_cast<double> (2.0), 2) / log(activation);
    for (int i = 0; i < num_rfs_; i++)
    {
      widths_(i) = width;
    }
  }

  return (initialized_ = true);
}

bool LWRParameters::getThetas(VectorXd& thetas) const
{
  assert(initialized_);
  assert((thetas.cols() == slopes_.cols()) && (thetas.rows() == slopes_.rows()));
  thetas = slopes_;
  return true;
}

bool LWRParameters::getThetas(vector<double>& thetas) const
{
  assert(initialized_);
  thetas.clear();
  for (int i = 0; i < slopes_.rows(); ++i)
  {
    thetas.push_back(slopes_(i));
  }
  return true;
}

bool LWRParameters::setThetas(const VectorXd& thetas)
{
  assert((thetas.cols() == slopes_.cols()) && (thetas.rows() == slopes_.rows()));
  slopes_ = thetas;
  return true;
}
bool LWRParameters::setThetas(const vector<double>& thetas)
{
  assert((static_cast<int>(thetas.size()) == slopes_.rows()) && (1 == slopes_.cols()));
  for (int i = 0; i < slopes_.rows(); ++i)
  {
    slopes_(i) = thetas[i];
  }
  return true;
}

bool LWRParameters::updateThetas(const VectorXd& delta_thetas)
{
  assert(initialized_);
  assert((delta_thetas.cols() == slopes_.cols()) && (delta_thetas.rows() == slopes_.rows()));
  slopes_ += delta_thetas;
  return true;
}

bool LWRParameters::setNumRFS(const int num_rfs)
{
  if (num_rfs <= 0)
  {
    Logger::logPrintf("Number of receptive fields >%i< is invalid.", Logger::ERROR, num_rfs);
    return false;
  }
  num_rfs_ = num_rfs;
  return true;
}

int LWRParameters::getNumRFS() const
{
  return num_rfs_;
}

bool LWRParameters::setWidthsAndCenters(const VectorXd& widths,
                                        const VectorXd& centers)
{
  assert((widths.cols() == widths_.cols()) && (widths.rows() == widths_.rows()));
  assert((centers.cols() == centers_.cols()) && (centers.rows() == centers_.rows()));
  widths_ = widths;
  centers_ = centers;
  return true;
}

bool LWRParameters::setWidthsAndCenters(const vector<double>& widths,
                                        const vector<double>& centers)
{
  assert((static_cast<int>(widths.size()) == widths_.rows()) && (1 == widths_.cols()));
  assert((static_cast<int>(centers.size()) == centers_.rows()) && (1 == centers_.cols()));
  for (int i = 0; i < widths_.rows(); ++i)
  {
    widths_(i) = widths[i];
    centers_(i) = centers[i];
  }
  return true;
}

bool LWRParameters::getWidthsAndCenters(VectorXd& widths,
                                        VectorXd& centers) const
{
  assert(initialized_);
  assert((widths.cols() == widths_.cols()) && (widths.rows() == widths_.rows()));
  assert((centers.cols() == centers_.cols()) && (centers.rows() == centers_.rows()));
  widths = widths_;
  centers = centers_;
  return true;
}

bool LWRParameters::getWidthsAndCenters(vector<double>& widths,
                                        vector<double>& centers) const
{
  assert(initialized_);
  widths.clear();
  centers.clear();
  for (int i = 0; i < widths_.rows(); ++i)
  {
    widths.push_back(widths_(i));
    centers.push_back(centers_(i));
  }
  return true;
}

bool LWRParameters::setOffsets(const VectorXd& offsets)
{
  assert((offsets.cols() == offsets_.cols()) && (offsets.rows() == offsets_.rows()));
  offsets_ = offsets;
  return true;
}

bool LWRParameters::setOffsets(const vector<double>& offsets)
{
  assert((static_cast<int>(offsets.size()) == offsets_.rows()) && (1 == offsets_.cols()));
  for (int i = 0; i < offsets_.rows(); ++i)
  {
    offsets_(i) = offsets[i];
  }
  return true;
}

bool LWRParameters::getOffsets(VectorXd& offsets) const
{
  assert(initialized_);
  assert((offsets.cols() == offsets_.cols()) && (offsets.rows() == offsets_.rows()));
  offsets = offsets_;
  return true;
}

bool LWRParameters::getOffsets(vector<double>& offsets) const
{
  assert(initialized_);
  offsets.clear();
  for (int i = 0; i < offsets_.rows(); ++i)
  {
    offsets.push_back(offsets_(i));
  }
  return true;
}

string LWRParameters::getInfoString() const
{
  string info;
  stringstream ss;
  ss << num_rfs_;
  info.assign("LWRParameters:\n num_rfs: " + ss.str());
  ss.clear();
  ss.str("");
  ss.precision(4);
  ss << std::fixed;
  string centers, widths, slopes, offsets;
  centers.assign("\n centers: ");
  widths.assign("\n widths:  ");
  slopes.assign("\n slopes:  ");
  offsets.assign("\n offsets: ");
  for (int i = 0; i < num_rfs_; ++i)
  {
    ss << centers_(i);
    centers.append(ss.str());
    if (i + 1 < num_rfs_)
      centers.append(string(" "));
    ss.clear();
    ss.str("");
    ss << widths_(i);
    widths.append(ss.str());
    if (i + 1 < num_rfs_)
      widths.append(string(" "));
    ss.clear();
    ss.str("");
    ss << slopes_(i);
    slopes.append(ss.str());
    if (i + 1 < num_rfs_)
      slopes.append(string(" "));
    ss.clear();
    ss.str("");
    ss << offsets_(i);
    offsets.append(ss.str());
    if (i + 1 < num_rfs_)
      offsets.append(string(" "));
    ss.clear();
    ss.str("");
  }
  info.append(centers + widths + slopes + offsets);
  return info;
}

bool LWRParameters::writeToDisc(const string& file_name)
{
  Logger::logPrintf("Writing lwr parameters from file >%s< is NOT implemented yet.", Logger::ERROR, file_name.c_str());
  return false;
}

}
