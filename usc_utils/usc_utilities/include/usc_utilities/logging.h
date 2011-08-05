/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		logging.h

  \author	Peter Pastor
  \date		Jun 20, 2011

 *********************************************************************/

#ifndef USC_UTILITIES_LOGGING_H_
#define USC_UTILITIES_LOGGING_H_

// system includes
#include <ros/ros.h>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>

#include <boost/lexical_cast.hpp>

// local includes

namespace usc_utilities
{

inline bool log(const std::vector<double>& vector,
                const std::string filename = std::string("/tmp/log.txt"));

inline bool log(const std::vector<std::vector<double> >& matrix,
                const std::string filename = std::string("/tmp/log.txt"));

inline bool log(const Eigen::MatrixXd& matrix,
                const std::string filename = std::string("/tmp/log.txt"));

inline bool log(const std::vector<int>& vector,
                const std::string filename = std::string("/tmp/log.txt"));

inline bool read(std::vector<int>& vector,
                 const std::string filename = std::string("/tmp/log.txt"));
inline bool read(std::vector<double>& vector,
                 const std::string filename = std::string("/tmp/log.txt"));

inline bool read(std::vector<std::vector<double> >& matrix,
                 const std::string filename = std::string("/tmp/log.txt"));

// inline functions follow

inline bool log(const std::vector<double>& vector, const std::string filename)
{
  if(vector.empty())
  {
    ROS_ERROR("Vector is empty. Not logging anything to >%s<.", filename.c_str());
    return false;
  }
  Eigen::VectorXd eigen_vector = Eigen::VectorXd::Map(&vector[0], vector.size());
  std::ofstream outfile_input;
  outfile_input.open(filename.c_str());
  if(!outfile_input.is_open())
  {
    ROS_ERROR("Could not open file >%s<. Cannot log vector.", filename.c_str());
    return false;
  }
  outfile_input.setf(std::ios::fixed, std::ios::floatfield);
  outfile_input << eigen_vector;
  outfile_input.close();
  return true;
}

inline bool log(const std::vector<std::vector<double> >& matrix, const std::string filename)
{
  if(matrix.empty())
  {
    ROS_ERROR("Matrix is empty. Not logging anything to >%s<.", filename.c_str());
    return false;
  }
  for (int i = 0; i < (int)matrix.size(); ++i)
  {
    if(matrix[0].size() != matrix[i].size())
    {
      ROS_ERROR("Matrix is not rectangular. Row >%i< has >%i< colums, but should have >%i<. Not logging anything to >%s<.",
               i, (int)matrix[i].size(),(int)matrix[0].size(), filename.c_str());
      return false;
    }
  }
  Eigen::MatrixXd eigen_matrix = Eigen::MatrixXd::Zero((Eigen::DenseIndex)matrix.size(), (Eigen::DenseIndex)matrix[0].size());
  for (int i = 0; i < (int)matrix.size(); ++i)
  {
    eigen_matrix.row(i) = Eigen::VectorXd::Map(&matrix[i][0], matrix[i].size());
  }

  std::ofstream outfile_input;
  outfile_input.open(filename.c_str());
  if(!outfile_input.is_open())
  {
    ROS_ERROR("Could not open file >%s<. Cannot log matrix.", filename.c_str());
    return false;
  }
  outfile_input.setf(std::ios::fixed, std::ios::floatfield);
  outfile_input << eigen_matrix;
  outfile_input.close();
  return true;
}

inline bool log(const Eigen::MatrixXd& matrix, const std::string filename)
{
  std::ofstream outfile_input;
  outfile_input.open(filename.c_str());
  if(!outfile_input.is_open())
  {
    ROS_ERROR("Could not open file >%s<. Cannot log matrix.", filename.c_str());
    return false;
  }
  outfile_input.setf(std::ios::fixed, std::ios::floatfield);
  outfile_input << matrix;
  outfile_input.close();
  return true;
}

inline bool log(const std::vector<int>& vector, const std::string filename)
{
  if(vector.empty())
  {
    ROS_ERROR("Vector is empty. Not logging anything to >%s<.", filename.c_str());
    return false;
  }
  Eigen::VectorXi eigen_vector = Eigen::VectorXi::Map(&vector[0], vector.size());
  std::ofstream outfile_input;
  outfile_input.open(filename.c_str());
  if(!outfile_input.is_open())
  {
    ROS_ERROR("Could not open file >%s<. Cannot log vector.", filename.c_str());
    return false;
  }
  outfile_input.setf(std::ios::fixed);
  outfile_input << eigen_vector;
  outfile_input.close();

  return true;
}

// TODO: make this a template and use it everywhere
inline bool read(std::vector<int>& vector, const std::string filename)
{
  vector.clear();

  std::string line;
  std::ifstream infile(filename.c_str(), std::ios_base::in);
  if (infile.is_open())
  {
    while (getline(infile, line, '\n'))
    {
      try
      {
        vector.push_back(boost::lexical_cast<int>(line));
      }
      catch(boost::bad_lexical_cast ex)
      {
        ROS_ERROR("Problems when reading from file >%s< : %s", filename.c_str(), ex.what());
        return false;
      }
    }
  }
  else
  {
    ROS_ERROR("Problems when opening file >%s<.", filename.c_str());
    return false;
  }
  return true;
}
inline bool read(std::vector<double>& vector, const std::string filename)
{
  vector.clear();
  std::string line;
  std::ifstream infile(filename.c_str(), std::ios_base::in);
  if (infile.is_open())
  {
    while (getline(infile, line, '\n'))
    {
      try
      {
        vector.push_back(boost::lexical_cast<double>(line));
      }
      catch(boost::bad_lexical_cast ex)
      {
        ROS_ERROR("Problems when reading from file >%s< : %s", filename.c_str(), ex.what());
        return false;
      }
    }
  }
  else
  {
    ROS_ERROR("Problems when opening file >%s<.", filename.c_str());
    return false;
  }
  return true;
}

inline bool read(std::vector<std::vector<double> >& matrix, const std::string filename)
{
  matrix.clear();

  std::string line;
  std::ifstream infile(filename.c_str(), std::ios_base::in);
  int num_cols = -1;
  if (infile.is_open())
  {
    while (getline(infile, line, '\n'))
    {
      // split the group names based on whitespace
      std::stringstream ss_line(line);
      std::string line_part;
      std::vector<double> vector;
      while (ss_line >> line_part)
      {
        try
        {
          vector.push_back(boost::lexical_cast<double>(line_part));
        }
        catch(boost::bad_lexical_cast ex)
        {
          ROS_ERROR("Problems when reading from file >%s< : %s", filename.c_str(), ex.what());
          return false;
        }
      }
      if(num_cols < 0)
      {
        num_cols = (int)vector.size();
      }
      else
      {
        if(num_cols != (int)vector.size())
        {
          ROS_ERROR("Matrix read from file is not rectangular.");
          return false;
        }
      }
      matrix.push_back(vector);
    }
  }
  else
  {
    ROS_ERROR("Problems when opening file >%s<.", filename.c_str());
    return false;
  }

  ROS_INFO("Read matrix with >%i< rows and >%i< cols.", (int)matrix.size(), num_cols);
  return true;
}

}

#endif /* USC_UTILITIES_LOGGING_H_ */
