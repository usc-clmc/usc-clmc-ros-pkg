/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		test_data.h

  \author	Peter Pastor, Mrinal Kalakrishnan
  \date		Feb 16, 2011

 *********************************************************************/

#ifndef TEST_DATA_H_
#define TEST_DATA_H_

// system includes
#include <string>
#include <vector>

// local includes

namespace test_dmp
{

class TestData
{

public:

  /*! Constructor
   */
  TestData() :
    initialized_(false), num_rfs_(20), activation_(0.1), cutoff_(0.001), num_data_points_(3000), k_gain_(0), d_gain_(0) {};

  /*! Destructor
   */
  virtual ~TestData() {};

  enum TestCase
  {
    SIMPLE_TEST,
    QUAT_TEST,
  };

  /*!
   * @return
   */
  bool initialize(TestCase test_case = SIMPLE_TEST);

  /*!
   * @return
   */
  std::vector<std::string> getTestVariableNames() const
  {
    return variable_names_;
  }

  std::string getTestTrajectoryFileName() const
  {
    return test_trajectory_file_name_;
  }

  /*!
   * @param index
   * @param goal_offset
   * @return
   */
  bool getGoal(const int index, std::vector<double>& goal_offset) const;

  /*!
   * @param index
   * @param duration_scale
   * @return
   */
  bool getDurationScale(const int index, double& duration_scale) const;

  /*!
   * @param index
   * @param error_threshold
   * @return
   */
  bool getErrorThreshold(const int index, double& error_threshold) const;

  /*!
   * @return
   */
  int getNumTests() const;

  int getNumRFS() const
  {
    return num_rfs_;
  }

  double getActivation() const
  {
    return activation_;
  }

  int getNumDataPoints() const
  {
    return num_data_points_;
  }

  double getCutoff() const
  {
    return cutoff_;
  }

  double getKGain() const
  {
    return k_gain_;
  }
  double getDGain() const
  {
    return d_gain_;
  }

  TestCase getTestCase() const
  {
    return test_case_;
  }
  int getQuatTSIndex() const
  {
    return quat_ts_index_;
  }

private:

  bool initialized_;

  TestCase test_case_;
  int quat_ts_index_;

  int num_rfs_;
  double activation_;
  double cutoff_;
  int num_data_points_;

  double k_gain_;
  double d_gain_;

  std::string test_trajectory_file_name_;

  std::vector<std::string> variable_names_;
  std::vector<std::vector<double> > goal_offsets_;
  std::vector<double> duration_scales_;
  std::vector<double> error_thresholds_;

};

}

#endif /* TEST_DATA_H_ */
