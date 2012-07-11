/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...
 
 \file    trajectory.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Nov 4, 2010

 *********************************************************************/

#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// system include
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

// local include
#include <dmp_lib/status.h>
#include <dmp_lib/logger.h>

namespace dmp_lib
{

static const int MAX_TRAJECTORY_LENGTH = 30000;
static const int MAX_TRAJECTORY_DIMENSION = 350;

static const int ABSOLUTE_MAX_TRAJECTORY_LENGTH = 20 * MAX_TRAJECTORY_LENGTH;
static const int ABSOLUTE_MAX_TRAJECTORY_DIMENSION = 20 * MAX_TRAJECTORY_DIMENSION;

/*!
 */
class Trajectory : public Status
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Constructor
   */
  Trajectory() :
    positions_only_(false),
    trajectory_length_(0),
    trajectory_dimension_(0),
    index_to_last_trajectory_point_(0),
    sampling_frequency_(0.0),
    trajectory_duration_(0.0) {};

  /*! Destructor
   */
  virtual ~Trajectory() {};

  /*!
   * @param variable_names
   * @param sampling_frequency
   * @param positions_only
   * @param trajectory_length
   * @return True on success, otherwise False
   */
  bool initialize(const std::vector<std::string>& variable_names,
                  const double sampling_frequency,
                  const bool positions_only = false,
                  const int trajectory_length = MAX_TRAJECTORY_LENGTH);

  /*!
   * @param variable_names
   * @param sampling_frequency
   * @param start
   * @param goal
   * @param num_samples
   * @param positions_only
   * @param trajectory_length
   * @return True on success, otherwise False
   */
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const Eigen::VectorXd& start,
                             const Eigen::VectorXd& goal,
                             const int num_samples,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const std::vector<double>& start,
                             const std::vector<double>& goal,
                             const int num_samples,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);

  /*!
   * @param variable_names
   * @param sampling_frequency
   * @param duration
   * @param start
   * @param goal
   * @param positions_only
   * @param trajectory_length
   * @return True on success, otherwise False
   */
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const double duration,
                             const Eigen::VectorXd& start,
                             const Eigen::VectorXd& goal,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const double duration,
                             const std::vector<double>& start,
                             const std::vector<double>& goal,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);

  /*!
   * @param variable_names
   * @param sampling_frequency
   * @param waypoints
   * @param num_samples
   * @param positions_only
   * @param trajectory_length
   * @return True on success, otherwise False
   */
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const std::vector<Eigen::VectorXd>& waypoints,
                             const std::vector<int>& num_samples,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const std::vector<std::vector<double> >& waypoints,
                             const std::vector<int>& num_samples,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);

  /*!
   * @param variable_names
   * @param sampling_frequency
   * @param durations
   * @param waypoints
   * @param positions_only
   * @param trajectory_length
   * @return True on success, otherwise False
   */
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const std::vector<double>& durations,
                             const std::vector<Eigen::VectorXd>& waypoints,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);
  bool initializeWithMinJerk(const std::vector<std::string>& variable_names,
                             const double sampling_frequency,
                             const std::vector<double>& durations,
                             const std::vector<std::vector<double> >& waypoints,
                             const bool positions_only = false,
                             const int trajectory_length = MAX_TRAJECTORY_LENGTH);

  /*!
   * @param start
   * @param goal
   * @param num_samples
   * @return True on success, otherwise False
   */
  bool setMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const int num_samples);
  bool setMinJerk(const std::vector<double>& start, const std::vector<double>& goal, const int num_samples);

  /*!
   * @param start
   * @param goal
   * @param duration
   * @return True on success, otherwise False
   */
  bool setMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double duration);
  bool setMinJerk(const std::vector<double>& start, const std::vector<double>& goal, const double duration);

  /*!
   * @param waypoints
   * @param num_samples
   * @return
   */
  bool setMinJerk(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<int>& num_samples);
  bool setMinJerk(const std::vector<std::vector<double> >& waypoints, const std::vector<int>& num_samples);

  /*!
   * @param waypoints
   * @param durations
   * @return True on success, otherwise False
   */
  bool setMinJerk(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<double>& durations);
  bool setMinJerk(const std::vector<std::vector<double> >& waypoints, const std::vector<double>& durations);

  /*! Initializes the trajectory from the File pointed to by the provided file_name
   * @param file_name
   * @param positions_only
   * @return True on success, otherwise False
   */
  bool readFromCLMCFile(const std::string& file_name,
                        const bool positions_only = false);

  /*! Initializes the trajectory from the File pointed to by the provided file_name
   * @param file_name
   * @param variable_names
   * @param positions_only
   * @param use_variable_names If this is set to false, variable names parameter is ignored and ALL variable names are read from file
   * @return True on success, otherwise False
   */
  bool readFromCLMCFile(const std::string& file_name,
                        const std::vector<std::string>& variable_names,
                        const bool positions_only = false,
                        const bool use_variable_names = true);

  /*!
   * @param file_name
   * @return True on success, otherwise False
   */
  bool writeToCLMCFile(const std::string& file_name,
                       const bool positions_only = false) const;

  /*!
   * @param other_trajectory
   * @param verbose
   * @return True if this trajectory is compatible with the other_trajectory such that they can be combined
   */
  bool isCompatible(const Trajectory& other_trajectory,
                    bool verbose = false) const;

  /*!
   * @param other_trajectory
   * @param verbose
   * @return True if this trajectory can be appended with the other_trajectory
   */
  bool isAppendable(const Trajectory& other_trajectory,
                    bool verbose = false) const;

  /*!
   * @return Number of dimensions of the trajectory
   */
  int getDimension() const;

  /*!
   * @return Number of samples that fit in the trajectory.
   */
  int getNumTotalCapacity() const;

  /*!
   * @return Number of contained samples stored in the trajectory
   */
  int getNumContainedSamples() const;

  /*!
   * @return
   */
  double getDuration() const;

  /*!
   * @return True if Trajectory only contains positions, otherwise False
   */
  bool containsPositionsOnly() const;

  /*!
   * @param variable_names
   * @return True on success, otherwise False
   */
  bool setVariableNames(const std::vector<std::string>& variable_names);

  /*!
   * @param variable_names
   * @return A copy of the variable names contained in the trajectory
   */
  std::vector<std::string> getVariableNames() const;

  /*!
   * @param variable_names
   * @return True if all variable names are present in the trajectory, otherwise False
   */
  bool containVariables(const std::vector<std::string>& variable_names) const;

  /*!
   * @param trajectory_index
   * @param trajectory_point
   * @return True on success, otherwise False
   */
  bool getTrajectoryPoint(const int trajectory_index,
                          Eigen::VectorXd& trajectory_point) const;

  /*!
   * @param trajectory_index
   * @param trajectory_point
   * @return True on success, otherwise False
   */
  bool setTrajectoryPoint(const int trajectory_index,
                          const Eigen::VectorXd& trajectory_point);

  /*!
   * @param trajectory_index
   * @param trajectory_dimension
   * @param position
   * @return True on success, otherwise False
   */
  bool getTrajectoryPosition(const int trajectory_index,
                             const int trajectory_dimension,
                             double& position) const;

  /*!
   * @param trajectory_index
   * @param trajectory_positions
   * @return True on success, otherwise False
   */
  bool getTrajectoryPosition(const int trajectory_index,
                             Eigen::VectorXd& trajectory_positions) const;

  /*!
   * @param trajectory_index
   * @param trajectory_dimension
   * @param velocity
   * @return True on success, otherwise False
   */
  bool getTrajectoryVelocity(const int trajectory_index,
                             const int trajectory_dimension,
                             double& velocity) const;

  /*!
   * @param trajectory_index
   * @param trajectory_dimension
   * @param acceleration
   * @return True on success, otherwise False
   */
  bool getTrajectoryAcceleration(const int trajectory_index,
                                 const int trajectory_dimension,
                                 double& acceleration) const;

  /*!
   * @return
   */
  double getSamplingFrequency() const;
  /*!
   *
   * @param sampling_frequency
   * @return True on success, otherwise False
   */
  bool setSamplingFrequency(const double sampling_frequency);

  /*!
   * @param trajectory_start
   * @return True on success, otherwise False
   */
  bool getStartPosition(Eigen::VectorXd& trajectory_start) const;

  /*!
   * @param trajectory_start
   * @return True on success, otherwise False
   */
  bool getEndPosition(Eigen::VectorXd& trajectory_end) const;

  /*!
   * @param other_trajectory
   * @param nmse_of_positions
   * @return True on success, otherwise False
   */
  bool computePositionNMSE(const Trajectory& other_trajectory,
                           Eigen::VectorXd& nmse_of_positions) const;

  /*! Rearange trajectory to match the list of variables provided
   * @param variable_names List of variables according to which the data traces of the
   * trajectory should be reordered.
   * @return True on success, otherwise False
   */
  bool rearange(const std::vector<std::string>& variable_names);

  /*!
   * @param variable_names
   * @return True on success, otherwise False
   */
  bool onlyKeep(const std::vector<std::string>& variable_names);

  /*!
   * @param trajectory
   * @param num_extra_points
   * @return
   */
  static bool blowUp(Trajectory& trajectory,
                     const int num_extra_points);

  /*!
   * @param num_extra_points
   * @return True on success, otherwise False
   */
  bool computeDerivatives(const int num_extra_points = 0);

  /*!
   * @param num_points Number of trajectory points which will be cropped at the beginning and ending
   * @return True on success, otherwise False
   */
  bool crop(const int num_trajectory_points);

  /*!
   * @param num_trajectory_points Number of trajectory points which will be cropped
   * @param crop_the_end If true, trajectory points will be cropped at the end
   * otherwise, at the beginning.
   * @return True on success, otherwise False
   */
  bool crop(const int num_trajectory_points, bool crop_the_end);

  /*! Returns string containing some information about the trajectory
   * @return string of information
   */
  std::string getInfo() const;

  /*! Adds positions, velocities, and accelerations to the end of the Trajectory.
   * @param trajectory_positions
   * @param trajectory_velocities
   * @param trajectory_accelerations
   * @param positions_only
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool add(const Eigen::VectorXd& trajectory_positons,
           const Eigen::VectorXd& trajectory_velocities,
           const Eigen::VectorXd& trajectory_accelerations,
           const bool positions_only = false);

  /*! Adds positions to the end of the Trajectory.
   * @param trajectory_positons
   * @param positions_only
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool add(const Eigen::VectorXd& trajectory_positons,
           const bool positions_only = true);

  /*!
   * @param trajectory_positons
   * @param positions_only
   * @return True on success, otherwise False
   * THIS FUNCTION IS NOT REAL-TIME FRIENDLY
   */
  bool add(const std::vector<double>& trajectory_positons,
           const bool positions_only = true);

  /*! Modifies this trajectory to include the other_trajectory
   * @param other_trajectory
   * @param verbose
   * @return True on success, otherwise False
   */
  bool combine(const Trajectory& other_trajectory, bool verbose = true);

  /*! Cuts the longer trajectory and modifies this trajectoy to include the other_trajectory
   * @param other_trajectory
   * @param verbose
   * @return True on success, otherwise False
   */
  bool cutAndCombine(const Trajectory& other_trajectory, bool verbose = true);

  /*! Cuts the longer trajectory
   * @param other_trajectory
   * @param verbose
   * @return True on success, otherwise False
   */
  bool cut(Trajectory& other_trajectory, bool verbose = true);

  /*!
   * @param other_trajectory
   * @return
   */
  bool append(const Trajectory& other_trajectory);

private:

  /*!
   */
  bool positions_only_;

  /*!
   */
  int trajectory_length_;
  int trajectory_dimension_;

  /*!
   */
  int index_to_last_trajectory_point_;

  /*!
   */
  double sampling_frequency_;

  /*!
   * mainly for debugging purpose (since it is actually redundant) to check for consistency
   * between trajectory_length, sampling_frequency and trajectory_duration
   */
  double trajectory_duration_;

  /*! This vectors contain the names of the variables
   * NOTE: only the position variable names is stored.
   */
  std::vector<std::string> variable_names_;
  std::vector<std::string> variable_units_;

  /*! This matrix actually contains the data of the trajectory.
   */
  Eigen::MatrixXd trajectory_positions_;
  Eigen::MatrixXd trajectory_velocities_;
  Eigen::MatrixXd trajectory_accelerations_;

  /*!
   * @param trajectory_point
   * @return True on success, otherwise False
   */
  bool check(const Eigen::VectorXd& trajectory_point) const;

  /*!
   * @return True on success, otherwise False
   */
  bool check() const;

  /*!
   * @param dimension_index
   * @return
   */
  bool isWithinDimensionBoundaries(const int dimension_index) const;

  /*!
   *
   * @param length_index
   * @return
   */
  bool isWithinLengthBoundaries(const int length_index) const;

  /*!
   * @param num_samples
   * @param num_dimensions
   * @param positions_only
   * @return True if Trajectory can hold data of length trajectory_length and with trajectory_dimension
   * REAL-TIME REQUIREMENTS
   */
  bool canHold(const int num_samples,
               const int num_dimensions,
               const bool positions_only) const;

  /*!
   */
  void clear();

  /*!
   * @param start
   * @param goal
   * @param duration
   * @param delta_t
   * @param next
   * @return True on success, otherwise False
   */
  bool calculateMinJerkNextStep(const Eigen::VectorXd &start,
                                const Eigen::VectorXd &goal,
                                const double duration,
                                const double delta_t,
                                Eigen::VectorXd &next);

};

/*! Abbreviation for convinience
 */
typedef boost::shared_ptr<Trajectory> TrajectoryPtr;

// Inline functions follow

// REAL-TIME REQUIREMENTS
inline bool Trajectory::canHold(const int trajectory_length,
                                const int trajectory_dimension,
                                const bool positions_only) const
{
  if (trajectory_length > trajectory_length_)
  {
    Logger::logPrintf("Trajectory is not big enough. Trajectory length is >%i<, however, >%i< is requested (Real-time violation).", Logger::ERROR, trajectory_length_, trajectory_length);
    return false;
  }
  if (trajectory_dimension > trajectory_dimension_)
  {
    Logger::logPrintf("Trajectory is not big enough. Trajectory dimension is >%i<, however, >%i< is requested (Real-time violation).", Logger::ERROR, trajectory_dimension_, trajectory_dimension);
    return false;
  }
  if (positions_only_ != positions_only)
  {
    Logger::logPrintf(positions_only_ && !positions_only, "Trajectory only contains positions, however, velocities and accelerations are required (Real-time violation).", Logger::ERROR);
    Logger::logPrintf(!positions_only_ && positions_only, "Trajectory contains positions, velocities, and accelerations, however, only positions are provided (Real-time violation).", Logger::ERROR);
    return false;
  }
  return true;
}
inline int Trajectory::getDimension() const
{
  assert(initialized_);
  return trajectory_dimension_;
}
inline int Trajectory::getNumTotalCapacity() const
{
  assert(initialized_);
  return trajectory_length_;
}
inline int Trajectory::getNumContainedSamples() const
{
  assert(initialized_);
  return index_to_last_trajectory_point_;
}
inline double Trajectory::getDuration() const
{
  assert(initialized_);
  return trajectory_duration_;
}
inline bool Trajectory::containsPositionsOnly() const
{
  assert(initialized_);
  return positions_only_;
}

inline double Trajectory::getSamplingFrequency() const
{
  assert(initialized_);
  return sampling_frequency_;
}
inline bool Trajectory::setSamplingFrequency(const double sampling_frequency)
{
  assert(initialized_);
  if (sampling_frequency <= 0)
  {
    Logger::logPrintf("Sampling frequency >%f< is invalid.", Logger::ERROR, sampling_frequency);
    return false;
  }
  sampling_frequency_ = sampling_frequency;
  trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
  return true;
}

inline bool Trajectory::check() const
{
  assert(initialized_);
  if (trajectory_length_ == 0)
  {
    Logger::logPrintf("Trajectory has length >%i<, cannot get start position.", Logger::ERROR,trajectory_length_);
    return false;
  }
  if (trajectory_dimension_ == 0)
  {
    Logger::logPrintf("Trajectory has dimension >%i<, cannot get start position.", Logger::ERROR,trajectory_dimension_);
    return false;
  }
  return true;
}

inline bool Trajectory::check(const Eigen::VectorXd& trajectory_point) const
{
  if (!check())
  {
    return false;
  }
  if (trajectory_point.size() != trajectory_dimension_)
  {
    Logger::logPrintf("Trajectory has >%i< dimensions, but the trajectory point is of size >%i<.", Logger::ERROR, trajectory_dimension_, trajectory_point.size());
    return false;
  }
  return true;
}

inline bool Trajectory::getStartPosition(Eigen::VectorXd& trajectory_start) const
{
  if (!check(trajectory_start))
  {
    return false;
  }
  trajectory_start = trajectory_positions_.row(0);
  return true;
}

inline bool Trajectory::getEndPosition(Eigen::VectorXd& trajectory_end) const
{
  if (!check(trajectory_end))
  {
    return false;
  }
  trajectory_end = trajectory_positions_.row(index_to_last_trajectory_point_ - 1);
  return true;
}

inline bool Trajectory::getTrajectoryPosition(const int trajectory_index,
                                              const int trajectory_dimension,
                                              double& position) const
{
  if (!check())
  {
    return false;
  }
  if (!isWithinDimensionBoundaries(trajectory_dimension) || !isWithinLengthBoundaries(trajectory_index))
  {
    return false;
  }

  position = trajectory_positions_(trajectory_index, trajectory_dimension);
  return true;
}

inline bool Trajectory::getTrajectoryPosition(const int trajectory_index,
                                              Eigen::VectorXd& trajectory_positions) const
{
  if (!check())
  {
    return false;
  }
  if (!isWithinLengthBoundaries(trajectory_index))
  {
    return false;
  }
  trajectory_positions = trajectory_positions_.row(trajectory_index);
  return true;
}

inline bool Trajectory::getTrajectoryVelocity(const int trajectory_index,
                                              const int trajectory_dimension,
                                              double& velocity) const
{
  if (positions_only_)
  {
    Logger::logPrintf("Cannot get trajectory velocity, since trajectory only contains positions.", Logger::ERROR);
    return false;
  }
  if (!check())
  {
    return false;
  }
  if (!isWithinDimensionBoundaries(trajectory_dimension) || !isWithinLengthBoundaries(trajectory_index))
  {
    return false;
  }

  velocity = trajectory_velocities_(trajectory_index, trajectory_dimension);
  return true;
}

inline bool Trajectory::getTrajectoryAcceleration(const int trajectory_index,
                                                  const int trajectory_dimension,
                                                  double& acceleration) const
{
  if (positions_only_)
  {
    Logger::logPrintf("Cannot get trajectory velocity, since trajectory only contains positions.", Logger::ERROR);
    return false;
  }
  if (!check())
  {
    return false;
  }
  if (!isWithinDimensionBoundaries(trajectory_dimension) || !isWithinLengthBoundaries(trajectory_index))
  {
    return false;
  }

  acceleration = trajectory_accelerations_(trajectory_index, trajectory_dimension);
  return true;
}

inline bool Trajectory::isWithinDimensionBoundaries(const int dimension_index) const
{
  if ((dimension_index < 0) || (dimension_index >= trajectory_dimension_))
  {
    Logger::logPrintf("Requested dimension >%i< is out of bound. Trajectory has dimension >%i<.", Logger::ERROR, dimension_index, trajectory_dimension_);
    return false;
  }
  return true;
}

inline bool Trajectory::isWithinLengthBoundaries(const int length_index) const
{
  if ((length_index < 0) || (length_index >= trajectory_length_))
  {
    Logger::logPrintf("Requested trajectory index >%i< is out of bound. Trajectory is of length >%i<.", Logger::ERROR, length_index, trajectory_length_);
    return false;
  }
  return true;
}

}

#endif /* TRAJECTORY_BASE_H_ */
