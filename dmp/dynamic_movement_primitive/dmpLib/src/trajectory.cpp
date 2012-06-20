/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		trajectory.cpp

 \author	Peter Pastor, Mrinal Kalakrishnan
 \date		Nov 4, 2010

 *********************************************************************/

// system includes
#include <math.h>
#include <sstream>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

// local include
#include <dmp_lib/trajectory.h>
#include <dmp_lib/logger.h>

// 32 bit word byte/word swap macros
#define LLSB(x) ((x) & 0xff)
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x) (((x) >> 24) & 0xff)
#define LONGSWAP(x) ((LLSB(x) << 24) | (LNLSB(x) << 16)| (LNMSB(x) << 8) | (LMSB(x)))

using namespace Eigen;
using namespace std;

namespace dmp_lib
{

static const int POS_VEL_ACC = 3;
static const int POS = 0;
static const int VEL = 1;
static const int ACC = 2;

static const int FILENAME_LENGTH = 200;
static const int MAX_VARNAME_LENGTH = 20;

static const char* DEFAULT_VARIABLE_NAME = "name";
static const char* DEFAULT_VARIABLE_UNIT = "unit";

bool Trajectory::initialize(const std::vector<std::string>& variable_names,
                            const double sampling_frequency,
                            const bool positions_only,
                            const int trajectory_length)
{
  Logger::logPrintf(positions_only, "Initializing position trajectory with >%i< dimensions, with a sampling frequency of >%.1f< Hz and maximal length of >%i< samples.",
                    Logger::DEBUG, (int)variable_names.size(), sampling_frequency, trajectory_length);
  Logger::logPrintf(!positions_only, "Initializing position, velocity and acceleration trajectory with >%i< dimensions, with a sampling frequency of >%.1f< Hz and maximal length of >%i< samples.",
                    Logger::DEBUG, (int)variable_names.size(), sampling_frequency, trajectory_length);
  if (trajectory_length <= 0 || trajectory_length > ABSOLUTE_MAX_TRAJECTORY_LENGTH)
  {
    Logger::logPrintf("Trajectory length >%i< is out of bounds (0..%i]. Cannot initialize trajectory.", Logger::ERROR, trajectory_length, ABSOLUTE_MAX_TRAJECTORY_LENGTH);
    return (initialized_ = false);
  }
  if (variable_names.empty())
  {
    Logger::logPrintf("Cannot initialize trajectory with no variable names.", Logger::ERROR);
    return (initialized_ = false);
  }
  const int trajectory_dimension = static_cast<int> (variable_names.size());

  if (trajectory_dimension > ABSOLUTE_MAX_TRAJECTORY_DIMENSION)
  {
    Logger::logPrintf("Maximum allowed trajectory dimension is >%i<. Cannot set it to >%i<.", Logger::ERROR, ABSOLUTE_MAX_TRAJECTORY_DIMENSION, trajectory_dimension);
    return (initialized_ = false);
  }
  if(sampling_frequency <= 0.0)
  {
    Logger::logPrintf("Invalid sampling frequency >%f<. Cannot initialize trajectory.", Logger::ERROR, sampling_frequency);
    return (initialized_ = false);
  }
  trajectory_dimension_ = trajectory_dimension;
  variable_names_ = variable_names;
  trajectory_length_ = trajectory_length;
  positions_only_ = positions_only;
  index_to_last_trajectory_point_ = 0;
  trajectory_duration_ = 0.0;
  sampling_frequency_ = sampling_frequency;
  trajectory_positions_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  if (!positions_only_)
  {
    trajectory_velocities_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
    trajectory_accelerations_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  }
  return (initialized_ = true);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string> & variable_names,
                                       const double sampling_frequency,
                                       const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       const int num_samples,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  if(!initialize(variable_names, sampling_frequency, positions_only, trajectory_length))
  {
    return false;
  }
  return setMinJerk(start, goal, num_samples);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string> & variable_names,
                                       const double sampling_frequency,
                                       const std::vector<double>& start,
                                       const std::vector<double>& goal,
                                       const int num_samples,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  return initializeWithMinJerk(variable_names,
                               sampling_frequency,
                               VectorXd::Map(&start[0], start.size()),
                               VectorXd::Map(&goal[0], goal.size()),
                               num_samples,
                               positions_only, trajectory_length);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string> & variable_names,
                                       const double sampling_frequency,
                                       const double duration,
                                       const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  int num_samples = duration * sampling_frequency;
  return initializeWithMinJerk(variable_names, sampling_frequency, start, goal, num_samples, positions_only, trajectory_length);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string> & variable_names,
                                       const double sampling_frequency,
                                       const double duration,
                                       const std::vector<double>& start,
                                       const std::vector<double>& goal,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  return initializeWithMinJerk(variable_names, sampling_frequency, duration, VectorXd::Map(&start[0], start.size()), VectorXd::Map(&goal[0], goal.size()),
                               positions_only, trajectory_length);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string>& variable_names,
                                       const double sampling_frequency,
                                       const std::vector<Eigen::VectorXd>& waypoints,
                                       const std::vector<int>& num_samples,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  if(!initialize(variable_names, sampling_frequency, positions_only, trajectory_length))
  {
    return false;
  }
  return setMinJerk(waypoints, num_samples);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string>& variable_names,
                           const double sampling_frequency,
                           const std::vector<std::vector<double> >& waypoints,
                           const std::vector<int>& num_samples,
                           const bool positions_only,
                           const int trajectory_length)
{
  std::vector<VectorXd> eigen_waypoints;
  for(int i=0; i<(int)waypoints.size(); ++i)
  {
    VectorXd waypoint = VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return initializeWithMinJerk(variable_names, sampling_frequency, eigen_waypoints, num_samples, positions_only, trajectory_length);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string>& variable_names,
                                       const double sampling_frequency,
                                       const std::vector<double>& durations,
                                       const std::vector<Eigen::VectorXd>& waypoints,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  std::vector<int> num_samples;
  for(int i=0; i<(int)durations.size(); ++i)
  {
    num_samples.push_back(durations[i] * sampling_frequency_);
  }
  return initializeWithMinJerk(variable_names, sampling_frequency, waypoints, num_samples, positions_only, trajectory_length);
}

bool Trajectory::initializeWithMinJerk(const std::vector<std::string>& variable_names,
                                       const double sampling_frequency,
                                       const std::vector<double>& durations,
                                       const std::vector<std::vector<double> >& waypoints,
                                       const bool positions_only,
                                       const int trajectory_length)
{
  std::vector<VectorXd> eigen_waypoints;
  for(int i=0; i<(int)waypoints.size(); ++i)
  {
    VectorXd waypoint = VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return initializeWithMinJerk(variable_names, sampling_frequency, durations, eigen_waypoints, positions_only, trajectory_length);
}

bool Trajectory::setMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const int num_samples)
{
  if(!initialized_)
  {
    Logger::logPrintf("Trajectory not initialized. Cannot set minimum jerk trajectory.", Logger::ERROR);
    return false;
  }
  if (trajectory_dimension_ != start.size() || trajectory_dimension_ != goal.size())
  {
      Logger::logPrintf("Trajectory dimension >%i< does not match size of provided start >%i< or goal >%i<.",
                        Logger::ERROR, trajectory_dimension_, start.size(), goal.size());
      return false;
  }
  if(trajectory_length_ < num_samples)
  {
    Logger::logPrintf("Cannot set minimum jerk trajectory with >%i< samples because trajectory is only of length >%i<.",
                      Logger::ERROR, num_samples, trajectory_length_);
    return false;
  }

  double delta_t = static_cast<double>(1.0) / sampling_frequency_;
  double duration = num_samples * delta_t;

  Logger::logPrintf("Setting minimum jerk trajectory with >%i< samples using a sampling frequency of >%.1f< Hz and a duration of >%.1f< seconds.",
                    Logger::DEBUG, num_samples, sampling_frequency_, duration);

  VectorXd tmp_current = VectorXd::Zero(POS_VEL_ACC);
  VectorXd tmp_goal = VectorXd::Zero(POS_VEL_ACC);
  VectorXd tmp_next = VectorXd::Zero(POS_VEL_ACC);

  VectorXd next_position = VectorXd::Zero(trajectory_dimension_);
  VectorXd next_velocity = VectorXd::Zero(trajectory_dimension_);
  VectorXd next_acceleration = VectorXd::Zero(trajectory_dimension_);

  // add first trajectory point
  next_position = start;
  if (!add(next_position, next_velocity, next_acceleration, positions_only_))
  {
    Logger::logPrintf("Could not add first trajectory point. Cannot set minimum jerk trajectory.", Logger::ERROR);
    return false;
  }

  for (int i = 1; i < num_samples; i++)
  {
    for (int j = 0; j < trajectory_dimension_; j++)
    {
      // update the current state
      tmp_current(POS) = next_position(j);
      tmp_current(VEL) = next_velocity(j);
      tmp_current(ACC) = next_acceleration(j);
      tmp_goal(0) = goal(j);

      if (!calculateMinJerkNextStep(tmp_current, tmp_goal, duration - ((i - 1) * delta_t), delta_t, tmp_next))
      {
        Logger::logPrintf("Could not compute next minimum jerk trajectory point. Cannot set minimum jerk trajectory.", Logger::ERROR);
        return false;
      }

      next_position(j) = tmp_next(POS);
      next_velocity(j) = tmp_next(VEL);
      next_acceleration(j) = tmp_next(ACC);
    }

    if (!add(next_position, next_velocity, next_acceleration, positions_only_))
    {
      Logger::logPrintf("Could not add first trajectory point. Cannot set minimum jerk trajectory.", Logger::ERROR);
      return false;
    }
  }
  index_to_last_trajectory_point_ = num_samples;
  return true;
}

bool Trajectory::setMinJerk(const std::vector<double>& start, const std::vector<double>& goal, const int num_samples)
{
  return setMinJerk(VectorXd::Map(&start[0], start.size()), VectorXd::Map(&goal[0], goal.size()), num_samples);
}

bool Trajectory::setMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double duration)
{
  int num_samples = duration * sampling_frequency_;
  return setMinJerk(start, goal, num_samples);
}

bool Trajectory::setMinJerk(const std::vector<double>& start, const std::vector<double>& goal, const double duration)
{
  return setMinJerk(VectorXd::Map(&start[0], start.size()), VectorXd::Map(&goal[0], goal.size()), duration);
}

bool Trajectory::setMinJerk(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<int>& num_samples)
{
  // error checking
  if(waypoints.size() != num_samples.size()+1)
  {
    Logger::logPrintf("Number of waypoints >%i< does not match number of num_samples provided >%i<. Cannot set minimum jerk trajectory.",
                      Logger::ERROR, (int)waypoints.size(), (int)num_samples.size()+1);
    return false;
  }
  if(waypoints.size() < 2)
  {
    Logger::logPrintf("Number of waypoints >%i< provided must be greater or equal 2. Cannot set minimum jerk trajectory.",
                      Logger::ERROR, (int)waypoints.size());
    return false;
  }

  for (int i = 0; i < (int)waypoints.size()-1; ++i)
  {
    Trajectory minimum_jerk;
    if(!minimum_jerk.initialize(variable_names_, sampling_frequency_, positions_only_))
    {
      Logger::logPrintf("Could not initialize trajectory. Cannot set minimum jerk trajectory from waypoints.", Logger::FATAL);
      return false;
    }
    // TODO: fix this
    int extra = 0;
    if(i>0 && i<(int)waypoints.size()-2)
      extra = 1;
    if(!minimum_jerk.setMinJerk(waypoints[i], waypoints[i+1], num_samples[i] + extra))
    {
      Logger::logPrintf("Could not set minimum jerk trajectory. Cannot set minimum jerk trajectory from waypoints.", Logger::FATAL);
      return false;
    }
    if(!append(minimum_jerk))
    {
      Logger::logPrintf("Could not append jerk trajectory. Cannot set minimum jerk trajectory from waypoints.", Logger::FATAL);
      return false;
    }
  }
  return true;
}

bool Trajectory::setMinJerk(const std::vector<std::vector<double> >& waypoints, const std::vector<int>& num_samples)
{
  std::vector<VectorXd> eigen_waypoints;
  for (int i = 0; i < (int)waypoints.size(); ++i)
  {
    VectorXd waypoint = VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return setMinJerk(eigen_waypoints, num_samples);
}

bool Trajectory::setMinJerk(const std::vector<Eigen::VectorXd>& waypoints, const std::vector<double>& durations)
{
  std::vector<int> num_samples;
  for(int i=0; i<(int)durations.size(); ++i)
  {
    num_samples.push_back(durations[i] * sampling_frequency_);
  }
  return setMinJerk(waypoints, num_samples);
}

bool Trajectory::setMinJerk(const std::vector<std::vector<double> >& waypoints, const std::vector<double>& durations)
{
  std::vector<VectorXd> eigen_waypoints;
  for(int i=0; i<(int)waypoints.size(); ++i)
  {
    VectorXd waypoint = VectorXd::Map(&(waypoints[i])[0], waypoints[i].size());
    eigen_waypoints.push_back(waypoint);
  }
  return setMinJerk(waypoints, durations);
}

bool Trajectory::readFromCLMCFile(const string& file_name,
                                  const bool positions_only)
{
  std::vector<std::string> empty;
  return readFromCLMCFile(file_name, empty, positions_only, false);
}

bool Trajectory::readFromCLMCFile(const string& file_name,
                                  const vector<string>& variable_names,
                                  const bool positions_only,
                                  const bool use_variable_names)
{

  FILE *fp;
  // open the file, and parse the parameters
  if ((fp = fopen(file_name.c_str(), "r")) == NULL)
  {
    Logger::logPrintf("Cannot fopen file >%s<.", Logger::ERROR, file_name.c_str());
    return false;
  }

  int tmp_trajectory_type, tmp_num_rows, tmp_num_cols;
  double tmp_sampling_frequency;
  if (fscanf(fp, "%d %d %d %lf", &tmp_trajectory_type, &tmp_num_cols, &tmp_num_rows, &tmp_sampling_frequency) != 4)
  {
    Logger::logPrintf("Could not read/parse header.", Logger::ERROR);
    fclose(fp);
    return false;
  }

  // checking whether values make sense
  if ((tmp_num_cols <= 0) || (tmp_num_rows <= 0))
  {
    Logger::logPrintf("Values for number of columns >%i< and rows >%i< are negative.", Logger::ERROR, tmp_num_cols, tmp_num_rows);
    fclose(fp);
    return false;
  }
  if ((tmp_num_cols > ABSOLUTE_MAX_TRAJECTORY_DIMENSION) || (tmp_num_rows > ABSOLUTE_MAX_TRAJECTORY_LENGTH))
  {
    Logger::logPrintf("Values for number of columns >%i< and rows >%i< are out of bound (%i x %i).", Logger::ERROR, tmp_num_cols, tmp_num_rows,
           ABSOLUTE_MAX_TRAJECTORY_DIMENSION, ABSOLUTE_MAX_TRAJECTORY_LENGTH);
    fclose(fp);
    return false;
  }
  if (tmp_sampling_frequency <= 0)
  {
    Logger::logPrintf("Read implausible sampling frequency >%.1f<.", Logger::ERROR, tmp_sampling_frequency);
    fclose(fp);
    return false;
  }

  vector<int> position_variable_indices;
  vector<int> velocity_variable_indices;
  vector<int> acceleration_variable_indices;

  variable_names_.clear();
  variable_units_.clear();

  vector<string> file_variable_names;
  vector<string> file_variable_units;
  for (int j = 0; j < tmp_num_cols; ++j)
  {
    char tmp_name[MAX_VARNAME_LENGTH];
    char tmp_unit[MAX_VARNAME_LENGTH];
    if (fscanf(fp, "%s %s", tmp_name, tmp_unit) != 2)
    {
      Logger::logPrintf("Cannot read variable names and units.", Logger::ERROR);
      fclose(fp);
      return false;
    }
    file_variable_names.push_back(tmp_name);
    file_variable_units.push_back(tmp_unit);
  }

  vector<string> variable_names_list = variable_names;
  if(!use_variable_names)
  {
    // read ALL variables...
    variable_names_list = file_variable_names;
  }

  for (int i = 0; i < (int)variable_names_list.size(); ++i)
  {
    bool variable_name_found = false;
    for (int j = 0; j < (int)file_variable_names.size(); ++j)
    {
      if (variable_names_list[i].compare(file_variable_names[j]) == 0)
      {
        variable_name_found = true;
        position_variable_indices.push_back(j);
        variable_names_.push_back(file_variable_names[j]);
        variable_units_.push_back(file_variable_units[j]);
        Logger::logPrintf("Read %s [%s].", Logger::DEBUG, variable_names_.back().c_str(), variable_units_.back().c_str());

        if (!positions_only)
        {
          string velocity_variable_name = variable_names_list[i] + "d";
          bool velocity_variable_name_found = false;
          for (int s = 0; s < (int)file_variable_names.size(); ++s)
          {
            if (velocity_variable_name.compare(file_variable_names[s]) == 0)
            {
              velocity_variable_name_found = true;
            }
          }
          if(!velocity_variable_name_found)
          {
            Logger::logPrintf("Could not find variable >%s<. Maybe use position only option.", Logger::ERROR, velocity_variable_name.c_str());
            return false;
          }

          string acceleration_variable_name = variable_names_list[i] + "dd";
          bool acceleration_variable_name_found = false;
          for (int s = 0; s < (int)file_variable_names.size(); ++s)
          {
            if (acceleration_variable_name.compare(file_variable_names[s]) == 0)
            {
              acceleration_variable_name_found = true;
            }
          }
          if(!acceleration_variable_name_found)
          {
            Logger::logPrintf("Could not find variable >%s<. Maybe use position only option.", Logger::ERROR, acceleration_variable_name.c_str());
            return false;
          }

          for (int m = 0; m < (int)file_variable_names.size(); ++m)
          {
            if (velocity_variable_name.compare(file_variable_names[m]) == 0)
            {
              velocity_variable_indices.push_back(m);
            }
            if (acceleration_variable_name.compare(file_variable_names[m]) == 0)
            {
              acceleration_variable_indices.push_back(m);
            }
          }
        }
        if (variable_name_found)
        {
          break;
        }
      }
    }
    if (!variable_name_found)
    {
      Logger::logPrintf("Could not find variable named >%s< in trajectory file >%s<.", Logger::ERROR, variable_names_list[i].c_str(), file_name.c_str());
      fclose(fp);
      return false;
    }
  }

  // there are two extra blank chars at the end of the block and a line return which we must account for
  fgetc(fp);
  fgetc(fp);
  fgetc(fp);

  // read file into a buffer
  //  float trajectory_data[tmp_num_rows][tmp_num_cols];
  //  if (fread(&(trajectory_data[0][0]), sizeof(float), tmp_num_rows * tmp_num_cols, fp) != (unsigned)(tmp_num_rows * tmp_num_cols))
  //  {
  //    Logger::logPrintf("Cannot read trajectory data.", Logger::ERROR);
  //    fclose(fp);
  //    return false;
  //  }

  // Allocate memory on the heap to hold really biiiig data
  Logger::logPrintf("Reading >%i< rows and >%i< cols (%.2f MBytes).", Logger::INFO, tmp_num_rows, tmp_num_cols, (double)(4 * tmp_num_rows * tmp_num_cols)/(double)1000000);
  float** buffer;
  buffer = (float**) calloc((size_t)tmp_num_rows, sizeof(float*));
  float* chunk;
  chunk = (float *) calloc((size_t) tmp_num_rows * tmp_num_cols, sizeof(float));
  for (int i = 0; i < tmp_num_rows; ++i)
  {
    buffer[i] = (float*) &(chunk[i*tmp_num_cols]);
  }
  if (fread(buffer[0], sizeof(float), tmp_num_rows * tmp_num_cols, fp) != (unsigned)(tmp_num_rows * tmp_num_cols))
  {
    Logger::logPrintf("Cannot read trajectory data of size >%i< x >%i<.", Logger::ERROR, tmp_num_rows, tmp_num_cols);
    fclose(fp);
    return false;
  }
  fclose(fp);

  // convert little-endian to big-endian
  // TODO: check this on different platforms
  int aux;
  for (int i = 0; i < tmp_num_rows; i++)
  {
    for (int j = 0; j < tmp_num_cols; j++)
    {
      // aux = LONGSWAP(*((int *)&(trajectory_data->at(i)[j])));
      aux = LONGSWAP(*((int *)&(buffer[i][j])));
      // trajectory_data[i][j] = *((float *)&aux);
      buffer[i][j] = *((float *)&aux);
    }
  }

  // initialize trajectory and allocate memory to hold the trajectory
  if (!initialize(variable_names_list, tmp_sampling_frequency, positions_only, tmp_num_rows))
  {
    Logger::logPrintf("Could not initialize trajectory. Reading from file failed.", Logger::ERROR);
    return false;
  }

  for (int i = 0; i < trajectory_length_; i++)
  {
    for (int j = 0; j < trajectory_dimension_; j++)
    {
      // trajectory_positions_(i, j) = static_cast<double> (trajectory_data[i][position_variable_indices[j]]);
      trajectory_positions_(i, j) = static_cast<double> (buffer[i][position_variable_indices[j]]);
      if (!positions_only)
      {
        // trajectory_velocities_(i, j) = static_cast<double> (trajectory_data[i][velocity_variable_indices[j]]);
        // trajectory_accelerations_(i, j) = static_cast<double> (trajectory_data[i][acceleration_variable_indices[j]]);
        trajectory_velocities_(i, j) = static_cast<double> (buffer[i][velocity_variable_indices[j]]);
        trajectory_accelerations_(i, j) = static_cast<double> (buffer[i][acceleration_variable_indices[j]]);
      }
    }
  }
  index_to_last_trajectory_point_ = trajectory_length_;
  trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
  positions_only_ = positions_only;

  string all_variable_names = "";
  for (int i = 0; i < (int)variable_names_list.size(); ++i)
  {
    all_variable_names.append(variable_names_list[i]);
    if (i + 1 < (int)variable_names_list.size())
    {
      all_variable_names.append(" ");
    }
  }

  Logger::logPrintf(positions_only, "Read position trajectory containing >%i< variables with each >%i< data points.", Logger::INFO, (int)variable_names_list.size(),
                    trajectory_length_);
  Logger::logPrintf(!positions_only, "Read trajectory containing position, velocity, and acceleration of >%i< variables with each >%i< data points.",
                    Logger::INFO, (int)variable_names_list.size(), trajectory_length_);
  Logger::logPrintf("Variable names are >%s<.", Logger::DEBUG, all_variable_names.c_str());

  // deallocate
  free(buffer[0]);
  free(buffer);

  return true;
}

bool Trajectory::writeToCLMCFile(const std::string& file_name,
                                 const bool positions_only) const
{
  if (!positions_only && positions_only_)
  {
    Logger::logPrintf("Only positions are contained in trajectory, cannot write more into >%s<.", Logger::ERROR, file_name.c_str());
    return false;
  }

  Logger::logPrintf(positions_only, "Writing trajectory with >%i< position traces and >%i< samples to clmc file >%s<.",
                    Logger::INFO, trajectory_dimension_, index_to_last_trajectory_point_, file_name.c_str());
  Logger::logPrintf(!positions_only, "Writing trajectory with >%i< position, velocity and acceleration traces and >%i< samples to clmc file >%s<.",
                    Logger::INFO, trajectory_dimension_, index_to_last_trajectory_point_, file_name.c_str());

  // open the file, and write all data
  FILE *fp;
  if ((fp = fopen(file_name.c_str(), "w")) == NULL)
  {
    Logger::logPrintf("Cannot fopen file >%s< : %s.", Logger::ERROR, file_name.c_str(), strerror(errno));
    return false;
  }

  int num_dimensions;
  if (positions_only)
  {
    num_dimensions = trajectory_dimension_;
  }
  else
  {
    num_dimensions = trajectory_dimension_ * POS_VEL_ACC;
  }

  // create a floating point buffer and copy all data over
  Logger::logPrintf("Creating buffer of size %i x %i to hold temporary data.", Logger::DEBUG, index_to_last_trajectory_point_, num_dimensions);
  boost::shared_ptr<vector<vector<float> > > buffer(new vector<vector<float> >(index_to_last_trajectory_point_, vector<float>(num_dimensions)) );
  Logger::logPrintf("Buffer created.", Logger::DEBUG);
  for (int i = 0; i < index_to_last_trajectory_point_; i++)
  {
    for (int j = 0; j < num_dimensions; j++)
    {
      (*buffer)[i][j] = 0.0;
    }
  }

  for (int i = 0; i < index_to_last_trajectory_point_; i++)
  {
    if (positions_only)
    {
      for (int j = 0; j < num_dimensions; j++)
      {
        (*buffer)[i][j] = trajectory_positions_(i, j);
      }
    }
    else
    {
      for (int j = 0; j < num_dimensions / POS_VEL_ACC; j++)
      {
        (*buffer)[i][j * POS_VEL_ACC + POS] = trajectory_positions_(i, j);
        (*buffer)[i][j * POS_VEL_ACC + VEL] = trajectory_velocities_(i, j);
        (*buffer)[i][j * POS_VEL_ACC + ACC] = trajectory_accelerations_(i, j);
      }
    }
  }

  // write the the (*buffer) size, the number of columns, the sampling time, the column names and units
  if (sampling_frequency_ <= 0.0)
  {
    Logger::logPrintf("Sampling frequency >%.1f< is invalid.", Logger::ERROR, sampling_frequency_);
    return false;
  }
  fprintf(fp, "%d  %d  %d  %f\n", num_dimensions * index_to_last_trajectory_point_, num_dimensions, index_to_last_trajectory_point_, sampling_frequency_);

  for (int i = 0; i < (int)variable_names_.size(); i++)
  {

    string position_variable_name = variable_names_[i];
    if (position_variable_name.empty())
    {
      Logger::logPrintf("Variable name is empty.", Logger::WARN);
      char tmp[MAX_VARNAME_LENGTH];
      sprintf(tmp, "%s_%d", DEFAULT_VARIABLE_NAME, i);
      position_variable_name.assign(tmp);
    }
    fprintf(fp, "%s  ", position_variable_name.c_str());

    if (variable_units_.size() == variable_names_.size())
    {
      string position_variable_unit = variable_units_[i];
      if (position_variable_unit.empty())
      {
        Logger::logPrintf("Variable unit is empty.", Logger::WARN);
        char tmp[MAX_VARNAME_LENGTH];
        sprintf(tmp, "%s_%d", DEFAULT_VARIABLE_UNIT, i);
        position_variable_unit.assign(tmp);
      }
      fprintf(fp, "%s  ", position_variable_unit.c_str());
    }
    else
    {
      fprintf(fp, "-  ");
    }

    if (!positions_only)
    {
      string velocity_variable_name = variable_names_[i] + "d";
      if (velocity_variable_name.empty())
      {
        Logger::logPrintf("Variable name is empty.", Logger::WARN);
        char tmp[MAX_VARNAME_LENGTH];
        sprintf(tmp, "%s_%dd", DEFAULT_VARIABLE_NAME, i);
        velocity_variable_name.assign(tmp);
      }
      fprintf(fp, "%s  ", velocity_variable_name.c_str());

      if (variable_units_.size() == variable_names_.size())
      {
        string velocity_variable_unit = variable_units_[i] + "/s";
        if (variable_units_[i].empty())
        {
          Logger::logPrintf("Variable unit is empty.", Logger::WARN);
          char tmp[MAX_VARNAME_LENGTH];
          sprintf(tmp, "%s_%d/s", DEFAULT_VARIABLE_UNIT, i);
          velocity_variable_unit.assign(tmp);
        }
        fprintf(fp, "%s  ", velocity_variable_unit.c_str());
      }
      else
      {
        fprintf(fp, "-  ");
      }

      string acceleration_variable_name = variable_names_[i] + "dd";
      if (acceleration_variable_name.empty())
      {
        Logger::logPrintf("Variable name is empty.", Logger::WARN);
        char tmp[MAX_VARNAME_LENGTH];
        sprintf(tmp, "%s_%ddd", DEFAULT_VARIABLE_UNIT, i);
        acceleration_variable_name.assign(tmp);
      }
      fprintf(fp, "%s  ", acceleration_variable_name.c_str());

      if (variable_units_.size() == variable_names_.size())
      {
        string acceleration_variable_unit = variable_units_[i] + "/s^2";
        if (variable_units_[i].empty())
        {
          Logger::logPrintf("Variable unit is empty.", Logger::WARN);
          char tmp[MAX_VARNAME_LENGTH];
          sprintf(tmp, "%s_%d/s^2", DEFAULT_VARIABLE_UNIT, i);
          acceleration_variable_unit.assign(tmp);
        }
        fprintf(fp, "%s  ", acceleration_variable_unit.c_str());
      }
      else
      {
        fprintf(fp, "-  ");
      }
    }

  }
  fprintf(fp, "\n");

  // convert little-endian to big-endian
  int aux;
  Logger::logPrintf("Byteswap active.", Logger::DEBUG);
  for (int i = 0; i < index_to_last_trajectory_point_; ++i)
  {
    for (int j = 0; j < num_dimensions; ++j)
    {
      aux = LONGSWAP(*((int *) &((*buffer)[i][j])));
      (*buffer)[i][j] = *((float *)&aux);
    }
  }

  // TODO: valgrind says "Syscall param write(buf) points to uninitialised byte(s)" change this !!!
  for (int i = 0; i < index_to_last_trajectory_point_; ++i)
  {
    if (fwrite(&((*buffer)[i][0]), sizeof(float), num_dimensions, fp) != (unsigned)num_dimensions)
    {
      Logger::logPrintf("Cannot fwrite trajectory data.", Logger::ERROR);
      return false;
    }
  }
  fclose(fp);

  return true;
}

bool Trajectory::rearange(const vector<string>& variable_names_order)
{

  if (variable_names_order.size() != variable_names_.size())
  {
    Logger::logPrintf("Number of variable names >%i< does not match number of variables >%i< contained in trajectory. Cannot rearange trajectory.",
                      Logger::ERROR, (int)variable_names_order.size(), (int)variable_names_.size());
    return false;
  }

  vector<int> variable_index;
  for (int i = 0; i < (int)variable_names_order.size(); ++i)
  {
    bool found = false;
    for (int j = 0; j < (int)variable_names_.size(); ++j)
    {

      if (variable_names_order[i].compare(variable_names_[j]) == 0)
      {
        found = true;
        variable_index.push_back(j);
      }
      if (found)
      {
        break;
      }
    }
    if (!found)
    {
      Logger::logPrintf("Could not find variable named >%s< in trajectory.", Logger::ERROR, variable_names_order[i].c_str());
      return false;
    }
  }

  MatrixXd tmp_trajectory_positions = trajectory_positions_;
  MatrixXd tmp_trajectory_velocities = trajectory_velocities_;
  MatrixXd tmp_trajectory_accelerations = trajectory_accelerations_;

  for (int i = 0; i < (int)variable_index.size(); ++i)
  {
    trajectory_positions_.col(i) = tmp_trajectory_positions.col(variable_index[i]);
    if (!positions_only_)
    {
      trajectory_velocities_.col(i) = tmp_trajectory_velocities.col(variable_index[i]);
      trajectory_accelerations_.col(i) = tmp_trajectory_accelerations.col(variable_index[i]);
    }
  }

  // assign new variable names
  variable_names_ = variable_names_order;

  return true;
}

bool Trajectory::onlyKeep(const vector<string>& variable_names)
{
  if (variable_names.size() > variable_names_.size())
  {
    Logger::logPrintf("Number of variable names >%i< must be less than number of variables >%i< contained in trajectory. Cannot reduce trajectory dimension.",
                      Logger::ERROR, (int)variable_names.size(), (int)variable_names_.size());
    return false;
  }

  vector<int> variable_index;
  for (int i = 0; i < (int)variable_names.size(); ++i)
  {
    Logger::logPrintf("keeping: %s", Logger::INFO, variable_names[i].c_str());
    bool found = false;
    for (int j = 0; j < (int)variable_names_.size(); ++j)
    {
      Logger::logPrintf("current: %s", Logger::INFO, variable_names_[i].c_str());
      if (variable_names[i].compare(variable_names_[j]) == 0)
      {
        found = true;
        variable_index.push_back(j);
      }
      if (found)
      {
        break;
      }
    }
    if (!found)
    {
      Logger::logPrintf("Could not find variable named >%s< in trajectory.", Logger::ERROR, variable_names[i].c_str());
      return false;
    }
  }

  MatrixXd tmp_trajectory_positions = trajectory_positions_;
  MatrixXd tmp_trajectory_velocities = trajectory_velocities_;
  MatrixXd tmp_trajectory_accelerations = trajectory_accelerations_;

  // shrink trajectory dimension
  trajectory_dimension_ = variable_names.size();
  trajectory_positions_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  if (!positions_only_)
  {
    trajectory_velocities_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
    trajectory_accelerations_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  }

  for (int i = 0; i < (int)variable_index.size(); ++i)
  {
    trajectory_positions_.col(i) = tmp_trajectory_positions.col(variable_index[i]);
    if (!positions_only_)
    {
      trajectory_velocities_.col(i) = tmp_trajectory_velocities.col(variable_index[i]);
      trajectory_accelerations_.col(i) = tmp_trajectory_accelerations.col(variable_index[i]);
    }
  }

  // assign new variable names
  variable_names_ = variable_names;
  return true;
}

bool Trajectory::setVariableNames(const vector<string>& variable_names)
{
  if (static_cast<int> (variable_names.size()) != trajectory_dimension_)
  {
    Logger::logPrintf("Number of variable names is >%i<, but should be >%i<.", Logger::ERROR, (int)variable_names.size(), trajectory_dimension_);
    return false;
  }
  variable_names_ = variable_names;
  return true;
}

vector<string> Trajectory::getVariableNames() const
{
  return variable_names_;
}

bool Trajectory::containVariables(const vector<string>& variable_names) const
{
  // If the (pure) number of variable names is greater than the ones in the trajectory
  // we know that not all are contained
  if (variable_names.size() > variable_names_.size())
  {
    return false;
  }

  for (vector<string>::const_iterator vci = variable_names.begin(); vci != variable_names.end(); ++vci)
  {
    bool found = false;
    for (vector<string>::const_iterator vcj = variable_names_.begin(); vcj != variable_names_.end(); ++vcj)
    {
      if (vci->compare(*vcj) == 0) // Name match
      {
        found = true;
      }
    }
    if (!found)
    {
      Logger::logPrintf("Variable named >%s< is not contained in Trajectory.", Logger::ERROR, vci->c_str());
      return false;
    }
  }
  return true;
}

bool Trajectory::getTrajectoryPoint(const int trajectory_index,
                                    VectorXd& trajectory_point) const
{
  if(!isWithinLengthBoundaries(trajectory_index))
  {
    Logger::logPrintf("Trajectory index >%i< is out of bound [0..%i].", Logger::ERROR, trajectory_index, trajectory_length_);
    return false;
  }

  if(positions_only_)
  {
    if(trajectory_point.size() != trajectory_dimension_)
    {
      Logger::logPrintf("Trajectory point size provided is >%i<, but should be >%i<.", Logger::ERROR, (int)trajectory_point.size(), trajectory_dimension_);
      return false;
    }
    trajectory_point = trajectory_positions_.row(trajectory_index);
  }
  else
  {
    if(trajectory_point.size() != trajectory_dimension_ * POS_VEL_ACC)
    {
      Logger::logPrintf("Trajectory point size provided is >%i<, but should be >%i<.", Logger::ERROR, (int)trajectory_point.size(), trajectory_dimension_ * POS_VEL_ACC);
      return false;
    }
    for (int i = 0; i < trajectory_dimension_; ++i)
    {
      trajectory_point(i * POS_VEL_ACC + POS) = trajectory_positions_(trajectory_index, i);
      trajectory_point(i * POS_VEL_ACC + VEL) = trajectory_velocities_(trajectory_index, i);
      trajectory_point(i * POS_VEL_ACC + ACC) = trajectory_accelerations_(trajectory_index, i);
    }
  }
  return true;
}

bool Trajectory::setTrajectoryPoint(const int trajectory_index,
                                    const VectorXd& trajectory_point)
{
  if(!isWithinLengthBoundaries(trajectory_index))
  {
    Logger::logPrintf("Trajectory index >%i< is out of bound [0..%i].", Logger::ERROR, trajectory_index, trajectory_length_);
    return false;
  }

  if(positions_only_)
  {
    if(trajectory_point.size() != trajectory_dimension_)
    {
      Logger::logPrintf("Trajectory point size provided is >%i<, but should be >%i<.", Logger::ERROR, (int)trajectory_point.size(), trajectory_dimension_);
      return false;
    }
    trajectory_positions_.block(trajectory_index, 0, 1, trajectory_dimension_) = trajectory_point.transpose();
  }
  else
  {
    if(trajectory_point.size() != trajectory_dimension_ * POS_VEL_ACC)
    {
      Logger::logPrintf("Trajectory point size provided is >%i<, but should be >%i<.", Logger::ERROR, (int)trajectory_point.size(), trajectory_dimension_ * POS_VEL_ACC);
      return false;
    }
    for (int i = 0; i < trajectory_dimension_; ++i)
    {
      trajectory_positions_(trajectory_index, i) = trajectory_point(i * POS_VEL_ACC + POS);
      trajectory_velocities_(trajectory_index, i) = trajectory_point(i * POS_VEL_ACC + VEL);
      trajectory_accelerations_(trajectory_index, i) = trajectory_point(i * POS_VEL_ACC + ACC);
    }
  }
  return true;
}

bool Trajectory::blowUp(Trajectory& trajectory, const int num_extra_points)
{
  const int pos_length = num_extra_points + trajectory.getNumContainedSamples() + num_extra_points;
  Logger::logPrintf("blowUp NOT implemented yet >%i<", Logger::FATAL, pos_length);
  return false;
}

bool Trajectory::computeDerivatives(const int num_extra_points)
{
  const int add_pos_points = 4;
  const int pos_length = add_pos_points + num_extra_points + trajectory_length_ + num_extra_points + add_pos_points;
  const int vel_length = - 2 + pos_length - 2;
  const int acc_length = - 2 + vel_length - 2;
  const int new_trajectory_length = num_extra_points + trajectory_length_ + num_extra_points;

  // add points at the beginning and ending
  MatrixXd tmp_trajectory_pos = MatrixXd::Zero(pos_length, trajectory_dimension_);
  for (int i = 0; i < pos_length; i++)
  {
    for (int j = 0; j < trajectory_dimension_; j++)
    {
      if (i < add_pos_points + num_extra_points)
      {
        tmp_trajectory_pos(i, j) = trajectory_positions_(0, j);
      }
      else if (i < add_pos_points + num_extra_points + trajectory_length_)
      {
        tmp_trajectory_pos(i, j) = trajectory_positions_(i - add_pos_points - num_extra_points, j);
      }
      else
      {
        tmp_trajectory_pos(i, j) = tmp_trajectory_pos(i - 1, j);
      }
    }
  }

  // derive the positions
  MatrixXd tmp_trajectory_vel = MatrixXd::Zero(vel_length, trajectory_dimension_);
  for (int i = 0; i < vel_length; i++)
  {
    for (int j = 0; j < trajectory_dimension_; j++)
    {
      tmp_trajectory_vel(i, j) = (tmp_trajectory_pos(i, j) - (static_cast<double> (8.0) * tmp_trajectory_pos(i + 1, j)) + (static_cast<double> (8.0)
          * tmp_trajectory_pos(i + 3, j)) - tmp_trajectory_pos(i + 4, j)) / static_cast<double> (12.0);
      tmp_trajectory_vel(i, j) *= sampling_frequency_;
    }
  }

  // derive the velocities
  MatrixXd tmp_trajectory_acc = MatrixXd::Zero(acc_length, trajectory_dimension_);
  for (int i = 0; i < acc_length; i++)
  {
    for (int j = 0; j < trajectory_dimension_; j++)
    {
      tmp_trajectory_acc(i, j) = (tmp_trajectory_vel(i, j) - (static_cast<double> (8.0) * tmp_trajectory_vel(i + 1, j)) + (static_cast<double> (8.0)
          * tmp_trajectory_vel(i + 3, j)) - tmp_trajectory_vel(i + 4, j)) / static_cast<double> (12.0);
      tmp_trajectory_acc(i, j) *= sampling_frequency_;
    }
  }

  // zero out trajectory
  clear();

  // resize trajectory
  trajectory_length_ = new_trajectory_length;
  positions_only_ = false;
  trajectory_positions_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  trajectory_velocities_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  trajectory_accelerations_ = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);

  VectorXd trajectory_positions = VectorXd::Zero(trajectory_dimension_);
  VectorXd trajectory_velocities = VectorXd::Zero(trajectory_dimension_);
  VectorXd trajectory_accelerations = VectorXd::Zero(trajectory_dimension_);
  for (int i = 0; i < acc_length; ++i)
  {
    for (int j = 0; j < trajectory_dimension_; ++j)
    {
      trajectory_positions(j) = tmp_trajectory_pos(i + 4, j);
      trajectory_velocities(j) = tmp_trajectory_vel(i + 2, j);
      trajectory_accelerations(j) = tmp_trajectory_acc(i + 0, j);
    }
    if(!add(trajectory_positions, trajectory_velocities, trajectory_accelerations))
    {
      Logger::logPrintf("Could not add trajectory point.", Logger::ERROR);
      return false;
    }
  }

  return true;
}

bool Trajectory::crop(const int num_trajectory_points, bool crop_the_end)
{
  if(num_trajectory_points<=0)
  {
    Logger::logPrintf("Cannot shrink the trajectory by >%i< trajectory points.", Logger::ERROR, num_trajectory_points);
    return false;
  }
  int new_index_to_last_trajectory_point = index_to_last_trajectory_point_ - num_trajectory_points;
  if(new_index_to_last_trajectory_point < 0)
  {
    Logger::logPrintf("Cannot shrink trajectory by >%i< points, it only contains >%i< trajectory points.", Logger::ERROR, num_trajectory_points, index_to_last_trajectory_point_);
    return false;
  }
  if(sampling_frequency_ < 1e-10)
  {
    Logger::logPrintf("Sampling frequency >%.1f< is invalid. Cannot shrink trajctory.", Logger::ERROR, sampling_frequency_);
    return false;
  }

  // shrink
  MatrixXd trajectory_data = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
  if(crop_the_end)
  {
    trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
    trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
        = trajectory_positions_.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
    trajectory_positions_ = trajectory_data;
    if(!positions_only_)
    {
      trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
      trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
          = trajectory_velocities_.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
      trajectory_velocities_ = trajectory_data;
      trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
      trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
          = trajectory_accelerations_.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
      trajectory_accelerations_ = trajectory_data;
    }
  }
  else
  {
    trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
    trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
        = trajectory_positions_.block(num_trajectory_points, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
    trajectory_positions_ = trajectory_data;
    if(!positions_only_)
    {
      trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
      trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
          = trajectory_velocities_.block(num_trajectory_points, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
      trajectory_velocities_ = trajectory_data;

      trajectory_data.setZero(trajectory_length_, trajectory_dimension_);
      trajectory_data.block(0, 0, new_index_to_last_trajectory_point, trajectory_dimension_)
          = trajectory_accelerations_.block(num_trajectory_points, 0, new_index_to_last_trajectory_point, trajectory_dimension_);
      trajectory_accelerations_ = trajectory_data;
    }
  }

  index_to_last_trajectory_point_ = new_index_to_last_trajectory_point;

  // compute new duration
  trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
  return true;
}

bool Trajectory::crop(const int num_trajectory_points)
{
  if(!crop(num_trajectory_points, true))
  {
    return false;
  }
  if(!crop(num_trajectory_points, false))
  {
    return false;
  }
  return true;
}

string Trajectory::getInfo() const
{
  string info;
  info.assign("Trajectory information: initialized=");
  if (initialized_)
  {
    info.append("true ");
  }
  else
  {
    info.append("false ");
  }
  info.append("position_only=");
  if (positions_only_)
  {
    info.append("true");
  }
  else
  {
    info.append("false");
  }

  info.append(" length=");
  stringstream ss;
  ss << trajectory_length_;
  info.append(ss.str() + string(" "));

  ss.clear();
  ss.str("");
  ss << trajectory_dimension_;
  info.append(string("dimension=") + ss.str() + string(" "));

  ss.clear();
  ss.str("");
  ss << sampling_frequency_;
  info.append(string("sampling_frequency=") + ss.str() + string(" "));

  ss.clear();
  ss.str("");
  ss << trajectory_duration_;
  info.append(string("duration=") + ss.str());

  info.append(string(" variable_names="));
  for (int i = 0; i < (int)variable_names_.size(); ++i)
  {
    info.append(variable_names_[i] + " ");
  }

  return info;
}

// REAL-TIME REQUIREMENTS
bool Trajectory::add(const VectorXd& trajectory_positons,
                     const VectorXd& trajectory_velocities,
                     const VectorXd& trajectory_accelerations,
                     const bool positions_only)
{
  if(positions_only)
  {
    return add(trajectory_positons, true);
  }

  assert(initialized_);
  assert(trajectory_positons.size() == trajectory_velocities.size());
  assert(trajectory_positons.size() == trajectory_accelerations.size());
  assert(trajectory_positons.size() == trajectory_dimension_);

  if (sampling_frequency_ <= 0.0)
  {
    Logger::logPrintf("Sampling frequency >%.1f< is invalid. Not adding trajectory point (Real-time violation). Cannot add sample to trajectory.",
                      Logger::ERROR, sampling_frequency_);
    return false;
  }
  if (!canHold(index_to_last_trajectory_point_, trajectory_positons.size(), false))
  {
    return false;
  }

  // VectorXd comes in as colum vector... so we have to transpose it
  trajectory_positions_.block(index_to_last_trajectory_point_, 0, 1, trajectory_dimension_) = trajectory_positons.transpose();
  trajectory_velocities_.block(index_to_last_trajectory_point_, 0, 1, trajectory_dimension_) = trajectory_velocities.transpose();
  trajectory_accelerations_.block(index_to_last_trajectory_point_, 0, 1, trajectory_dimension_) = trajectory_accelerations.transpose();
  index_to_last_trajectory_point_++;
  // update the duration of the trajectory. It has been checked that sampling_frequency is non zero
  trajectory_duration_ = static_cast<double> (index_to_last_trajectory_point_) / sampling_frequency_;
  // Logger::logPrintf("Setting trajectory duration to >%.1f< seconds.", Logger::DEBUG, trajectory_duration_);

  return true;
}

// REAL-TIME REQUIREMENTS
bool Trajectory::add(const VectorXd& trajectory_positons,
                     const bool positions_only)
{
  // TODO: check whether all this check should be done inside
  assert(initialized_);
  if(trajectory_positons.size() != trajectory_dimension_)
  {
    Logger::logPrintf("Size of provided vector >%i< does not match trajectory dimension >%i< (Real-time violation). Cannot add sample to trajectory.",
                      Logger::FATAL, (int)trajectory_positons.size(), trajectory_dimension_);
    return false;
  }
  if (sampling_frequency_ <= 0.0)
  {
    Logger::logPrintf("Sampling frequency >%.1f< is invalid. Not adding trajectory point (Real-time violation). Cannot add sample to trajectory.",
                      Logger::ERROR, sampling_frequency_);
    return false;
  }
  if (!canHold(index_to_last_trajectory_point_, trajectory_positons.size(), positions_only))
  {
    return false;
  }

  // VectorXd comes in as colum vector... so we have to transpose it
  trajectory_positions_.block(index_to_last_trajectory_point_, 0, 1, trajectory_dimension_) = trajectory_positons.transpose();
  index_to_last_trajectory_point_++;
  // update the duration of the trajectory. It has been checked that sampling_frequency is non zero
  trajectory_duration_ = static_cast<double> (index_to_last_trajectory_point_) / sampling_frequency_;
  // Logger::logPrintf("Setting trajectory duration to >%.1f< seconds.", Logger::DEBUG, trajectory_duration_);

  return true;
}

// THIS FUNCTION IS NOT REAL-TIME FRIENDLY
bool Trajectory::add(const std::vector<double>& trajectory_positons,
                     const bool positions_only)
{
  const int size = (int)trajectory_positons.size();
  VectorXd eigen_trajectory_positions = VectorXd::Zero((DenseIndex)size);
  for (int i = 0; i < size; ++i)
  {
    eigen_trajectory_positions(i) = trajectory_positons[i];
  }
  return add(eigen_trajectory_positions, positions_only);
}

bool Trajectory::computePositionNMSE(const Trajectory& other_trajectory,
                                     VectorXd& nmse_of_positions) const
{
  if(!isCompatible(other_trajectory, true))
  {
    Logger::logPrintf("Trajectories are not compatible, cannot compute normalized mean squared error.", Logger::ERROR);
    return false;
  }
  if (nmse_of_positions.size() != trajectory_dimension_)
  {
    Logger::logPrintf("Provided vector to hold normalized MSE has size >%i<, but it should have size >%i<.",
                      Logger::ERROR, nmse_of_positions.size(), trajectory_dimension_);
    return false;
  }
  if ((!other_trajectory.containVariables(variable_names_)) || (!containVariables(other_trajectory.variable_names_)))
  {
    Logger::logPrintf("Variable names are not the same, cannot compute MSE.", Logger::ERROR);
    return false;
  }
  Logger::logPrintf("Computing normalized mean squared error between two trajectories with >%i< dimensions and >%i< samples.",
                    Logger::DEBUG, trajectory_dimension_, index_to_last_trajectory_point_);

  Trajectory trajectory = other_trajectory;
  if (!trajectory.rearange(variable_names_))
  {
    return false;
  }

//  nmse_of_positions = (trajectory_positions_.block(0, 0, index_to_last_trajectory_point_, trajectory_dimension_)
//      - trajectory.trajectory_positions_.block(0, 0, index_to_last_trajectory_point_, trajectory_dimension_)).cwise().square().colwise().sum();
  nmse_of_positions = (trajectory_positions_.block(0, 0, index_to_last_trajectory_point_, trajectory_dimension_)
      - trajectory.trajectory_positions_.block(0, 0, index_to_last_trajectory_point_, trajectory_dimension_)).array().square().matrix().colwise().sum();

  // normalize error
  nmse_of_positions /= index_to_last_trajectory_point_;

  //    for (int i = 0; i < num_trajectory_points_used_for_computing_mse; ++i)
  //    {
  //        for (int j = 0; j < trajectory_dimension_; ++j)
  //        {
  //            mse_of_positions(j) += pow(trajectory_positions_(i, j) - trajectory.getTrajectoryPosition(i, j), 2);
  //        }
  //    }
  //
  //    for (int j = 0; j < num_position_traces; j++)
  //    {
  //        mse_of_positions(j) /= num_trajectory_points_used_for_computing_mse;
  //    }
  return true;
}

bool Trajectory::cut(Trajectory& other_trajectory, bool verbose)
{
  if(!initialized_)
  {
    Logger::logPrintf(verbose, "Trajectory is not initialized. Cannot cut it.", Logger::FATAL);
    return false;
  }
  if(!other_trajectory.initialized_)
  {
    Logger::logPrintf(verbose, "Other trajectory is not initialized. Cannot cut it.", Logger::FATAL);
    return false;
  }
  // if (sampling_frequency_ != other_trajectory.sampling_frequency_)
  if (fabs(sampling_frequency_ - other_trajectory.sampling_frequency_) > 1e-3)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same sampling frequency (>%.3f< Hz vs. >%.3f< Hz). Cannot cut them.",
                      Logger::ERROR, sampling_frequency_, other_trajectory.sampling_frequency_);
    return false;
  }

  if (index_to_last_trajectory_point_ < other_trajectory.index_to_last_trajectory_point_)
  {
    Logger::logPrintf(verbose, "Cutting trajectory to contain >%i< samples with duration >%.1f< seconds and sampled at >%.1f< Hz.",
                      Logger::INFO, index_to_last_trajectory_point_, sampling_frequency_, trajectory_duration_);
    other_trajectory.index_to_last_trajectory_point_ = index_to_last_trajectory_point_;
    other_trajectory.sampling_frequency_ = sampling_frequency_;
    other_trajectory.trajectory_duration_ = trajectory_duration_;
  }
  else
  {
    Logger::logPrintf(verbose, "Cutting trajectory to contain >%i< samples with duration >%.1f< seconds and sampled at >%.1f< Hz.",
                      Logger::INFO, other_trajectory.index_to_last_trajectory_point_, other_trajectory.sampling_frequency_, other_trajectory.trajectory_duration_);
    index_to_last_trajectory_point_ = other_trajectory.index_to_last_trajectory_point_;
    sampling_frequency_ = other_trajectory.sampling_frequency_;
    trajectory_duration_ = other_trajectory.trajectory_duration_;
  }
  return true;
}

bool Trajectory::isCompatible(const Trajectory& other_trajectory, bool verbose) const
{
  if(!initialized_)
  {
    Logger::logPrintf(verbose, "Trajectory is not initialized.", Logger::FATAL);
    return false;
  }
  if(!other_trajectory.initialized_)
  {
    Logger::logPrintf(verbose, "Other trajectory is not initialized.", Logger::FATAL);
    return false;
  }
  if (positions_only_ != other_trajectory.positions_only_)
  {
    Logger::logPrintf(verbose, "One trajectory only contains positions, and the other also contains velocities and accelerations.",
                      Logger::ERROR);
    return false;
  }
  if (index_to_last_trajectory_point_ != other_trajectory.index_to_last_trajectory_point_)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same index to last trajectory point (>%i< vs. >%i<).",
                      Logger::ERROR, index_to_last_trajectory_point_, other_trajectory.index_to_last_trajectory_point_);
    return false;
  }
  if (fabs(sampling_frequency_ - other_trajectory.sampling_frequency_) > 1e-3)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same sampling frequency (>%.3f< Hz vs. >%.3f< Hz).",
                      Logger::ERROR, sampling_frequency_, other_trajectory.sampling_frequency_);
    return false;
  }
  if (fabs(trajectory_duration_ - other_trajectory.trajectory_duration_) > 1e-3)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same duration (>%.3f< seconds vs. >%.3f< seconds).",
                      Logger::ERROR, trajectory_duration_, other_trajectory.trajectory_duration_);
    return false;
  }
  return true;
}

bool Trajectory::isAppendable(const Trajectory& other_trajectory, bool verbose) const
{
  if (!initialized_)
  {
    Logger::logPrintf(verbose, "Trajectory is not initialized.", Logger::FATAL);
    return false;
  }
  if (!other_trajectory.initialized_)
  {
    Logger::logPrintf(verbose, "Other trajectory is not initialized.", Logger::FATAL);
    return false;
  }
  if (positions_only_ != other_trajectory.positions_only_)
  {
    Logger::logPrintf(verbose, "One trajectory only contains positions, and the other also contains velocities and accelerations.", Logger::ERROR);
    return false;
  }
  // if (sampling_frequency_ != other_trajectory.sampling_frequency_)
  if (fabs(sampling_frequency_ - other_trajectory.sampling_frequency_) > 1e-3)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same sampling frequency (>%.3f< Hz vs. >%.3f< Hz).", Logger::ERROR, sampling_frequency_,
                      other_trajectory.sampling_frequency_);
    return false;
  }
  if (trajectory_dimension_ != other_trajectory.trajectory_dimension_)
  {
    Logger::logPrintf(verbose, "Trajectories do not have same dimensions (>%i< vs. >%i<).", Logger::ERROR, trajectory_dimension_,
                      other_trajectory.trajectory_dimension_);
    return false;
  }
  if(!containVariables(other_trajectory.getVariableNames()))
  {
    Logger::logPrintf(verbose, "Variable names do not match up.", Logger::ERROR);
    return false;
  }
  return true;
}

bool Trajectory::cutAndCombine(const Trajectory& other_trajectory, bool verbose)
{
  Trajectory trajectory = other_trajectory;
  if(!cut(trajectory, verbose))
  {
    Logger::logPrintf("Could not cut trajectories.", Logger::ERROR);
    return false;
  }
  return combine(trajectory, verbose);
}

bool Trajectory::combine(const Trajectory& other_trajectory, bool verbose)
{

  if(!isCompatible(other_trajectory, verbose))
  {
    Logger::logPrintf(verbose, "Cannot combine trajectories.", Logger::ERROR);
    return false;
  }

  variable_names_.insert(variable_names_.end(), other_trajectory.variable_names_.begin(), other_trajectory.variable_names_.end());
  variable_units_.insert(variable_units_.end(), other_trajectory.variable_units_.begin(), other_trajectory.variable_units_.end());

  // use longer trajectory to determine the container length ( in case they are different )
  if(trajectory_length_ < other_trajectory.trajectory_length_)
  {
    trajectory_length_ = other_trajectory.trajectory_length_;
  }

  int new_trajectory_dimension = trajectory_dimension_ + other_trajectory.trajectory_dimension_;
  MatrixXd new_trajectory_positions = MatrixXd::Zero(trajectory_length_, new_trajectory_dimension);
  new_trajectory_positions.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
      = trajectory_positions_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
  new_trajectory_positions.block(0,trajectory_dimension_,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_)
      = other_trajectory.trajectory_positions_.block(0,0,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_);
  trajectory_positions_ = new_trajectory_positions;

  if(!positions_only_)
  {
    MatrixXd new_trajectory_velocities = MatrixXd::Zero(trajectory_length_, new_trajectory_dimension);
    new_trajectory_velocities.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
        = trajectory_velocities_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
    new_trajectory_velocities.block(0,trajectory_dimension_,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_)
        = other_trajectory.trajectory_velocities_.block(0,0,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_);
    trajectory_velocities_ = new_trajectory_velocities;

    MatrixXd new_trajectory_accelerations = MatrixXd::Zero(trajectory_length_, new_trajectory_dimension);
    new_trajectory_accelerations.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
        = trajectory_accelerations_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
    new_trajectory_accelerations.block(0,trajectory_dimension_,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_)
        = other_trajectory.trajectory_accelerations_.block(0,0,index_to_last_trajectory_point_,other_trajectory.trajectory_dimension_);
    trajectory_accelerations_ = new_trajectory_accelerations;
  }
  trajectory_dimension_ = new_trajectory_dimension;

  return true;
}

bool Trajectory::append(const Trajectory& other_trajectory)
{
  if(!isAppendable(other_trajectory, true))
  {
    Logger::logPrintf("Could not append trajectory.", Logger::ERROR);
    return false;
  }

  Trajectory trajectory = other_trajectory;
  if(!trajectory.rearange(variable_names_))
  {
    Logger::logPrintf("Could not append trajectory.", Logger::ERROR);
    return false;
  }

  // allocate more memory if this trajectory is not large enough
  if (index_to_last_trajectory_point_ + trajectory.index_to_last_trajectory_point_ > trajectory_length_)
  {
    // set new trajectory length
    trajectory_length_ = index_to_last_trajectory_point_ + trajectory.index_to_last_trajectory_point_;
    MatrixXd new_trajectory_positions = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
    new_trajectory_positions.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
          = trajectory_positions_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
    new_trajectory_positions.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
        = trajectory.trajectory_positions_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
    trajectory_positions_ = new_trajectory_positions;
    if(!positions_only_)
    {
      MatrixXd new_trajectory_velocities = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
      new_trajectory_velocities.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
            = trajectory_velocities_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
      new_trajectory_velocities.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
          = trajectory.trajectory_velocities_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
      trajectory_velocities_ = new_trajectory_velocities;

      MatrixXd new_trajectory_accelerations = MatrixXd::Zero(trajectory_length_, trajectory_dimension_);
      new_trajectory_accelerations.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_)
            = trajectory_accelerations_.block(0,0,index_to_last_trajectory_point_,trajectory_dimension_);
      new_trajectory_accelerations.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
          = trajectory.trajectory_accelerations_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
      trajectory_accelerations_ = new_trajectory_accelerations;
    }
  }
  else
  {
    trajectory_positions_.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
        = trajectory.trajectory_positions_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
    if(!positions_only_)
    {
      trajectory_velocities_.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
            = trajectory.trajectory_velocities_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
      trajectory_accelerations_.block(index_to_last_trajectory_point_,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_)
            = trajectory.trajectory_accelerations_.block(0,0,trajectory.index_to_last_trajectory_point_,trajectory_dimension_);
    }
  }

  // update index and trajectory duration
  index_to_last_trajectory_point_ += trajectory.index_to_last_trajectory_point_;
  trajectory_duration_ = static_cast<double> (index_to_last_trajectory_point_) / sampling_frequency_;
  return true;
}

void Trajectory::clear()
{
  assert(initialized_);
  index_to_last_trajectory_point_ = 0;
  trajectory_duration_ = 0.0;
  trajectory_positions_.setZero(trajectory_length_, trajectory_dimension_);
  if (!positions_only_)
  {
    trajectory_velocities_.setZero(trajectory_length_, trajectory_dimension_);
    trajectory_accelerations_.setZero(trajectory_length_, trajectory_dimension_);
  }
}

bool Trajectory::calculateMinJerkNextStep(const Eigen::VectorXd &start,
                                          const Eigen::VectorXd &goal,
                                          const double duration,
                                          const double delta_t,
                                          Eigen::VectorXd &next)
{
  if ((duration <= 0) || (delta_t <= 0) || (delta_t > duration))
  {
    Logger::logPrintf(duration <= 0, "Duration >%.1f< seconds is invalid.", Logger::ERROR, duration);
    Logger::logPrintf(delta_t <= 0, "Value for delta_t >%f< is invalid.", Logger::ERROR, delta_t);
    Logger::logPrintf(delta_t > duration, "Value for delta_t >%f< is greater than duration >%f<.", Logger::ERROR, delta_t, duration);
    return false;
  }
  if ((start.size() != POS_VEL_ACC) || (goal.size() != POS_VEL_ACC) || (next.size() != POS_VEL_ACC))
  {
    Logger::logPrintf(start.size() != POS_VEL_ACC, "Start vector has wrong size >%i<, should be >%i<. Cannot compute next minimum jerk step.",
                      Logger::ERROR, start.size(), POS_VEL_ACC);
    Logger::logPrintf(goal.size() != POS_VEL_ACC, "Goal vector has wrong size >%i<, should be >%i<. Cannot compute next minimum jerk step.",
                      Logger::ERROR, goal.size(), POS_VEL_ACC);
    Logger::logPrintf(next.size() != POS_VEL_ACC, "Next vector has wrong size >%i<, should be >%i<. Cannot compute next minimum jerk step.",
                      Logger::ERROR, next.size(), POS_VEL_ACC);
    return false;
  }

  const double t1 = delta_t;
  const double t2 = t1 * delta_t;
  const double t3 = t2 * delta_t;
  const double t4 = t3 * delta_t;
  const double t5 = t4 * delta_t;

  const double tau1 = duration;
  const double tau2 = tau1 * duration;
  const double tau3 = tau2 * duration;
  const double tau4 = tau3 * duration;
  const double tau5 = tau4 * duration;

  // calculate the constants
  const double dist = goal(POS) - start(POS);
  const double c1 = 6. * dist / tau5 + (goal(ACC) - start(ACC)) / (2. * tau3) - 3. * (start(VEL) + goal(VEL)) / tau4;
  const double c2 = -15. * dist / tau4 + (3. * start(ACC) - 2. * goal(ACC)) / (2. * tau2) + (8. * start(VEL) + 7. * goal(VEL)) / tau3;
  const double c3 = 10. * dist / tau3 + (goal(ACC) - 3. * start(ACC)) / (2. * duration) - (6. * start(VEL) + 4. * goal(VEL)) / tau2;
  const double c4 = start(ACC) / 2.;
  const double c5 = start(VEL);
  const double c6 = start(POS);

  next(POS) = c1 * t5 + c2 * t4 + c3 * t3 + c4 * t2 + c5 * t1 + c6;
  next(VEL) = 5. * c1 * t4 + 4 * c2 * t3 + 3 * c3 * t2 + 2 * c4 * t1 + c5;
  next(ACC) = 20. * c1 * t3 + 12. * c2 * t2 + 6. * c3 * t1 + 2. * c4;
  return true;
}

}
