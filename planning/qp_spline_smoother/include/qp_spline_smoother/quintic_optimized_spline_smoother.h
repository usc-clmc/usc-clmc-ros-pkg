/*
 * quintic_optimized_spline_smoother.h
 *
 *  Created on: Jan 26, 2011
 *      Author: kalakris
 */

#ifndef QUINTIC_OPTIMIZED_SPLINE_SMOOTHER_H_
#define QUINTIC_OPTIMIZED_SPLINE_SMOOTHER_H_

#include <spline_smoother/spline_smoother.h>
#include <spline_smoother/spline_smoother_utils.h>
#include <quadprog/QuadProg++.hh>

namespace qp_spline_smoother
{

template<typename T>
class QuinticOptimizedSplineSmoother : public spline_smoother::SplineSmoother<T>
{
public:
  QuinticOptimizedSplineSmoother();
  virtual ~QuinticOptimizedSplineSmoother();

  virtual bool smooth(const T& trajectory_in, T& trajectory_out) const;
  virtual bool configure();

private:
  enum
  {
    MIN_VEL=0,
    MIN_ACC=1,
    MIN_JERK=2,
    NUM_WEIGHTS
  };

  std::vector<double> cost_function_weights_;
  int chunk_size_;
  double min_dt_;

  bool optimize(const double *x, double *xd, double *xdd, double *t, int length) const;
  bool numericalDifferentiation(const T& trajectory_in, T& trajectory_out) const;
  double weightedAvg(double w1, double v1, double w2, double v2) const;

};

template<typename T>
QuinticOptimizedSplineSmoother<T>::QuinticOptimizedSplineSmoother()
{
  cost_function_weights_.resize(NUM_WEIGHTS);
  cost_function_weights_[MIN_VEL]=0.0;
  cost_function_weights_[MIN_ACC]=1.0;
  cost_function_weights_[MIN_JERK]=0.0;
  chunk_size_=10;
  min_dt_ = 0.01;
}

template<typename T>
bool QuinticOptimizedSplineSmoother<T>::configure()
{
  if (filters::FilterBase<T>::params_.find("velocity_cost") != filters::FilterBase<T>::params_.end())
  {
    cost_function_weights_[MIN_VEL] = filters::FilterBase<T>::params_["velocity_cost"];
  }
  if (filters::FilterBase<T>::params_.find("acceleration_cost") != filters::FilterBase<T>::params_.end())
  {
    cost_function_weights_[MIN_ACC] = filters::FilterBase<T>::params_["acceleration_cost"];
  }
  if (filters::FilterBase<T>::params_.find("jerk_cost") != filters::FilterBase<T>::params_.end())
  {
    cost_function_weights_[MIN_JERK] = filters::FilterBase<T>::params_["jerk_cost"];
  }
  if (filters::FilterBase<T>::params_.find("chunk_size") != filters::FilterBase<T>::params_.end())
  {
    chunk_size_ = filters::FilterBase<T>::params_["chunk_size"];
  }
  ROS_INFO("velocity cost = %f", cost_function_weights_[MIN_VEL]);
  ROS_INFO("acceleration cost = %f", cost_function_weights_[MIN_ACC]);
  ROS_INFO("jerk cost = %f", cost_function_weights_[MIN_JERK]);
  ROS_INFO("chunk size = %d", chunk_size_);
  ROS_INFO("min_dt = %f", min_dt_);
  
  for (int i=0; i<NUM_WEIGHTS; ++i)
  {
    cost_function_weights_[i] *= 1000.0;
  }
  return true;
}

template<typename T>
QuinticOptimizedSplineSmoother<T>::~QuinticOptimizedSplineSmoother()
{
}

template<typename T>
double QuinticOptimizedSplineSmoother<T>::weightedAvg(double w1, double v1, double w2, double v2) const
{
  return (w1*v1 + w2*v2)/(w1+w2);
}

template<typename T>
bool QuinticOptimizedSplineSmoother<T>::numericalDifferentiation(const T& trajectory_in, T& trajectory_out) const
{
  int size = trajectory_in.trajectory.points.size();
  int num_traj = trajectory_in.trajectory.joint_names.size();

  // for every point in time:
  for (int i = 1; i < size - 1; ++i)
  {
    double dt1 = (trajectory_in.trajectory.points[i].time_from_start
        - trajectory_in.trajectory.points[i - 1].time_from_start).toSec();
    double dt2 = (trajectory_in.trajectory.points[i + 1].time_from_start
        - trajectory_in.trajectory.points[i].time_from_start).toSec();

    // for every (joint) trajectory
    for (int j = 0; j < num_traj; ++j)
    {
      double dx1 = trajectory_in.trajectory.points[i].positions[j]
          - trajectory_in.trajectory.points[i - 1].positions[j];
      double dx2 = trajectory_in.trajectory.points[i + 1].positions[j]
          - trajectory_in.trajectory.points[i].positions[j];

      double v1 = dx1 / dt1;
      double v2 = dx2 / dt2;

      trajectory_out.trajectory.points[i].velocities[j] = 0.5 * (v1 + v2);

      trajectory_out.trajectory.points[i].accelerations[j] = (v2 - v1) / (0.5 * (dt1 + dt2));

    }
  }
  return true;
}


template<typename T>
bool QuinticOptimizedSplineSmoother<T>::smooth(const T& trajectory_in, T& trajectory_out) const
{
  ros::Time start_time = ros::Time::now();
  bool success = true;
  int size = trajectory_in.trajectory.points.size();
  int num_traj = trajectory_in.trajectory.joint_names.size();
  trajectory_out = trajectory_in;

  if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
    return false;

  // apply min_dt:
  double dt_add=0.0;
  for (int i=1; i<size; ++i)
  {
    double t1 = trajectory_out.trajectory.points[i-1].time_from_start.toSec();
    double t2 = trajectory_out.trajectory.points[i].time_from_start.toSec();
    t2 += dt_add;
    double dt = t2 - t1;
    if (dt < min_dt_)
    {
      dt = min_dt_;
      double old_t2 = t2;
      t2 = t1 + dt;
      dt_add += (t2 - old_t2);
    }
    trajectory_out.trajectory.points[i].time_from_start = ros::Duration(t2);
  }

  bool qp_success = true;

  // zero the first and last velocities and accelerations
  for (int j = 0; j < num_traj; ++j)
  {
    trajectory_out.trajectory.points[0].velocities[j] = 0.0;
    trajectory_out.trajectory.points[0].accelerations[j] = 0.0;
    trajectory_out.trajectory.points[size - 1].velocities[j] = 0.0;
    trajectory_out.trajectory.points[size - 1].accelerations[j] = 0.0;
  }

  // initialize with numerical differentiation
  numericalDifferentiation(trajectory_out, trajectory_out);

  if (size>=3)
  {
    // optimize in chunks
#pragma omp parallel for
    for (int j = 0; j < num_traj; ++j)
    {
      double x[chunk_size_];
      double xd[chunk_size_];
      double xdd[chunk_size_];
      double t[chunk_size_];
      int last_valid = 0;

      do
      {
        int start_point = last_valid - chunk_size_/2;
        if (start_point < 0)
          start_point = 0;
        int end_point = start_point + chunk_size_ - 1;
        if (end_point > size-1)
          end_point = size-1;

        int num_points = end_point - start_point + 1;
        if (num_points < 3)
        {
          int diff = 3 - num_points;
          start_point -= diff;
          num_points +=diff;
          if (start_point < 0)
          {
            ROS_ERROR("QuinticOptimized: Strange condition occurred that should never happen!");
            qp_success = false;
            break;
          }
        }

        //ROS_INFO("Chunk = %d to %d", start_point, end_point);

        // create input for optimize:
        for (int i=start_point; i<=end_point; ++i)
        {
          x[i-start_point] = trajectory_out.trajectory.points[i].positions[j];
          xd[i-start_point] = trajectory_out.trajectory.points[i].velocities[j];
          xdd[i-start_point] = trajectory_out.trajectory.points[i].accelerations[j];
          t[i-start_point] = trajectory_out.trajectory.points[i].time_from_start.toSec() -
              trajectory_out.trajectory.points[start_point].time_from_start.toSec();
        }
        if (!optimize(x, xd, xdd, t, num_points))
          qp_success = false;
        else
        {
          for (int i=start_point; i<=end_point; ++i)
          {
            double w1=0.0, w2=1.0;
            if (i<last_valid)
            {
              w1 = last_valid-i;
              w2 = (last_valid-start_point)-w1;
            }
            //ROS_INFO("%i - w1=%f, w2=%f", i, w1, w2);
            trajectory_out.trajectory.points[i].velocities[j] =
                weightedAvg(w1, trajectory_out.trajectory.points[i].velocities[j], w2, xd[i-start_point]);
            trajectory_out.trajectory.points[i].accelerations[j] =
                weightedAvg(w1, trajectory_out.trajectory.points[i].accelerations[j], w2, xdd[i-start_point]);
          }
        }

        last_valid = end_point;

      }
      while (last_valid < size-1);
    }
  }


  if (!qp_success)
  {
    numericalDifferentiation(trajectory_out, trajectory_out);
  }

  ROS_INFO("Quintic optimization took %f seconds", (ros::Time::now()-start_time).toSec());

  return success;
}

template<typename T>
bool QuinticOptimizedSplineSmoother<T>::optimize(const double *x, double *xd, double *xdd, double *t, int length) const
{
  int numSegments = length - 1;
  int numVars = numSegments * 5;
  int numEqual = numSegments * 3 + 2;
  int numInEqual = 0;

  QuadProgPP::Vector<double> vars(numVars);
  QuadProgPP::Matrix<double> quadCost(numVars, numVars);
  QuadProgPP::Vector<double> linearCost(numVars);
  QuadProgPP::Matrix<double> inequalities(numVars, numInEqual);
  QuadProgPP::Vector<double> ineqConst(numInEqual);
  QuadProgPP::Matrix<double> equalities(numVars, numEqual);
  QuadProgPP::Vector<double> eqConst(numEqual);

  // dt and powers of dt
  QuadProgPP::Matrix<double> dt(numSegments,10);
  double dx[numSegments];

  for (int i = 0; i < length - 1; i++)
  {
    dt[i][1] = t[i + 1] - t[i];
    dx[i] = x[i + 1] - x[i];
    for (int j = 2; j <= 9; j++)
    {
      dt[i][j] = dt[i][j - 1] * dt[i][1];
    }
  }

  // the variables are arranged as [a b c d e] for each spline segment,
  // where the spline is x = at^5 + bt^4 + ct^3 + dt^2 + et + f
  // the f can be determined without optimization for each segment (f = x_0)


  // zero the quadratic cost matrix and linear cost vector:
  for (int i = 0; i < numVars; i++)
  {
    for (int j = 0; j < numVars; j++)
    {
      quadCost[i][j] = 0.0;
    }
    linearCost[i] = 0.0;
  }

  // construct the quadratic cost matrix:

  if (cost_function_weights_[MIN_VEL] > 0.0)
  {
    double w = cost_function_weights_[MIN_VEL];
    for (int i = 0; i < numSegments; i++)
    {
      int a = i * 5;
      int b = i * 5 + 1;
      int c = i * 5 + 2;
      int d = i * 5 + 3;
      int e = i * 5 + 4;
      quadCost[a][a] = (25.0 / 9.0) * dt[i][9] * w;
      quadCost[a][b] = quadCost[b][a] = (5.0 / 2.0) * dt[i][8] * w;
      quadCost[a][c] = quadCost[c][a] = ((30.0 / 7.0) / 2.0) * dt[i][7] * w;
      quadCost[b][b] = (16.0 / 7.0) * dt[i][7] * w;
      quadCost[a][d] = quadCost[d][a] = ((20.0 / 6.0) / 2.0) * dt[i][6] * w;
      quadCost[c][b] = quadCost[b][c] = ((24.0 / 6.0) / 2.0) * dt[i][6] * w;
      quadCost[a][e] = quadCost[e][a] = ((10.0 / 5.0) / 2.0) * dt[i][5] * w;
      quadCost[b][d] = quadCost[d][b] = ((16.0 / 5.0) / 2.0) * dt[i][5] * w;
      quadCost[c][c] = (9.0 / 5.0) * dt[i][5] * w;
      quadCost[b][e] = quadCost[e][b] = ((8.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[c][d] = quadCost[d][c] = ((12.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[c][e] = quadCost[e][c] = ((6.0 / 3.0) / 2.0) * dt[i][3] * w;
      quadCost[d][d] = (4.0 / 3.0) * dt[i][3] * w;
      quadCost[d][e] = quadCost[e][d] = dt[i][2] * w;
      quadCost[e][e] = dt[i][1] * w;
    }
  }
  if (cost_function_weights_[MIN_ACC] > 0.0)
  {
    double w = cost_function_weights_[MIN_ACC];
    for (int i = 0; i < numSegments; i++)
    {
      int a = i * 5;
      int b = i * 5 + 1;
      int c = i * 5 + 2;
      int d = i * 5 + 3;
      int e = i * 5 + 4;
      quadCost[a][a] += (400.0 / 7.0) * dt[i][7] * w;
      quadCost[a][b] += (80.0 / 2.0) * dt[i][6] * w;
      quadCost[b][a] += (80.0 / 2.0) * dt[i][6] * w;
      quadCost[b][b] += (144.0 / 5.0) * dt[i][5] * w;
      quadCost[a][c] += ((240.0 / 5.0) / 2.0) * dt[i][5] * w;
      quadCost[c][a] += ((240.0 / 5.0) / 2.0) * dt[i][5] * w;
      quadCost[c][b] += ((144.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[b][c] += ((144.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[a][d] += ((80.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[d][a] += ((80.0 / 4.0) / 2.0) * dt[i][4] * w;
      quadCost[b][d] += ((48.0 / 3.0) / 2.0) * dt[i][3] * w;
      quadCost[d][b] += ((48.0 / 3.0) / 2.0) * dt[i][3] * w;
      quadCost[c][c] += (36.0 / 3.0) * dt[i][3] * w;
      quadCost[c][d] += ((12.0) / 2.0) * dt[i][2] * w;
      quadCost[d][c] += ((12.0) / 2.0) * dt[i][2] * w;
      quadCost[d][d] += (4.0) * dt[i][1] * w;
      quadCost[e][e] += 10e-10 * w;
    }
  }
  if (cost_function_weights_[MIN_JERK] > 0.0)
  {
    double w = cost_function_weights_[MIN_JERK];
    for (int i = 0; i < numSegments; i++)
    {
      int a = i * 5;
      int b = i * 5 + 1;
      int c = i * 5 + 2;
      int d = i * 5 + 3;
      int e = i * 5 + 4;
      quadCost[a][a] += (720) * dt[i][5] * w;
      quadCost[a][b] += (720.0 / 2.0) * dt[i][4] * w;
      quadCost[b][a] += (720.0 / 2.0) * dt[i][4] * w;
      quadCost[a][c] += ((720.0 / 3.0) / 2.0) * dt[i][3] * w;
      quadCost[c][a] += ((720.0 / 3.0) / 2.0) * dt[i][3] * w;
      quadCost[b][b] += (576.0 / 3.0) * dt[i][3] * w;
      quadCost[c][b] += ((144.0) / 2.0) * dt[i][2] * w;
      quadCost[b][c] += ((144.0) / 2.0) * dt[i][2] * w;
      quadCost[c][c] += (36.0) * dt[i][1] * w;
      quadCost[d][d] += 10e-10 * w;
      quadCost[e][e] += 10e-10 * w;
    }
  }

  // print the quadratic cost matrix:
  /*printf("Quadcost = \n");
   for (int i=0; i<numVars; i++)
   {
   for (int j=0; j<numVars; j++)
   {
   printf("%f\t",quadCost[i][j]);
   }
   printf("\n");
   }*/

  // zero the equalities:
  for (int i = 0; i < numEqual; i++)
  {
    for (int j = 0; j < numVars; j++)
    {
      equalities[j][i] = 0.0;
    }
    eqConst[i] = 0.0;
  }

  // construct the equalities:
  int j = 0;
  for (int i = 0; i < numSegments; i++)
  {
    int a = i * 5;
    int b = i * 5 + 1;
    int c = i * 5 + 2;
    int d = i * 5 + 3;
    int e = i * 5 + 4;
    //int aNext = a + 5;
    //int bNext = b + 5;
    //int cNext = c + 5;
    int dNext = d + 5;
    int eNext = e + 5;

    // end position of the segment:
    equalities[a][j] = dt[i][5];
    equalities[b][j] = dt[i][4];
    equalities[c][j] = dt[i][3];
    equalities[d][j] = dt[i][2];
    equalities[e][j] = dt[i][1];
    eqConst[j] = x[i] - x[i + 1];
    j++;

    // end velocity of the segment:
    equalities[a][j] = 5.0 * dt[i][4];
    equalities[b][j] = 4.0 * dt[i][3];
    equalities[c][j] = 3.0 * dt[i][2];
    equalities[d][j] = 2.0 * dt[i][1];
    equalities[e][j] = 1.0;
    if (i < numSegments - 1)
      equalities[eNext][j] = -1.0;
    else
      eqConst[j] = -xd[length - 1];
    j++;

    // end acceleration of the segment:
    equalities[a][j] = 20.0 * dt[i][3];
    equalities[b][j] = 12.0 * dt[i][2];
    equalities[c][j] = 6.0 * dt[i][1];
    equalities[d][j] = 2.0;
    if (i < numSegments - 1)
      equalities[dNext][j] = -2.0;
    else
      eqConst[j] = -xdd[length - 1];
    j++;

  }
  // the extra equalities are for start vel and acc
  equalities[4][j] = 1.0;
  eqConst[j] = -xd[0];
  j++;

  equalities[3][j] = 2.0;
  eqConst[j] = -xdd[0];
  j++;

  ROS_ASSERT(numEqual == j);

  // zero the inequalities:
  for (int i = 0; i < numInEqual; i++)
  {
    for (int j = 0; j < numVars; j++)
    {
      inequalities[j][i] = 0.0;
    }
    ineqConst[i] = 0.0;
  }

  double cost = 0.0;
  try
  {
     cost = QuadProgPP::solve_quadprog(quadCost, linearCost, equalities, eqConst, inequalities, ineqConst, vars);
  }
  catch (...)
  {
    ROS_ERROR("QuinticOptimizedSplineSmoother: Quadratic program failed!\n");
    return false;
  }

  if (cost == std::numeric_limits<double>::infinity())
  {
    ROS_ERROR("QuinticOptimizedSplineSmoother: Quadratic program failed!\n");
    return false;
    // we're pretty much doomed here, since this shouldn't happen!
  }
  else
  {
    //              printf("Quadratic program succeeded gwith cost=%f\n",cost);
    /*printf("Variables are: \n");
     for (int i=0; i<numVars; i++)
     {
     printf("%f\t", vars[i]);
     }
     printf("\n");*/

  }

  //printf("First spline segment: %f %f %f %f\n", vars[0], vars[1], vars[2], xd[0]);

  // now get back the boundary conditions from the vars:
  for (int i = 1; i < length - 1; i++)
  {
    int d = i * 5 + 3;
    int e = i * 5 + 4;
    xd[i] = vars[e];
    xdd[i] = 2.0 * vars[d];
    //ROS_DEBUG("%f\t%f\n",xd[i],xdd[i]);
  }
  return true;
}

}

#endif /* QUINTIC_OPTIMIZED_SPLINE_SMOOTHER_H_ */
