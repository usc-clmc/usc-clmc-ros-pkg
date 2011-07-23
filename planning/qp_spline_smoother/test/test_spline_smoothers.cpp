#include <qp_spline_smoother/quintic_optimized_spline_smoother.h>
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>

int main(int argc, char** argv)
{
  //qp_spline_smoother::QuinticOptimizedSplineSmoother<motion_planning_msgs::FilterJointTrajectory::Request>
  qp_spline_smoother::QuinticOptimizedSplineSmoother<motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request> foo;
  //foo.configure();
  return 0;
}
