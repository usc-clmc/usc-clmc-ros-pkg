#include <qp_spline_smoother/quintic_optimized_spline_smoother.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

int main(int argc, char** argv)
{
  //qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectory::Request>
  qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints> foo;
  //foo.configure();
  return 0;
}
