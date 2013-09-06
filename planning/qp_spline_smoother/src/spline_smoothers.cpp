#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <qp_spline_smoother/quintic_optimized_spline_smoother.h>

//  PLUGINLIB_EXPORT_CLASS( class_type, filters::FilterBase<T>)

//PLUGINLIB_EXPORT_CLASS( qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectory::Request>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory::Request>)
//
//PLUGINLIB_EXPORT_CLASS( qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::JointTrajectoryWithLimits>, filters::FilterBase<arm_navigation_msgs::JointTrajectoryWithLimits>)
//
//PLUGINLIB_EXPORT_CLASS( qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>, filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request>)

PLUGINLIB_DECLARE_CLASS(qp_spline_smoother, QuinticOptimizedSplineSmootherFilterJointTrajectory,
                          qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectory>,
                          filters::FilterBase<arm_navigation_msgs::FilterJointTrajectory>)

/*
PLUGINLIB_DECLARE_CLASS(qp_spline_smoother, QuinticOptimizedSplineSmootherJointTrajectoryWithLimits,
                          qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::JointTrajectoryWithLimits>,
                          filters::FilterBase<arm_navigation_msgs::JointTrajectoryWithLimits>)
*/

PLUGINLIB_DECLARE_CLASS(qp_spline_smoother, QuinticOptimizedSplineSmootherFilterJointTrajectoryWithConstraints,
                          qp_spline_smoother::QuinticOptimizedSplineSmoother<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
                          filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>)
