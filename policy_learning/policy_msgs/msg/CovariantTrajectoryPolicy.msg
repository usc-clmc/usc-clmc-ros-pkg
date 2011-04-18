float64 discretization_interval
string[] dimension_names

# points[] must contain the start and goal also
policy_msgs/TrajectoryPoint[] points

# array of size 3, containinrg vel, acc and jerk cost weights
float64[] derivative_costs

float64 cost_ridge_factor

geometry_msgs/Pose nominal_start_pose
