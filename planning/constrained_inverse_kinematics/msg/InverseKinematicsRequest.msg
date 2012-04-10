# link for which IK is required
# NOT USED CURRENTLY
string link_name

# pose of the tool attached to the link.
# All IK will be performed for the tool frame
geometry_msgs/Pose link_to_tool_pose

# Desired tool pose
geometry_msgs/Pose desired_tool_pose

# end-effector position constraint
bool use_position_constraint
arm_navigation_msgs/Shape position_constraint_shape
geometry_msgs/Quaternion position_constraint_orientation

# end-effector orientation constraint
bool use_orientation_constraint
# tolerance in rotations about x, y, and z axes
float64[] orientation_constraint_angular_tolerance
