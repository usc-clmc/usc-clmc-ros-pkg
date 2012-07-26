#boost uuid and Time
string uuid
time stamp

#reference to demonstration
string demo_id
string demo_filename

float64 grasp_success

#	Viewpoint Transform
geometry_msgs/PoseStamped viewpoint_transform

#  GraspTemplate
grasp_template/Heightmap grasp_template
geometry_msgs/PoseStamped template_pose

#  Gripper
geometry_msgs/PoseStamped gripper_pose
float64 gripper_joint_state

#  Additional Gripper Info for specific architectures
DoubleVector fingerpositions
