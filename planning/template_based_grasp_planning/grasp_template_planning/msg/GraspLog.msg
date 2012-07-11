#boost uuid and Time
string uuid
time stamp

sensor_msgs/PointCloud2 target_object
geometry_msgs/Pose table_frame
GraspAnalysis applied_grasp
GraspAnalysis matched_grasp

int32 rank

# matching scores (sorted by rank)
float64[] matching_scores

# uuids of matched grasps
string[] candidate_to_match