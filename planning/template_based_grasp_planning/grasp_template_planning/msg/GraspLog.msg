#boost uuid and Time
string uuid
time stamp
int32 seq

sensor_msgs/PointCloud2 target_object
geometry_msgs/Pose table_frame
GraspAnalysis applied_grasp
GraspAnalysis matched_grasp

int32 rank

# matching scores (sorted by rank)
float64[] matching_scores

# uuids of matched grasps sorted bu rank of candidate
string[] candidate_to_match

# candidates sorted by rank
GraspAnalysis[] candidates

# mask[i] = 1 => candidates[i] was considered for execution
bool[] mask