Header header

# the setup id used to generate the prediction
int64 setup_id

# prediction results
std_msgs/String asm_id_current
float32 asm_current_score
float32 asm_current_failure
float32 asm_current_progress

std_msgs/String asm_id_successor
float32 asm_successor_score
float32 asm_successor_failure
float32 asm_successor_progress

std_msgs/String[] predictor_data_names
float32[] predictor_data_values

task_recorder2_msgs/DataSample data_samples_original
task_recorder2_msgs/DataSample data_samples_predicted

float32 progress 

int32 return_code
int32 FAIL_PREDICT_SETUP				=1
int32 FAIL_PREDICT_FILTER_DATA			=2
int32 FAIL_PREDICT_PREDICTOR			=3
int32 FAIL_FILTER						=4

int32 SUCCESS							=0