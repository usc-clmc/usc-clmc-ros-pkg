int32 svm_lib
int32 CT_LIGHT = 10
int32 CT_LIGHTONECLASS = 11
int32 CT_LIBSVM = 20
int32 CT_LIBSVMONECLASS = 30

int32 kernel_type
int32 K_GAUSSIAN = 30
int32 K_LINEAR = 10
int32 K_POWER = 410
		
float64 kernel_width
float64 svm_c
float64 svm_eps

float64 classification_boundary
int32 num_variables
int32 num_data_samples
int32[] indices
string[] variable_names

task_event_detector/GridSearchParametersMsg grid_search_parameters
