#./bin/main_data_grasp_database --dir-path-bagfiles data/log_archive/grasping_experiments_data/Experiments_Dec/learning_second_round/ --dir-path-database data/deep_learning/grasp-database/ --database-name arm-grasp

#roslaunch ./launch/main_data_grasp_supervised_grasp_log_dec_first.launch 
#roslaunch ./launch/main_data_grasp_supervised_grasp_log_dec_second.launch 

#roslaunch ./launch/main_data_grasp_supervised_grasp_analysis_dec_grasping_retry.launch 
#roslaunch ./launch/main_data_grasp_supervised_grasp_analysis_dec_learned_failes.launch 


./scripts/deep_learning/dataset_template_supervised_grasping.py --dir-path-source data/deep_learning/grasp-dataset-supervised-dec-first-run/ --dir-path-destination data/deep_learning/grasp-dataset-supervised-dec-first-run/ --file-name learning-first-run --fast False
./scripts/deep_learning/dataset_template_supervised_grasping.py --dir-path-source data/deep_learning/grasp-dataset-supervised-dec-learning-second-round/ --dir-path-destination data/deep_learning/grasp-dataset-supervised-dec-learning-second-round/ --file-name learning-second-round --fast False

./scripts/deep_learning/dataset_template_supervised_grasping.py --dir-path-source data/deep_learning/grasp-dataset-supervised-dec-grasping-retry/ --dir-path-destination data/deep_learning/grasp-dataset-supervised-dec-grasping-retry/ --file-name grasping-with-retry-failures --fast False
./scripts/deep_learning/dataset_template_supervised_grasping.py --dir-path-source data/deep_learning/grasp-dataset-supervised-dec-learned-failes/ --dir-path-destination data/deep_learning/grasp-dataset-supervised-dec-learned-failes/ --file-name learned-failes-second-round --fast False
