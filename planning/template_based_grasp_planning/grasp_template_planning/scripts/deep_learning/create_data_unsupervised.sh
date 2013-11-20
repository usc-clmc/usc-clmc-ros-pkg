rosparam set /main_dataset_unsupervised/dir_path_database data/deep_learning/grasp-database/
rosparam set /main_dataset_unsupervised/database_name dl_grasp_database

rosparam set /main_dataset_unsupervised/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/ICRA_MATCHING/After_Learning_1/log_data
rosparam set /main_dataset_unsupervised/dir_path_dataset data/deep_learning/dataset-unsupervised/
rosparam set /main_dataset_unsupervised/dataset_name icra-after-learning
roslaunch grasp_template_planning main_dataset_unsupervised.launch


#rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/ICRA_MATCHING/Learning_Spraycan/log_data
#roslaunch grasp_template_planning main_dataset_unsupervised.launch

#rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/ICRA_MATCHING/Without_learning_2/log_data
#roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/ICRA_MATCHING/Without_learning_Spraycan/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/After_Learning_Mine_Broke_up/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch

#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/After_Learning_Mine_Broke_up/library_negatives
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/Learning_Spraycan_1/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/Learning_Spraycan_2/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/Learning_Spraycan_3_Only_Fails/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch


#   rosparam set /main_dataset_unsupervised.launch/dir_path_bagfiles data/migrated_logs/12-07-PR2-Template-Grasping-Experiment-Logs/PARTIALLY_NORMED_DISTANCE/Without_Learning_Mine_1/log_data
#   roslaunch grasp_template_planning main_dataset_unsupervised.launch

