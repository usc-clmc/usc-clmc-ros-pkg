rosparam set /main_dataset_from_database/dir_path_database data/deep_learning/grasp-database/
rosparam set /main_dataset_from_database/database_name dl_grasp_database

rosparam set /main_dataset_from_database/dataset_name database-dataset
roslaunch grasp_template_planning main_dataset_from_database.launch


