cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(svm_classifier
  src/svm_classifier.cpp
  src/svm_parameters.cpp
  src/svm_trainer.cpp
  src/svm_io.cpp
  src/modelselection_grid_search_kernel.cpp
  src/cross_validator.cpp
  src/data_sample_filter.cpp
)
target_link_libraries(svm_classifier shogun)
rosbuild_add_openmp_flags(svm_classifier)

rosbuild_add_library(task_event_detector
  src/detector.cpp
  src/event_monitor.cpp
  src/task_event_detector.cpp
  src/task_event_detector_client.cpp
)
target_link_libraries(task_event_detector svm_classifier)

rosbuild_add_executable(task_event_detector_node
  src/task_event_detector_node.cpp
)
target_link_libraries(task_event_detector_node task_event_detector)

rosbuild_add_executable(cross_validator_node
  src/cross_validator_node.cpp
)
target_link_libraries(cross_validator_node task_event_detector)

rosbuild_add_executable(svm_trainer_node
  src/svm_trainer_node.cpp
)
target_link_libraries(svm_trainer_node task_event_detector)

rosbuild_add_executable(modelselection_grid_search_kernel_node
  src/modelselection_grid_search_kernel_node.cpp
)
target_link_libraries(modelselection_grid_search_kernel_node task_event_detector)

rosbuild_add_executable(test_svm_classifier_node
  src/data_sample_filter.cpp
  test/test_svm_classifier.cpp
)
target_link_libraries(test_svm_classifier_node svm_classifier)

rosbuild_add_executable(test_task_event_detector_client_node
  test/test_task_event_detector_client.cpp
)
target_link_libraries(test_task_event_detector_client_node task_event_detector)
