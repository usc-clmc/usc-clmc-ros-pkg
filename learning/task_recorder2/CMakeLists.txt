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

# rosbuild_genmsg()
rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(${PROJECT_NAME}
	src/joint_states_recorder.cpp
	src/pose_recorder.cpp
	src/audio_recorder.cpp
	src/tf_recorder.cpp
	src/imu_recorder.cpp
)

rosbuild_add_executable(task_recorder_node
  src/task_recorder_node.cpp
)
target_link_libraries(task_recorder_node ${PROJECT_NAME})

rosbuild_add_executable(task_recorder_node
  src/task_recorder_node.cpp
)
target_link_libraries(task_recorder_node ${PROJECT_NAME})

rosbuild_add_library(task_recorder2_manager
  src/task_recorder_manager.cpp
)
rosbuild_add_executable(task_recorder_manager_node
  src/task_recorder_manager_node.cpp
)
target_link_libraries(task_recorder_manager_node ${PROJECT_NAME} task_recorder2_manager)

rosbuild_add_library(task_recorder2_manager_client
  src/task_recorder_manager_client.cpp
)

rosbuild_add_executable(task_labeler_node
  src/task_labeler_node.cpp
)

rosbuild_add_executable(test_task_recorder_manager_node
  test/test_task_recorder_manager.cpp
)
target_link_libraries(test_task_recorder_manager_node ${PROJECT_NAME} task_recorder2_manager task_recorder2_manager_client)

rosbuild_add_executable(test_task_recorder_node
  test/test_task_recorder_node.cpp
)
target_link_libraries(test_task_recorder_node ${PROJECT_NAME})

rosbuild_add_library(bag_file_recorder_manager
  src/bag_file_recorder_manager.cpp
)
