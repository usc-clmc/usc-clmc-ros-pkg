cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE Debug)

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
  recorders/joint_states_recorder.cpp
  recorders/pose_recorder.cpp
  recorders/audio_recorder.cpp
)

rosbuild_add_library(task_recorder2_manager
  src/task_recorder_manager.cpp
  src/interrupt_handler.cpp
)
rosbuild_add_openmp_flags(task_recorder2_manager)

rosbuild_add_executable(task_recorder_manager_node
  src/task_recorder_manager_node.cpp
)
target_link_libraries(task_recorder_manager_node ${PROJECT_NAME} task_recorder2_manager)

rosbuild_add_executable(test_task_recorder_node
  test/test_task_recorder_node.cpp
)
target_link_libraries(test_task_recorder_node ${PROJECT_NAME})

