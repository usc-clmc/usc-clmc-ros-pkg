cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/cost_features/cartesian_orientation_feature.cpp
  src/cost_features/cartesian_vel_acc_feature.cpp
  src/cost_features/collision_feature.cpp
  src/cost_features/exact_collision_feature.cpp
  src/cost_features/joint_vel_acc_feature.cpp
  src/stomp_collision_point.cpp
  src/stomp_collision_space.cpp
  src/stomp_cost_function_input.cpp
  src/stomp_optimization_task.cpp
  src/stomp_robot_model.cpp
  src/treefksolverjointposaxis_partial.cpp
)

rosbuild_add_executable(display_robot_model
  src/display_robot_model.cpp
)

target_link_libraries(display_robot_model ${PROJECT_NAME})

rosbuild_add_executable(stomp_node
  src/stomp_node.cpp
)

target_link_libraries(stomp_node ${PROJECT_NAME})

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
