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

#rosbuild_genmsg()
#rosbuild_gensrv()

#common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME}
  src/dynamic_movement_primitive_utilities.cpp
  src/dynamic_movement_primitive_learner_utilities.cpp
  src/dynamic_movement_primitive_controller_client.cpp
  src/trajectory_utilities.cpp	
)

rosbuild_add_executable(dynamic_movement_primitive_utilities_test
  test/dynamic_movement_primitive_utilities_test.cpp
)

rosbuild_declare_test(dynamic_movement_primitive_utilities_test)
target_link_libraries(dynamic_movement_primitive_utilities_test gtest)
target_link_libraries(dynamic_movement_primitive_utilities_test ${PROJECT_NAME})
rosbuild_add_rostest(launch/arm_dynamic_movement_primitive_utilities_test.test)

# rosbuild_add_rostest(launch/pr2_dynamic_movement_primitive_utilities_test.test)
