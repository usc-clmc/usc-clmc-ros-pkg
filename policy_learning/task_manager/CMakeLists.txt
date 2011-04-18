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

rosbuild_add_library(task_manager
	src/task_manager.cpp
	)

#rosbuild_add_executable(task_regression_test
#	test/my_test_task.cpp)

rosbuild_add_library(task_test
	test/my_test_task.cpp)

rosbuild_add_executable(task_regression_test
	test/task_manager_test.cpp)

target_link_libraries(task_regression_test task_manager)

rosbuild_add_gtest_build_flags(task_regression_test)
rosbuild_add_rostest(launch/task_test.test)
