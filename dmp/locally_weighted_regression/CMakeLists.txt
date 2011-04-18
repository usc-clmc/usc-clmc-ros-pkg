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

# Add include directories for the external lwr lib
include_directories( "lwrLib/include/" )
link_directories( ${PROJECT_SOURCE_DIR}/lwrLib/lib/ )

# Make the external lwr lib
rosbuild_add_library(lwr
	lwrLib/src/lwr.cpp
	lwrLib/src/lwr_parameters.cpp
	lwrLib/src/logger.cpp
)

# Make the test program for the external lib
add_executable(../lwrLib/test/lwr_test
  lwrLib/test/test_locally_weigthed_regression.cpp
)
target_link_libraries(../lwrLib/test/lwr_test lwr)

rosbuild_genmsg()

#common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME}
	src/locally_weighted_regression.cpp
)
target_link_libraries(${PROJECT_NAME} lwr)

rosbuild_link_boost(${PROJECT_NAME} thread signals system filesystem)

rosbuild_add_executable(locally_weighted_regression_test test/locally_weighted_regression_test.cpp)
rosbuild_declare_test(locally_weighted_regression_test)
target_link_libraries(locally_weighted_regression_test gtest)
target_link_libraries(locally_weighted_regression_test ${PROJECT_NAME})
rosbuild_add_rostest(launch/locally_weighted_regression_test.test)
