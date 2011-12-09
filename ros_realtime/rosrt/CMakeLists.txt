cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Check for Xenomai:
set(XENOMAI_SEARCH_PATH /usr/local/xenomai /usr/xenomai /usr)
find_path(XENOMAI_DIR bin/xeno-config ${XENOMAI_SEARCH_PATH})
if (XENOMAI_DIR)
  set (XENOMAI_CONFIG ${XENOMAI_DIR}/bin/xeno-config)
  message ("Xenomai found in ${XENOMAI_DIR}")
  execute_process(COMMAND ${XENOMAI_CONFIG} --skin=native --cflags OUTPUT_VARIABLE XENOMAI_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND ${XENOMAI_CONFIG} --skin=native --ldflags OUTPUT_VARIABLE XENOMAI_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set (ROSRT_PLATFORM_CFLAGS ${XENOMAI_CFLAGS})
  set (ROSRT_PLATFORM_LDFLAGS ${XENOMAI_LDFLAGS})
  #add_definitions(${XENOMAI_CFLAGS})
endif (XENOMAI_DIR)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(include)

#add platform-specific compile flags
add_definitions(${ROSRT_PLATFORM_CFLAGS})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/malloc.cpp src/simple_gc.cpp src/publisher.cpp src/subscriber.cpp src/init.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

#add platform-specific linker flags
target_link_libraries(${PROJECT_NAME} ${ROSRT_PLATFORM_LDFLAGS})

rosbuild_add_executable(test_publisher EXCLUDE_FROM_ALL test/test_publisher.cpp)
rosbuild_add_gtest_build_flags(test_publisher)
target_link_libraries(test_publisher ${PROJECT_NAME})
rosbuild_add_rostest(test/test_publisher.xml)

#
rosbuild_add_executable(test_subscriber EXCLUDE_FROM_ALL  test/test_subscriber.cpp)
target_link_libraries(test_subscriber ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_subscriber)
rosbuild_add_rostest(test/test_subscriber.xml)

rosbuild_add_executable(test_filtered_subscriber EXCLUDE_FROM_ALL  test/test_filtered_subscriber.cpp)
target_link_libraries(test_filtered_subscriber ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_filtered_subscriber)
rosbuild_add_rostest(test/test_filtered_subscriber.xml)

rosbuild_add_gtest(test_malloc_wrappers test/test_malloc_wrappers.cpp)
target_link_libraries(test_malloc_wrappers ${PROJECT_NAME})

rosbuild_add_library(test_malloc_wrappers_so EXCLUDE_FROM_ALL test/test_malloc_wrappers_so.cpp)
rosbuild_declare_test(test_malloc_wrappers_so)
