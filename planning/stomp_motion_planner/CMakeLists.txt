cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_definitions(-msse -msse2)
add_definitions(-DEIGEN_NO_DEBUG)
add_definitions(-DNDEBUG)
add_definitions(-DEIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API)

rosbuild_add_library(stomp_motion_planner_lib
	src/constraint_evaluator.cpp
	src/covariant_trajectory_policy.cpp
	src/policy_improvement_loop.cpp
	src/policy_improvement.cpp
	src/stomp_collision_point.cpp
	src/stomp_collision_space.cpp
	src/stomp_cost.cpp
	src/stomp_optimizer.cpp
	src/stomp_parameters.cpp
	src/stomp_planner_node.cpp
	src/stomp_robot_model.cpp
	src/stomp_trajectory.cpp
	src/treefksolverjointposaxis.cpp
	src/treefksolverjointposaxis_partial.cpp
)	

rosbuild_add_executable(stomp_motion_planner
	src/stomp_planner_node.cpp
)
target_link_libraries(stomp_motion_planner stomp_motion_planner_lib)

#rosbuild_add_executable(test_omp
#	test/mesh_collision_object_reader.cpp
#	test/test_omp.cpp
#	test/test_collision_world.cpp
#)


#rosbuild_add_executable(stomp_cost_server
#	src/stomp_cost_server.cpp
#)
#target_link_libraries(stomp_cost_server stomp)
