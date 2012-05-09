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

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories( "dmpLib/include/" )
link_directories( ${PROJECT_SOURCE_DIR}/dmpLib/lib/ )

rosbuild_genmsg()
#rosbuild_gensrv()

#add_custom_target(
#   extern_lib
#   COMMAND make
#   ${PROJECT_SOURCE_DIR}/lwrLib
#)

#add_custom_target(
#   extern_lib
#   COMMAND make
#   ${PROJECT_SOURCE_DIR}/dmpLib
#)

rosbuild_add_library(dmp++

	dmpLib/src/dynamic_movement_primitive.cpp
	dmpLib/src/dynamic_movement_primitive_parameters.cpp
	dmpLib/src/dynamic_movement_primitive_state.cpp

	dmpLib/src/transformation_system.cpp
	dmpLib/src/transformation_system_parameters.cpp
	dmpLib/src/transformation_system_state.cpp

	dmpLib/src/canonical_system.cpp
	dmpLib/src/canonical_system_parameters.cpp
	dmpLib/src/canonical_system_state.cpp

	dmpLib/src/trajectory.cpp
	dmpLib/src/logger.cpp

	dmpLib/src/icra2009_dynamic_movement_primitive.cpp
	dmpLib/src/icra2009_dynamic_movement_primitive_parameters.cpp
	dmpLib/src/icra2009_dynamic_movement_primitive_state.cpp

	dmpLib/src/icra2009_transformation_system.cpp
	dmpLib/src/icra2009_transformation_system_parameters.cpp
	dmpLib/src/icra2009_transformation_system_state.cpp

	dmpLib/src/icra2009_canonical_system.cpp
	dmpLib/src/icra2009_canonical_system_parameters.cpp
	dmpLib/src/icra2009_canonical_system_state.cpp

	dmpLib/src/nc2010_dynamic_movement_primitive.cpp
	dmpLib/src/nc2010_dynamic_movement_primitive_parameters.cpp
	dmpLib/src/nc2010_dynamic_movement_primitive_state.cpp

	dmpLib/src/nc2010_transformation_system.cpp
	dmpLib/src/nc2010_transformation_system_parameters.cpp
	dmpLib/src/nc2010_transformation_system_state.cpp

	dmpLib/src/nc2010_canonical_system.cpp
	dmpLib/src/nc2010_canonical_system_parameters.cpp
	dmpLib/src/nc2010_canonical_system_state.cpp

)

# link against liblwr.a
target_link_libraries(dmp++ lwr)

add_executable(../dmpLib/test/dmp_test
  dmpLib/test/test_dynamic_movement_primitive.cpp
  dmpLib/test/test_trajectory.cpp
  dmpLib/test/test_data.cpp
  dmpLib/test/icra2009_test.cpp
)
target_link_libraries(../dmpLib/test/dmp_test dmp++)
rosbuild_link_boost(../dmpLib/test/dmp_test system filesystem) 

# common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME}

	src/dynamic_movement_primitive.cpp
	src/transformation_system.cpp
	src/canonical_system.cpp

	src/icra2009_dynamic_movement_primitive.cpp
	src/icra2009_transformation_system.cpp
	src/icra2009_canonical_system.cpp

	src/nc2010_dynamic_movement_primitive.cpp
	src/nc2010_transformation_system.cpp
	src/nc2010_canonical_system.cpp

)
target_link_libraries(${PROJECT_NAME} dmp++ lwr)
rosbuild_link_boost(${PROJECT_NAME} system filesystem) 

rosbuild_add_executable(dynamic_movement_primitive_test
  test/dynamic_movement_primitive_test.cpp
  dmpLib/test/test_trajectory.cpp
  dmpLib/test/test_data.cpp
  dmpLib/test/icra2009_test.cpp
)
rosbuild_declare_test(dynamic_movement_primitive_test)
target_link_libraries(dynamic_movement_primitive_test gtest)
target_link_libraries(dynamic_movement_primitive_test ${PROJECT_NAME})
rosbuild_add_rostest(launch/dynamic_movement_primitive_test.test)
rosbuild_link_boost(dynamic_movement_primitive_test system filesystem)
