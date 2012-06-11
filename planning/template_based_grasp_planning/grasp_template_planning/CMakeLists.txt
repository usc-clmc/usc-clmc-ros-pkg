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

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME}
  src/demonstration_parser.cpp
  src/grasp_planning_params.cpp
  src/grasp_demo_library.cpp
  src/planning_pipeline.cpp
  src/visualization.cpp
  src/grasp_pool.cpp
  src/demo_writer.cpp
  src/template_matching.cpp
  src/object_detection_listener.cpp
)

rosbuild_add_openmp_flags(${PROJECT_NAME})

rosbuild_add_executable(user_demonstration_recorder
  src/user_demonstration_recorder.cpp
)
target_link_libraries(user_demonstration_recorder ${PROJECT_NAME})

rosbuild_add_executable(palm_marker
  config/palm_marker.cpp
)
target_link_libraries(palm_marker ${PROJECT_NAME})

rosbuild_add_executable(generate_grasp_library
  src/generate_grasp_library.cpp
)
target_link_libraries(generate_grasp_library ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
